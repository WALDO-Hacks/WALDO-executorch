"""Small depthwise-separable CNN for "Hey Waldo" on the firmware's 49x10 MFCC window.

Input NHWC (N, 49, 10, 1) float MFCCs (model/kws/mfcc.py), output (N, 2) logits [other, wake].
Same layer vocabulary as MicroNet-M (conv, depthwise conv, 1x1 conv, ReLU6, global average
pool; TensorFlow 'SAME' padding) so all operators land on the Ethos-U. `deploy()` folds each
BatchNorm into the preceding convolution for export.
"""
import copy
import sys
from pathlib import Path

import torch
import torch.nn.functional as F
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "kws"))
from micronet import _same_pad  # noqa: E402


class WakeNet(nn.Module):
    def __init__(self, width=32, blocks=4, use_bn=True, dropout=0.1):
        super().__init__()
        self.spec = [(10, 4, 2)] + [(3, 3, 1), (1, 1, 1)] * blocks   # (kh, kw, stride) per conv
        chans = [1, width]
        groups = [1]
        for _ in range(blocks):
            chans += [width, width]
            groups += [width, 1]
        self.convs = nn.ModuleList()
        self.bns = nn.ModuleList()
        for i, (kh, kw, s) in enumerate(self.spec):
            self.convs.append(nn.Conv2d(chans[i], chans[i + 1], (kh, kw), stride=s, groups=groups[i], bias=not use_bn))
            self.bns.append(nn.BatchNorm2d(chans[i + 1]) if use_bn else nn.Identity())
        self.drop = nn.Dropout(dropout)
        self.fc = nn.Conv2d(width, 2, 1)

    def forward(self, x):
        x = x.permute(0, 3, 1, 2)   # NHWC -> NCHW
        for conv, bn, (kh, kw, s) in zip(self.convs, self.bns, self.spec):
            t, b = _same_pad(x.shape[2], kh, s)
            l, r = _same_pad(x.shape[3], kw, s)
            x = torch.clamp(bn(conv(F.pad(x, (l, r, t, b)))), 0.0, 6.0)
        x = self.drop(x.mean(dim=(2, 3), keepdim=True))
        return self.fc(x).flatten(1)

    def deploy(self):
        """Copy with BatchNorm folded into the convolutions (eval mode, for export)."""
        m = copy.deepcopy(self).eval()
        for i, (conv, bn) in enumerate(zip(m.convs, m.bns)):
            if isinstance(bn, nn.Identity):
                continue
            fused = nn.utils.fusion.fuse_conv_bn_eval(conv, bn)
            m.convs[i], m.bns[i] = fused, nn.Identity()
        return m


def n_params(model):
    return sum(p.numel() for p in model.parameters())
