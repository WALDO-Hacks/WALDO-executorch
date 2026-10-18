"""MicroNet-M keyword spotting (Arm ML-Zoo, Apache-2.0) rebuilt in PyTorch.

Arm publishes the trained network as an int8 TFLite file (kws_micronet_m.tflite). The
weights in micronet_m_weights.npz are that file's weights dequantised (tools: see
README), and this module is the same graph: conv 10x4 -> 5 x (depthwise 3x3 + pointwise
1x1), all ReLU6, TensorFlow 'SAME' padding -> global average pool -> 1x1 conv to 12 logits.

Input NHWC (1, 49, 10, 1): 49 MFCC frames x 10 coefficients (mfcc.py). Output (1, 12) logits.
"""
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from torch import nn

HERE = Path(__file__).resolve().parent
LABELS = (HERE / "micronet_kws_labels.txt").read_text().split()


def _same_pad(size, k, s):
    out = -(-size // s)
    total = max((out - 1) * s + k - size, 0)
    return total // 2, total - total // 2


class MicroNetM(nn.Module):
    def __init__(self, weights=HERE / "micronet_m_weights.npz"):
        super().__init__()
        w = np.load(weights)
        self.layers = nn.ModuleList()
        self.spec = []   # (kind, kernel, stride) per layer
        for i in range(12):
            wi, bi = w[f"w{i}"], w[f"b{i}"]
            if wi.shape[0] == 1:   # depthwise, 1HWC
                c, kh, kw = wi.shape[3], wi.shape[1], wi.shape[2]
                conv = nn.Conv2d(c, c, (kh, kw), groups=c)
                conv.weight.data = torch.from_numpy(wi.transpose(3, 0, 1, 2).astype(np.float32))
                stride = 2 if i == 1 else 1
            else:                  # OHWI
                o, kh, kw, ci = wi.shape
                conv = nn.Conv2d(ci, o, (kh, kw))
                conv.weight.data = torch.from_numpy(wi.transpose(0, 3, 1, 2).astype(np.float32))
                stride = 1
            conv.bias.data = torch.from_numpy(bi.astype(np.float32))
            conv.stride = (stride, stride)
            self.layers.append(conv)
            self.spec.append((kh, kw, stride))

    def forward(self, x):
        x = x.permute(0, 3, 1, 2)   # NHWC -> NCHW
        for i, (conv, (kh, kw, s)) in enumerate(zip(self.layers, self.spec)):
            if i == 11:   # classifier, after the global average pool
                x = conv(x.mean(dim=(2, 3), keepdim=True))
                return x.flatten(1)
            t, b = _same_pad(x.shape[2], kh, s)
            l, r = _same_pad(x.shape[3], kw, s)
            x = torch.clamp(conv(F.pad(x, (l, r, t, b))), 0.0, 6.0)   # ReLU6 (clamp: the Arm quantizer fuses it)
        return x
