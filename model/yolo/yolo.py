"""YOLOv8n for the Ethos-U85: the network up to the head's raw maps, 28 classes.

From the team's vision hand-off (host/export/yolo_wrapper.py): Ultralytics' pretrained
COCO yolov8n with the final 1x1 class conv of each head branch sliced to the 28 classes
in classes.py (the kept rows' weights are unchanged). No DFL, anchors, sigmoid or NMS in
the graph, so it is convolutions, SiLU, concat, upsample and max-pool only, all on the NPU;
the Cortex-M55 decodes (src/waldo/yolo_decode.h).

Input NCHW (1, 3, 320, 320) RGB / 255, letterboxed with grey 114. Outputs three maps
(1, 64 + 28, H, W), H = W = 40, 20, 10 (strides 8, 16, 32).
YOLOv8 is AGPL-3.0 (Ultralytics).
"""
import torch
from torch import nn

from classes import KEPT_IDS

IMGSZ = 320


class RawYolo(nn.Module):
    def __init__(self, weights="yolov8n.pt", classes=KEPT_IDS):
        super().__init__()
        from ultralytics import YOLO
        self.net = YOLO(weights).model.float().eval()
        self.head = self.net.model[-1]
        idx = torch.as_tensor(list(classes), dtype=torch.long)
        for seq in self.head.cv3:
            old = seq[-1]
            new = nn.Conv2d(old.in_channels, len(idx), 1, bias=old.bias is not None)
            with torch.no_grad():
                new.weight.copy_(old.weight[idx])
                if old.bias is not None:
                    new.bias.copy_(old.bias[idx])
            seq[-1] = new

    def forward(self, x):
        y = []
        for m in self.net.model[:-1]:
            if m.f != -1:
                x = y[m.f] if isinstance(m.f, int) else [x if j == -1 else y[j] for j in m.f]
            x = m(x)
            y.append(x if m.i in self.net.save else None)
        feats = [y[j] if j != -1 else x for j in self.head.f]
        return tuple(torch.cat((self.head.cv2[i](f), self.head.cv3[i](f)), 1) for i, f in enumerate(feats))
