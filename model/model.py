"""WALDO's model: BlazeFace face detection, quantized to int8 for the Ethos-U85 by create_ai_layer.py.

BlazeFace (MediaPipe's front-camera face detector, ~100k parameters) is all
convolutions, ReLU, pooling and padding, so it runs on the NPU. Two changes for WALDO:

- Grayscale input. The ESP32 head camera streams grayscale. Feeding the same value
  into R, G and B is the same as one input channel whose first-layer weights are the
  sum of the three, so the first convolution is folded to 1 input channel.
- One output tensor, (1, 896, 5): per anchor the raw face score (a logit) and the raw
  box offsets (x, y, w, h). The six facial keypoints are dropped. Decoding against the
  anchors and non-max suppression run on the Cortex-M55 (src/waldo/face_detector.cpp).

Input is NHWC (1, 128, 128, 1), pixels scaled to [-1, 1] (p / 127.5 - 1).
"""

from pathlib import Path

import numpy as np
import torch
from torch import nn

from blazeface.blazeface import BlazeFace

HERE = Path(__file__).resolve().parent
INPUT_SHAPE = (1, 128, 128, 1)


class WaldoFace(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        net = BlazeFace()
        net.load_state_dict(torch.load(HERE / "blazeface" / "blazeface.pth", map_location="cpu"))
        rgb = net.backbone1[0]
        gray = nn.Conv2d(1, rgb.out_channels, rgb.kernel_size, rgb.stride, rgb.padding, bias=True)
        with torch.no_grad():
            gray.weight.copy_(rgb.weight.sum(dim=1, keepdim=True))
            gray.bias.copy_(rgb.bias)
        net.backbone1[0] = gray
        self.net = net

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        boxes, scores = self.net(x.permute(0, 3, 1, 2))   # (1, 896, 16), (1, 896, 1)
        return torch.cat((scores, boxes[..., :4]), dim=-1)


def get_model(input_shape: tuple[int, ...] = INPUT_SHAPE) -> nn.Module:
    assert tuple(input_shape) == INPUT_SHAPE, f"BlazeFace front model takes {INPUT_SHAPE}"
    return WaldoFace().eval()


def get_calibration_inputs(
    input_shape: tuple[int, ...] = INPUT_SHAPE, calibration_samples: int = 12
) -> list[torch.Tensor]:
    """Face photos (grayscale, 128x128, with flips and brightness changes), scaled like on the board."""
    faces = np.load(HERE / "blazeface" / "calib" / "faces_gray128.npy")
    samples = []
    for i in range(calibration_samples):
        img = torch.from_numpy(faces[i % len(faces)].copy()).float() / 127.5 - 1.0
        samples.append(img.reshape(input_shape))
    return samples
