#!/usr/bin/env python3

import torch
import torch.nn as nn

class GSConv(nn.Module):
    """
    GSConv (Group Shuffle Convolution) module as described in the paper:
    "A method of dense point cloud SLAM based on improved YOLOV8 and fused with ORB-SLAM3 to cope with dynamic environments"
    
    This module combines standard convolution with depthwise separable convolution to balance
    accuracy and computational complexity.
    """
    def __init__(self, c1, c2, k=1, s=1, p=None, g=1, act=True):
        super().__init__()
        c_ = c2 // 2  # Half the output channels for standard conv
        self.cv1 = nn.Conv2d(c1, c_, k, s, autopad(k, p), g, bias=False)
        self.cv2 = nn.Conv2d(c_, c_, 3, 1, 1, c_, bias=False)  # Depthwise conv
        self.act = nn.SiLU() if act else nn.Identity()
        
    def forward(self, x):
        x1 = self.cv1(x)
        x2 = self.cv2(x1)
        # Concatenate and shuffle channels
        return self.act(torch.cat((x1, x2), 1))


class VoVGSCSP(nn.Module):
    """
    VoVGSCSP module based on GSConv, as described in the paper.
    This replaces the C2f module in YOLOv8 Neck structure.
    """
    def __init__(self, c1, c2, n=1, shortcut=False, g=1, e=0.5):
        super().__init__()
        c_ = int(c2 * e)  # Hidden channels
        self.cv1 = nn.Conv2d(c1, c_, 1, 1)
        self.cv2 = GSConv(c_, c_, 3, 1)
        self.cv3 = GSConv(c_, c_, 3, 1)
        self.cv4 = nn.Conv2d(2 * c_, c2, 1, 1)
        
    def forward(self, x):
        x1 = self.cv1(x)
        x2 = self.cv2(x1)
        x3 = self.cv3(x2)
        return self.cv4(torch.cat((x2, x3), 1))


def autopad(k, p=None):
    """Calculate padding to maintain same dimensions."""
    if p is None:
        p = k // 2 if isinstance(k, int) else [x // 2 for x in k]
    return p