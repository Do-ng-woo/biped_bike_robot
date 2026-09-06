from __future__ import annotations

import torch
from torch import nn


class VisualBCPolicy(nn.Module):
    """Small image-and-joint behavioral-cloning policy."""

    def __init__(self, state_dim: int, action_dim: int):
        super().__init__()
        self.image_encoder = nn.Sequential(
            nn.Conv2d(3, 32, 5, stride=2, padding=2), nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Conv2d(64, 128, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.Conv2d(128, 128, 3, stride=2, padding=1), nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((1, 1)), nn.Flatten(),
            nn.Linear(128, 128), nn.ReLU(inplace=True),
        )
        self.state_encoder = nn.Sequential(
            nn.Linear(state_dim, 64), nn.ReLU(inplace=True),
            nn.Linear(64, 64), nn.ReLU(inplace=True),
        )
        self.head = nn.Sequential(
            nn.Linear(192, 256), nn.ReLU(inplace=True),
            nn.Dropout(0.1),
            nn.Linear(256, 128), nn.ReLU(inplace=True),
            nn.Linear(128, action_dim),
        )

    def forward(self, image: torch.Tensor, state: torch.Tensor) -> torch.Tensor:
        visual = self.image_encoder(image)
        proprio = self.state_encoder(state)
        return self.head(torch.cat((visual, proprio), dim=1))

