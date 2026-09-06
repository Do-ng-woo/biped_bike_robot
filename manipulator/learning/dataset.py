from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset


def preprocess_image_bytes(data: bytes, width: int, height: int) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("failed to decode JPEG")
    image = cv2.cvtColor(cv2.resize(image, (width, height)), cv2.COLOR_BGR2RGB)
    return np.transpose(image.astype(np.float32) / 255.0, (2, 0, 1))


class EpisodeDataset(Dataset):
    def __init__(self, root: str | Path, width: int = 160, height: int = 120):
        self.root = Path(root).expanduser().resolve()
        metadata_path = self.root / "dataset.json"
        if not metadata_path.exists():
            raise FileNotFoundError(f"dataset.json not found: {metadata_path}")
        self.metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        self.width = int(width)
        self.height = int(height)
        self.records: list[tuple[Path, int, np.ndarray, np.ndarray]] = []
        self.episode_to_indices: dict[str, list[int]] = {}
        for episode in sorted(self.root.glob("episode_*")):
            sample_path = episode / "samples.npz"
            if not sample_path.exists():
                continue
            samples = np.load(sample_path, allow_pickle=False)
            states = np.asarray(samples["observation_state"], dtype=np.float32)
            actions = np.asarray(samples["action"], dtype=np.float32)
            frame_count = len(list((episode / "frames").glob("*.jpg")))
            count = min(len(states), len(actions), frame_count)
            for index in range(count):
                record_index = len(self.records)
                self.records.append((episode, index, states[index], actions[index]))
                self.episode_to_indices.setdefault(episode.name, []).append(record_index)
        if not self.records:
            raise ValueError(f"dataset has no samples: {self.root}")
        self.states = np.stack([item[2] for item in self.records])
        self.actions = np.stack([item[3] for item in self.records])
        self.set_normalization(range(len(self.records)))

    def set_normalization(self, indices) -> None:
        selected = np.asarray(list(indices), dtype=np.int64)
        if selected.size == 0:
            raise ValueError("normalization requires at least one training sample")
        states = self.states[selected]
        actions = self.actions[selected]
        self.state_mean = states.mean(axis=0)
        self.state_std = np.maximum(states.std(axis=0), 1e-4)
        self.action_mean = actions.mean(axis=0)
        self.action_std = np.maximum(actions.std(axis=0), 1e-4)

    def __len__(self):
        return len(self.records)

    def __getitem__(self, index):
        episode, frame_index, state, action = self.records[index]
        frame = (episode / "frames" / f"{frame_index:06d}.jpg").read_bytes()
        image = preprocess_image_bytes(frame, self.width, self.height)
        state = (state - self.state_mean) / self.state_std
        action = (action - self.action_mean) / self.action_std
        return (
            torch.from_numpy(image.copy()),
            torch.from_numpy(state.astype(np.float32)),
            torch.from_numpy(action.astype(np.float32)),
        )

    def stats(self) -> dict[str, list[float]]:
        return {
            "state_mean": self.state_mean.tolist(),
            "state_std": self.state_std.tolist(),
            "action_mean": self.action_mean.tolist(),
            "action_std": self.action_std.tolist(),
        }
