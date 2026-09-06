#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import random
import time

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Subset

from learning.dataset import EpisodeDataset
from learning.model import VisualBCPolicy


def parse_args():
    parser = argparse.ArgumentParser(description="Train the native visual behavioral-cloning policy")
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--width", type=int, default=160)
    parser.add_argument("--height", type=int, default=120)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    return parser.parse_args()


def main():
    args = parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device_name = "cuda" if args.device == "auto" and torch.cuda.is_available() else args.device
    if device_name == "auto":
        device_name = "cpu"
    if device_name == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but torch.cuda.is_available() is false")
    device = torch.device(device_name)

    dataset = EpisodeDataset(args.dataset, args.width, args.height)
    episodes = sorted(dataset.episode_to_indices)
    shuffled_episodes = list(episodes)
    random.Random(args.seed).shuffle(shuffled_episodes)
    if len(shuffled_episodes) >= 2:
        val_episode_count = min(
            len(shuffled_episodes) - 1,
            max(1, round(len(shuffled_episodes) * 0.2)),
        )
        val_episodes = set(shuffled_episodes[:val_episode_count])
        train_indices = [
            index for episode in episodes if episode not in val_episodes
            for index in dataset.episode_to_indices[episode]
        ]
        val_indices = [
            index for episode in episodes if episode in val_episodes
            for index in dataset.episode_to_indices[episode]
        ]
        dataset.set_normalization(train_indices)
        train_set, val_set = Subset(dataset, train_indices), Subset(dataset, val_indices)
    else:
        val_episodes = set()
        train_set, val_set = dataset, None
    train_loader = DataLoader(
        train_set, batch_size=args.batch_size, shuffle=True, num_workers=2,
        pin_memory=device.type == "cuda",
    )
    val_loader = DataLoader(val_set, batch_size=args.batch_size) if val_set else None

    state_dim = dataset.states.shape[1]
    action_dim = dataset.actions.shape[1]
    model = VisualBCPolicy(state_dim, action_dim).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.learning_rate, weight_decay=1e-4)
    loss_fn = nn.SmoothL1Loss()
    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    log_path = output / "training.jsonl"
    best_loss = float("inf")

    def evaluate():
        if val_loader is None:
            return None
        model.eval()
        total = 0.0
        count = 0
        with torch.no_grad():
            for image, state, action in val_loader:
                image, state, action = image.to(device), state.to(device), action.to(device)
                total += loss_fn(model(image, state), action).item() * len(image)
                count += len(image)
        return total / max(1, count)

    for epoch in range(1, args.epochs + 1):
        model.train()
        total = 0.0
        count = 0
        for image, state, action in train_loader:
            image, state, action = image.to(device), state.to(device), action.to(device)
            optimizer.zero_grad(set_to_none=True)
            loss = loss_fn(model(image, state), action)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimizer.step()
            total += loss.item() * len(image)
            count += len(image)
        train_loss = total / max(1, count)
        val_loss = evaluate()
        metric = val_loss if val_loss is not None else train_loss
        row = {"epoch": epoch, "train_loss": train_loss, "val_loss": val_loss, "time": time.time()}
        with log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(row) + "\n")
        print(json.dumps(row), flush=True)
        checkpoint = {
            "format": "biped-bike-visual-bc-v1",
            "model_state": model.state_dict(),
            "state_dim": state_dim,
            "action_dim": action_dim,
            "image_width": args.width,
            "image_height": args.height,
            "stats": dataset.stats(),
            "joint_names": dataset.metadata.get("action_joint_names", []),
            "dataset": str(dataset.root),
            "epoch": epoch,
            "training_episodes": [name for name in episodes if name not in val_episodes],
            "validation_episodes": sorted(val_episodes),
        }
        torch.save(checkpoint, output / "last.pt")
        if metric < best_loss:
            best_loss = metric
            torch.save(checkpoint, output / "best.pt")
    print(f"Training complete. Best model: {output / 'best.pt'}")


if __name__ == "__main__":
    main()
