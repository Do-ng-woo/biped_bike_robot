#!/usr/bin/env bash
set -euo pipefail

MANIPULATOR_ROOT="$(cd "$(dirname "$0")" && pwd)"
TRAINING_ENV="$MANIPULATOR_ROOT/.venv-training"

python3 -m venv --system-site-packages "$TRAINING_ENV"
"$TRAINING_ENV/bin/python" -m pip install --upgrade pip
"$TRAINING_ENV/bin/python" -m pip install -r "$MANIPULATOR_ROOT/requirements-training.txt"

echo "Training environment ready: $TRAINING_ENV"
"$TRAINING_ENV/bin/python" -c 'import cv2, rclpy, torch; print("torch", torch.__version__, "cuda", torch.cuda.is_available(), "ROS/OpenCV OK")'
