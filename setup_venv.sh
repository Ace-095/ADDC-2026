#!/usr/bin/env bash
# setup_venv.sh — sets up the drone_pi virtual environment
# Run this from inside the drone_pi/ directory:
#   bash setup_venv.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "==> Removing any old venvs..."
rm -rf drone_venv .venv venv

echo "==> Creating drone_venv with python3..."
python3 -m venv drone_venv --system-site-packages

echo "==> Upgrading pip..."
./drone_venv/bin/pip install --upgrade pip

echo "==> Installing requirements from requirements.txt..."
./drone_venv/bin/pip install -r requirements.txt

echo ""
echo "✅ Done! Activate with:"
echo "   source drone_venv/bin/activate"
