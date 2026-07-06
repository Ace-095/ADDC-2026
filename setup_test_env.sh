#!/usr/bin/env bash
# setup_test_env.sh — creates a separate test_venv and installs required dependencies

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "==> Creating separate test_venv..."
HOST_PYTHON="/home/ace/Downloads/apps_installation/miniconda3/bin/python3"
if [ -f "$HOST_PYTHON" ]; then
    echo "Using host python: $HOST_PYTHON"
    "$HOST_PYTHON" -m venv test_venv
else
    echo "Falling back to system python3"
    python3 -m venv test_venv
fi

echo "==> Upgrading pip in test_venv..."
./test_venv/bin/pip install --upgrade pip

echo "==> Installing required dependencies..."
./test_venv/bin/pip install pymavlink pyserial numpy opencv-python pyzbar PyYAML

echo ""
echo "✅ Separate test_venv created successfully!"
echo "Run the bench test with:"
echo "   ./test_venv/bin/python bench_test_alignment.py"
