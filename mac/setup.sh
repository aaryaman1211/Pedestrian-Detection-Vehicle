#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if [[ ! -d venv ]]; then
  python3 -m venv venv
fi

source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo ""
echo "Mac simulator ready."
echo "  source venv/bin/activate"
echo "  python main.py                  # webcam + dashboard"
echo "  python main.py --video clip.mp4 # replay a recorded clip"
