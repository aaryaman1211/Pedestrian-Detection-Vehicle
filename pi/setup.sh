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
echo "Pi environment ready."
echo "  source venv/bin/activate"
echo "  python main.py --no-serial          # vision test without ESP32"
echo "  python main.py --port /dev/ttyUSB0  # full run with UART"
echo "  python calibrate.py --distance 1.5  # calibrate distance k"
