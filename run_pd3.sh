#!/usr/bin/env bash
# PD #3 launcher: makes sure pymavlink is available, then runs the mission.
# Start the simulator first:  cd ~/Drone_Sim_Docker && make run
set -euo pipefail
cd "$(dirname "$0")"

if python3 -c "import pymavlink" 2>/dev/null; then
    PY=python3
else
    if [[ ! -x .venv/bin/python ]]; then
        echo "Creating .venv and installing pymavlink (one time)..."
        python3 -m venv .venv
        .venv/bin/pip install -q pymavlink
    fi
    PY=.venv/bin/python
fi

exec "$PY" pd3_mavlink_control.py "$@"
