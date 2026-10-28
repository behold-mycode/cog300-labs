#!/bin/zsh
# Double-click on macOS, or run from Terminal. No Tk installation is needed.
set -e
cd -- "${0:A:h}/controller"
trap 'print "\nLaunch failed. See the message above. Press Return to close."; read' ZERR
if ! command -v python3 >/dev/null 2>&1; then
  print 'Python 3 is missing. Install Python 3.9 or newer from https://www.python.org/downloads/macos/ and run this launcher again.'
  exit 1
fi
if ! python3 -c 'import sys; sys.exit(sys.version_info < (3, 9))'; then
  print 'Python 3.9 or newer is required. Update Python from python.org.'
  exit 1
fi
if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv .venv
fi
if ! .venv/bin/python -c 'import serial' >/dev/null 2>&1; then
  print 'Installing pyserial into this project’s .venv (first launch needs internet)…'
  .venv/bin/python -m pip install --disable-pip-version-check -r requirements.txt
fi
.venv/bin/python controller.py "$@"
