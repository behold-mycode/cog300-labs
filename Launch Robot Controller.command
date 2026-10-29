#!/bin/zsh
# Works on Intel and Apple Silicon Macs without a preinstalled Python.
set -eu
set -o pipefail
cd -- "${0:A:h}/controller"
trap 'print "\nLaunch failed. Check internet access and the message above."; if [[ -t 0 ]]; then read "?Press Return to close."; fi' ZERR

# Project-local tooling: no sudo, Homebrew, Xcode or shell-profile changes.
export UV_PYTHON_INSTALL_DIR="$PWD/.runtime/python"
export UV_CACHE_DIR="$PWD/.runtime/cache"
uv_bin="$PWD/.runtime/bin/uv"
if [[ ! -x "$uv_bin" ]]; then
  print 'First launch: downloading the Python setup tool (internet required)…'
  mkdir -p .runtime/bin
  /usr/bin/curl --fail --location --silent --show-error \
    https://astral.sh/uv/0.13.6/install.sh -o .runtime/install-uv.sh
  UV_INSTALL_DIR="$PWD/.runtime/bin" UV_NO_MODIFY_PATH=1 /bin/sh .runtime/install-uv.sh
fi
if ! .venv/bin/python -c 'import sys; assert sys.version_info >= (3, 9)' >/dev/null 2>&1; then
  # Preserve an unusable/copied environment rather than deleting user files.
  if [[ -e .venv || -L .venv ]]; then
    mv .venv ".venv.old.$(/bin/date +%s).$$"
  fi
  print 'Preparing a private Python 3.12 runtime…'
  "$uv_bin" venv --managed-python --python 3.12 .venv
fi
if ! .venv/bin/python -c 'import serial; assert serial.VERSION == "3.5"' >/dev/null 2>&1; then
  print 'Installing the controller dependency…'
  "$uv_bin" pip install --python .venv/bin/python -r requirements.txt
fi
.venv/bin/python controller.py "$@"
