#!/bin/zsh
cd "${0:A:h}" || exit 1
export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"
exec python3 robot_update.py
