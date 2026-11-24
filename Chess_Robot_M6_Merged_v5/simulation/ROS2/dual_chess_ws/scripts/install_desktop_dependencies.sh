#!/usr/bin/env bash
# Workspace-local installation; no sudo, no system Python modification.
set -eo pipefail
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
deps="$ws/.qt-deps"
mkdir -p "$deps"
if ! PYTHONPATH="$deps${PYTHONPATH:+:$PYTHONPATH}" /usr/bin/python3 -c 'import PySide6; assert PySide6.__version__ == "6.8.3"' 2>/dev/null; then
  curl -fsSL https://bootstrap.pypa.io/pip/pip.pyz -o "$deps/pip.pyz"
  /usr/bin/python3 "$deps/pip.pyz" install --upgrade --target "$deps" 'PySide6==6.8.3'
fi
if ! /usr/bin/python3 -c 'import ctypes; ctypes.CDLL("libxcb-cursor.so.0")' 2>/dev/null && [ ! -e "$deps/os/usr/lib/x86_64-linux-gnu/libxcb-cursor.so.0" ]; then
  mkdir -p "$deps/debs" "$deps/os"
  (cd "$deps/debs" && apt-get download libxcb-cursor0)
  for deb in "$deps"/debs/libxcb-cursor0_*.deb; do dpkg-deb -x "$deb" "$deps/os"; done
fi
printf '%s\n' 'Desktop dependencies installed locally. Build the workspace, then run scripts/run_desktop.sh.'
