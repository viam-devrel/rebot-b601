#!/usr/bin/env bash
# Viam module entrypoint: bootstrap a venv on first run, then exec the module.
set -euo pipefail
cd "$(dirname "$0")"

VENV=".venv"
if [ ! -x "$VENV/bin/python" ]; then
    if command -v uv >/dev/null 2>&1; then
        uv venv "$VENV"
        uv pip install -p "$VENV/bin/python" -r requirements.txt
    else
        python3 -m venv "$VENV"
        "$VENV/bin/pip" install -r requirements.txt
    fi
fi

exec "$VENV/bin/python" -m src.main "$@"
