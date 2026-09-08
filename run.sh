#!/usr/bin/env bash
# Viam module entrypoint: bootstrap a venv on first run, then exec the module.
#
#   ./run.sh <socket-path>   normal viam-server invocation
#   ./run.sh --check         bootstrap only, then verify the module imports (CI)
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

if [ "${1:-}" = "--check" ]; then
    exec "$VENV/bin/python" -c "import src.main; from src.rebot_b601 import arm_service; assert arm_service.installed(); print('rebot-b601 module OK')"
fi

exec "$VENV/bin/python" -m src.main "$@"
