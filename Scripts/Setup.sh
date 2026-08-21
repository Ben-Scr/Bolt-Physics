#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

# Konstante Pfade, relativ zu diesem Skript (scripts/).
readonly VENDORED_PREMAKE="../vendor/bin/premake5"
readonly PREMAKE_FILE="../premake5.lua"

# Locate a premake5 binary. Prefer a vendored one if it's executable on this
# platform; otherwise fall back to whatever is in PATH.
PREMAKE_BIN=""
if [ -x "$VENDORED_PREMAKE" ]; then
    PREMAKE_BIN="$VENDORED_PREMAKE"
elif command -v premake5 >/dev/null 2>&1; then
    PREMAKE_BIN="premake5"
else
    echo "[Setup] ERROR: premake5 not found." >&2
    echo "[Setup] Install premake5 (https://premake.github.io/download.html) or" >&2
    echo "[Setup] place a binary at vendor/bin/premake5." >&2
    exit 1
fi

# Default to a sensible action for the host OS.
if [ "$#" -ge 1 ]; then
    ACTION="$1"
else
    case "$(uname -s)" in
        Darwin) ACTION="xcode4" ;;
        Linux)  ACTION="gmake2" ;;
        *)      ACTION="gmake2" ;;
    esac
fi

echo "[Setup] Using $PREMAKE_BIN with action: $ACTION"
"$PREMAKE_BIN" --file="$PREMAKE_FILE" "$ACTION"
echo "[Setup] Done."
