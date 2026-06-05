#!/usr/bin/env zsh
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export IDF_PATH="${IDF_PATH:-$HOME/esp/esp-idf}"

if [[ ! -f "$IDF_PATH/export.sh" ]]; then
  print -u2 "ESP-IDF export.sh not found at: $IDF_PATH"
  print -u2 "Install ESP-IDF or set IDF_PATH to the correct directory."
  exit 1
fi

. "$IDF_PATH/export.sh" >/dev/null
cd "$PROJECT_ROOT"
exec idf.py "$@"
