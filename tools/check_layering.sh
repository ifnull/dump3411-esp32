#!/bin/sh
# Enforce the layering in docs/ARCHITECTURE.md ("Three layers, one direction
# of dependency"):
#
#   1. Core (components/odid and the capture code in main/wifi_capture.*)
#      includes no board header and no peripheral-driver header
#      (components/periph_*).
#   2. Only boards/board.h includes a boards/board_<name>.h, and it does so
#      through DUMP3411_BOARD_HEADER, so nothing names one directly.
#
# A grep, not a static analyzer: it catches #include lines, which is what
# matters for keeping core free of board knowledge.
set -eu
cd "$(dirname "$0")/.."

fail=0
core="components/odid main/wifi_capture.c main/wifi_capture.h"
include_re='^[[:space:]]*#[[:space:]]*include[[:space:]]*[<"]'

if grep -nE "${include_re}(board(_[A-Za-z0-9_]+)?|periph_[A-Za-z0-9_]+(/[A-Za-z0-9_./]+)?)\.h[>\"]" -r $core; then
    echo "FAIL: core code above includes a board or peripheral-driver header"
    fail=1
fi

if grep -rnE "${include_re}board_[A-Za-z0-9_]+\.h[>\"]" \
        --include='*.c' --include='*.h' components main boards host tests tools; then
    echo "FAIL: include board.h instead of a board_<name>.h"
    fail=1
fi

[ "$fail" -eq 0 ] && echo "layering ok"
exit "$fail"
