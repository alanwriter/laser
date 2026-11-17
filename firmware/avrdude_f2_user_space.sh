#!/bin/sh
# Run the F2 Pi's unpacked Debian avrdude without installing system packages.
set -eu
tool_root=$(CDPATH= cd -- "$(dirname -- "$0")/avrdude-tool/root" && pwd)
LD_LIBRARY_PATH="$tool_root/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH
exec "$tool_root/usr/bin/avrdude" -C "$tool_root/etc/avrdude.conf" "$@"
