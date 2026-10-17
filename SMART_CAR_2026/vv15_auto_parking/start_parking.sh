#!/bin/sh
set -eu
cd "$(dirname "$0")"
exec sh ./run_auto_parking.sh "$@"
