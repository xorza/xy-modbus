#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
PORT="${PORT:-/dev/ttyACM0}"

cd "$DIR"
cargo build --release

espflash flash \
    --monitor \
    --non-interactive \
    --port "$PORT" \
    "$DIR/target/riscv32imac-esp-espidf/release/xy-modbus-esp32c6-test"
