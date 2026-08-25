#!/usr/bin/env bash
# Type-check and lint the conformance suite on the host, no esp toolchain
# needed. Covers all of src/suite.rs; src/main.rs (the ESP-IDF bootstrap) still
# only compiles when flashed via ./flash.sh.
set -euo pipefail

cd "$(cd "$(dirname "$0")" && pwd)/../suite-hostcheck"
cargo fmt --check
cargo clippy --all-targets -- -D warnings
