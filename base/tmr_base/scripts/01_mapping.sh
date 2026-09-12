#!/usr/bin/env bash
# Compatibility name: mapping/navigation share one live SLAM session.
set -Eeuo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${root_dir}/scripts/03_start_navigation.sh"
