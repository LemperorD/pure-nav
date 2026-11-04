#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"
report_dir="$repo_dir/build/standalone/acceptance"
mkdir -p "$report_dir"
cmake --preset standalone 2>&1 | tee "$report_dir/configure.log"
cmake --build --preset standalone 2>&1 | tee "$report_dir/build.log"
ctest --test-dir build/standalone --output-on-failure --output-junit "$report_dir/ctest.xml" 2>&1 | tee "$report_dir/tests.log"
for scenario in navigation supply blocked sensor-loss; do
    ./bin/pure_nav --scenario "$scenario" --output "$report_dir/$scenario.csv" | tee "$report_dir/$scenario.log"
done
./bin/pure_nav --map assets/maps/rmul_2025.yaml --start 0.775,-5.525,0 --goal 10.525,0.225,0 --plan-only --output "$report_dir/rmul_2025_route.csv" | tee "$report_dir/map.log"
python3 scripts/render_acceptance.py "$report_dir"
