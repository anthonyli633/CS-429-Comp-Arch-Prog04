#!/usr/bin/env bash
set -euo pipefail

HW=./hw4

if [[ ! -x "$HW" ]]; then
  echo "Expected executable ./hw4 (build first)" >&2
  exit 1
fi

pass() { echo "PASS: $1"; }
fail() { echo "FAIL: $1"; exit 1; }

# 1) out42
out="$($HW test1.tko)"
[[ "$out" == "42" ]] && pass "test1_out42" || fail "test1_out42 (got: $out)"

# 2) echo
out="$(printf "123\n" | $HW test2.tko)"
[[ "$out" == "123" ]] && pass "test2_echo" || fail "test2_echo (got: $out)"

expect_sim_error () {
  local file="$1"
  set +e
  $HW "$file" 1>/dev/null 2>stderr.tmp
  local code=$?
  set -e
  local stderr="$(cat stderr.tmp || true)"
  rm -f stderr.tmp
  [[ $code -ne 0 ]] || fail "$file expected non-zero exit code"
  [[ "$stderr" == "Simulation error"* ]] && pass "$file" || fail "$file expected 'Simulation error' on stderr (got: $stderr)"
}

expect_sim_error test3.tko
expect_sim_error test4.tko
expect_sim_error test5.tko

echo "All tests passed."
