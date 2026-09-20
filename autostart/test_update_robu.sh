#!/bin/bash
# Local integration test; no GitHub, ROS installation or user checkout is used.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/update_robu.sh"
TEST_ROOT=$(mktemp -d)
trap 'rm -rf -- "$TEST_ROOT"' EXIT
mkdir -p "$TEST_ROOT/work" "$TEST_ROOT/state"
workspace="$TEST_ROOT/work/.robu"
repository="$TEST_ROOT/origin.git"
state="$TEST_ROOT/state"
git init -q --bare "$repository"
git init -q -b main "$TEST_ROOT/author"
git -C "$TEST_ROOT/author" config user.email test@example.invalid
git -C "$TEST_ROOT/author" config user.name Test
echo initial > "$TEST_ROOT/author/source.txt"
git -C "$TEST_ROOT/author" add .
git -C "$TEST_ROOT/author" commit -qm initial
git -C "$TEST_ROOT/author" remote add origin "$repository"
git -C "$TEST_ROOT/author" push -q origin main

colcon() {
    echo build >> "$state/builds"
    [[ ! -f "$state/fail-build" ]] || return 1
    mkdir -p install
    touch install/setup.bash
}
robu_update_and_build "$workspace" "$repository" "$state"
[[ $(wc -l < "$state/builds") -eq 1 ]]
robu_update_and_build "$workspace" "$repository" "$state"
[[ $(wc -l < "$state/builds") -eq 1 ]]

# Local tracked edits are reverted, untracked student-independent files retained.
echo local > "$workspace/source.txt"
echo retain > "$workspace/untracked.txt"
robu_update_and_build "$workspace" "$repository" "$state"
[[ $(cat "$workspace/source.txt") == initial && -f "$workspace/untracked.txt" ]]
[[ $(wc -l < "$state/builds") -eq 2 ]]

echo update > "$TEST_ROOT/author/source.txt"
git -C "$TEST_ROOT/author" commit -qam update
git -C "$TEST_ROOT/author" push -q origin main
touch "$state/fail-build"
if robu_update_and_build "$workspace" "$repository" "$state"; then exit 1; fi
[[ ! -f "$state/built-commit" ]]
rm "$state/fail-build"
robu_update_and_build "$workspace" "$repository" "$state"
[[ -f "$state/built-commit" && $(wc -l < "$state/builds") -eq 4 ]]

# Offline: neither reset local edits nor replace a healthy checkout.
echo offline > "$workspace/source.txt"
robu_update_and_build "$workspace" "$TEST_ROOT/unavailable.git" "$state"
[[ $(cat "$workspace/source.txt") == offline && ! -e "${workspace}.recovery" ]]

# Remove a reachable loose object to simulate actual corruption.
object=$(git -C "$workspace" rev-parse HEAD:source.txt)
rm "$workspace/.git/objects/${object:0:2}/${object:2}"
if robu_update_and_build "$workspace" "$TEST_ROOT/unavailable.git" "$state"; then exit 1; fi
[[ -f "$workspace/untracked.txt" && ! -e "${workspace}.recovery" ]]
robu_update_and_build "$workspace" "$repository" "$state"
[[ $(cat "$workspace/source.txt") == update && -f "${workspace}.recovery/untracked.txt" ]]
git -C "$workspace" fsck --full --no-dangling

# Repeated repairs rotate one backup instead of growing without limit.
object=$(git -C "$workspace" rev-parse HEAD:source.txt)
rm "$workspace/.git/objects/${object:0:2}/${object:2}"
robu_update_and_build "$workspace" "$repository" "$state"
[[ -d "${workspace}.recovery/.git" && ! -f "${workspace}.recovery/untracked.txt" ]]
git -C "$workspace" fsck --full --no-dangling
echo 'PASS: clone, no-op, reset, update, build retry, offline, corruption recovery, backup rotation'
