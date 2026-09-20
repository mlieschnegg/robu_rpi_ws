#!/bin/bash
# Functions only: loaded before updating the repository that contains this file.

robu_git_health() {
    local workspace="$1" report="$2"
    if [[ ! -d "$workspace/.git" ]]; then
        echo "[ROBU] Missing .git directory: $workspace" > "$report"
        return 1
    fi
    if LC_ALL=C git -C "$workspace" fsck --full --no-dangling > "$report" 2>&1; then
        if git -C "$workspace" rev-parse --verify HEAD^{commit} >> "$report" 2>&1; then
            return 0
        fi
        # An unborn/missing HEAD is not a usable installed checkout.
        return 1
    fi
    # Do not treat permission, I/O, configuration or other unknown failures as
    # proof of corruption. In those cases stop and leave the directory untouched.
    if grep -Eqi 'corrupt|missing (blob|tree|commit|tag)|broken link|invalid sha1 pointer|object file .* is empty|bad object|invalid object|index file smaller' "$report"; then
        return 1
    fi
    return 2
}

robu_reclone() {
    local workspace="$1" repository="$2" state="$3"
    local candidate backup="${workspace}.recovery"
    # Only rotate a real recovery directory, never follow a symlink.
    if [[ -L "$backup" || ( -e "$backup" && ! -d "$backup" ) ]]; then
        echo "[ROBU] Repair stopped: unexpected recovery path: $backup"
        return 1
    fi
    candidate=$(mktemp -d "${workspace}.clone.XXXXXX") || return 1
    if ! GIT_TERMINAL_PROMPT=0 git -c core.fsync=all -c core.fsyncMethod=fsync \
        clone --branch main --single-branch "$repository" "$candidate"; then
        rm -rf -- "$candidate"
        echo "[ROBU] Clone failed; existing directory retained."
        return 1
    fi
    if ! robu_git_health "$candidate" "$state/git-health.log"; then
        rm -rf -- "$candidate"
        echo "[ROBU] Replacement failed integrity check; existing directory retained."
        return 1
    fi
    # No ROS build in the temporary path: generated files contain absolute paths.
    if [[ -e "$workspace" ]]; then
        # Keep at most one recovery copy. Only remove the older one after the
        # replacement clone passed validation and the current checkout still exists.
        if [[ -d "$backup" ]]; then
            rm -rf -- "$backup" || return 1
        fi
        mv -- "$workspace" "$backup" || return 1
    fi
    if ! mv -- "$candidate" "$workspace"; then
        if [[ -d "$backup" && ! -e "$workspace" ]]; then
            mv -- "$backup" "$workspace"
        fi
        return 1
    fi
    rm -f -- "$state/built-commit"
    echo "[ROBU] Fresh clone installed. Previous directory, if any: $backup"
}

robu_update_and_build() {
    local workspace="$1" repository="$2" state="$3"
    local health=0 revision built=""
    mkdir -p "$state" || return 1
    # The destructive reset is limited to this managed directory, never a symlink.
    if [[ "${workspace##*/}" != ".robu" || -L "$workspace" || -L "$workspace/.git" ]]; then
        echo "[ROBU] Refusing unexpected workspace path: $workspace"
        return 1
    fi
    # Recover an interruption between the two directory renames.
    if [[ ! -e "$workspace" && -d "${workspace}.recovery" ]]; then
        mv -- "${workspace}.recovery" "$workspace" || return 1
    fi
    robu_git_health "$workspace" "$state/git-health.log" || health=$?
    case "$health" in
        1) robu_reclone "$workspace" "$repository" "$state" || return 1 ;;
        2) cat "$state/git-health.log"; return 1 ;;
    esac

    git -C "$workspace" config core.fsync all || return 1
    git -C "$workspace" config core.fsyncMethod fsync || return 1
    # Enforce the trusted repository URL for this centrally managed checkout.
    git -C "$workspace" remote set-url origin "$repository" || return 1
    if GIT_TERMINAL_PROMPT=0 git -C "$workspace" fetch --no-tags origin \
        '+refs/heads/main:refs/remotes/origin/main'; then
        # Invalidate the successful-build marker BEFORE touching source files:
        # reset may restore local edits even when HEAD itself has not changed.
        if [[ "$(git -C "$workspace" rev-parse HEAD)" != "$(git -C "$workspace" rev-parse origin/main)" ]] ||
            ! git -C "$workspace" diff --quiet HEAD --; then
            rm -f -- "$state/built-commit"
        fi
        git -C "$workspace" reset --hard origin/main || return 1
    else
        echo "[ROBU] Fetch failed; keeping existing checkout without reset."
        # A network/authentication failure must never trigger a reclone.
    fi

    revision=$(git -C "$workspace" rev-parse HEAD) || return 1
    [[ ! -f "$state/built-commit" ]] || read -r built < "$state/built-commit"
    if [[ "$built" != "$revision" || ! -f "$workspace/install/setup.bash" ]]; then
        rm -f -- "$state/built-commit"
        if (cd "$workspace" && colcon build); then
            [[ -f "$workspace/install/setup.bash" ]] || return 1
            printf '%s\n' "$revision" > "$state/built-commit.tmp" || return 1
            mv -- "$state/built-commit.tmp" "$state/built-commit" || return 1
        else
            echo "[ROBU] Build failed; retrying on the next start."
            return 1
        fi
    fi
}
