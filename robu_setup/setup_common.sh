#!/bin/bash

# Continue independent setup commands and retain every unhandled failure.
set +e
set -E
ROBU_SETUP_FAILURES=()

# Apply the same PATH cleanup now, before any child script builds ROS packages.
source "$(dirname "${BASH_SOURCE[0]}")/../config/shell/tool_paths.sh"

robu_record_failure() {
    local status="$1"
    local location="$2"
    local command_text="$3"
    ROBU_SETUP_FAILURES+=("${location}: ${command_text} (exit ${status})")
    printf '[ROBU] ERROR: %s: %s (exit %s); continuing.\n' \
        "$location" "$command_text" "$status" >&2
}

trap 'robu_record_failure "$?" "${BASH_SOURCE[0]}:${LINENO}" "$BASH_COMMAND"' ERR

robu_setup_report() {
    if [ "${#ROBU_SETUP_FAILURES[@]}" -eq 0 ]; then
        echo "[ROBU] Setup completed successfully."
        return 0
    fi
    printf '\n[ROBU] Setup finished with %s failure(s). Review and retry:\n' \
        "${#ROBU_SETUP_FAILURES[@]}" >&2
    printf '  - %s\n' "${ROBU_SETUP_FAILURES[@]}" >&2
    return 1
}

install_python_dev_tools() {
    if [ ! -x "$VENV_DIR/bin/python" ]; then
        robu_record_failure 1 "Python development tools" "Missing venv: $VENV_DIR"
        return 0
    fi

    local requirement
    while IFS= read -r requirement || [ -n "$requirement" ]; do
        case "$requirement" in
            ''|'#'*) continue ;;
        esac
        # Install separately so a missing optional package cannot block Ruff.
        "$VENV_DIR/bin/python" -m pip install --upgrade "$requirement"
    done < "$SCRIPT_DIR/requirements-dev.txt"

    # Firmware development runs on the PC; clang-format 19 has no Linux ARM wheel.
    if ! is_raspberry_pi; then
        "$VENV_DIR/bin/python" -m pip install --upgrade "clang-format==19.1.7"
    fi
    "$VENV_DIR/bin/ruff" --version
}

setup_formatting_configs() {
    "$SYS_PYTHON" "$SCRIPT_DIR/setup_formatting.py" --work-dir "$WORK_DIR"
}

setup_tool_paths() {
    "$SYS_PYTHON" "$SCRIPT_DIR/setup_tool_paths.py" "$@"
}

install_vscode_extensions() {
    if ! command -v code >/dev/null 2>&1; then
        robu_record_failure 1 "VS Code extensions" "The code command is unavailable"
        return 0
    fi
    local extension
    while IFS= read -r extension || [ -n "$extension" ]; do
        case "$extension" in
            ''|'#'*) continue ;;
        esac
        code --install-extension "$extension"
    done < "$SCRIPT_DIR/vscode_extensions.txt"
}
