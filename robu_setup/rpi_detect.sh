#!/bin/bash

# Usage:
#   source ./rpi_detect.sh

_rpi_read_model() {
    local f
    for f in /sys/firmware/devicetree/base/model /proc/device-tree/model; do
        if [[ -r "$f" ]]; then
            tr -d '\0' < "$f"
            return 0
        fi
    done

    if grep -qi '^model\s*:' /proc/cpuinfo 2>/dev/null; then
        grep -i '^model\s*:' /proc/cpuinfo | head -n1 | cut -d: -f2- | sed 's/^[[:space:]]*//'
        return 0
    fi

    return 1
}

is_raspberry_pi() {
    local model
    model="$(_rpi_read_model)" || return 1
    [[ "$model" == *"Raspberry Pi"* ]]
}

is_compute_module() {
    local model
    model="$(_rpi_read_model)" || return 1
    [[ "$model" == *"Compute Module"* ]]
}

get_rpi_model() {
    _rpi_read_model
}

get_rpi_revision() {
    if [[ -r /proc/cpuinfo ]]; then
        grep -i '^Revision\s*:' /proc/cpuinfo | tail -n1 | awk -F': ' '{print $2}'
    fi
}

get_rpi_type() {
    if is_raspberry_pi; then
        if is_compute_module; then
            echo "compute-module"
        else
            echo "pi"
        fi
    else
        echo "none"
    fi
}
