#!/usr/bin/env bash
set -euo pipefail

if (( $# > 1 )); then
    echo "Usage: bash $0 [/dev/serial/by-id/NAME or /dev/serial/by-path/NAME]" >&2
    exit 2
fi

arm_device=${1:-}
container_arm_device=
if [[ -n $arm_device ]]; then
    if [[ ! $arm_device =~ ^/dev/serial/by-(id|path)/[^/]+$ || ${arm_device##*/} == "." || ${arm_device##*/} == ".." ]]; then
        echo "Use the arm's /dev/serial/by-id/ or /dev/serial/by-path/ link." >&2
        exit 2
    fi
    container_arm_device="/host-dev${arm_device#/dev}"
fi

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
image=${HOSSHI_IMAGE:-demulab2025/ros2:latest}

# Use the mount syntax supported by the local Docker CLI.
client_version=$(docker version --format '{{.Client.Version}}')
client_major=${client_version%%.*}
if [[ ! $client_major =~ ^[0-9]+$ ]]; then
    echo "Could not determine Docker CLI version: $client_version" >&2
    exit 1
fi
dev_mount_option=bind-recursive=disabled
if (( client_major < 25 )); then
    dev_mount_option=bind-nonrecursive
fi

# Share device names dynamically while keeping the container's own /dev.
docker run --detach --init --pull=never --name hosshi-ros2-usb \
    --user 0:0 --cap-drop MKNOD --security-opt no-new-privileges \
    --mount "type=bind,src=/dev,dst=/host-dev,readonly,$dev_mount_option" \
    --device-cgroup-rule 'c 81:* rw' \
    --device-cgroup-rule 'c 188:* rw' \
    --mount "type=bind,src=$project_dir,dst=/work/hosshi_space" \
    --workdir /work/hosshi_space \
    --env ROS_LOCALHOST_ONLY=1 \
    --env "HOSSHI_ARM_DEVICE=$container_arm_device" \
    --entrypoint /bin/bash "$image" -ec '
        if [[ -n $HOSSHI_ARM_DEVICE && $(readlink /dev/ttyUSB0) != "$HOSSHI_ARM_DEVICE" ]]; then
            ln -sT -- "$HOSSHI_ARM_DEVICE" /dev/ttyUSB0
        fi
        exec sleep infinity
    '
