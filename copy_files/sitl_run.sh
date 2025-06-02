#!/bin/bash
# MuJoCo SITL startup script

set -e

if [ "$#" -lt 1 ]; then
	echo usage: sitl_run.sh sitl_bin model src_path build_path
	exit 1
fi

if [[ -n "$DONT_RUN" ]]; then
	echo "Not running simulation (DONT_RUN is set)."
	exit 0
fi


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# Default parameters
sitl_bin="$1"
MODEL_NAME=${2:-skydio_x2}
src_path="$3"
build_path="$4"
MAVLINK_ADDR=INADDR_ANY #${2:-127.0.0.1}
MAVLINK_PORT=4560 #${3:-4560}
MUJOCO_BRIDGE_PATH="${src_path}/Tools/simulation/mujoco/mujoco_bridge" #${4:-"${HOME}/mujoco-bridge"}

rootfs="$build_path/rootfs" # this is the working directory
mkdir -p "$rootfs"

# To disable user input
if [[ -n "$NO_PXH" ]]; then
	no_pxh=-d
else
	no_pxh=""
fi

export PX4_SIM_MODEL=mujoco_${MODEL_NAME}

MUJOCO_PID=0

# Model file path
MODEL_FILE="${MUJOCO_BRIDGE_PATH}/models/${MODEL_NAME}/scene.xml"

if [ ! -f "${MODEL_FILE}" ]; then
    echo "Error: Model file not found: ${MODEL_FILE}"
    exit 1
fi

echo "Starting MuJoCo SITL simulation..."
echo "Model: ${MODEL_FILE}"
echo "MAVLink: ${MAVLINK_ADDR}:${MAVLINK_PORT}"

# Start MuJoCo bridge
#"${build_path}/build_mujoco_bridge/bin/simulate" "${MODEL_FILE}" "${MAVLINK_ADDR}" "${MAVLINK_PORT}" &
MUJOCO_PID=$!

# Wait for bridge to initialize
#sleep 2

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

sitl_command="\"$sitl_bin\" $no_pxh \"$build_path\"/etc"

echo "SITL COMMAND: $sitl_command"
eval "$sitl_command"

popd >/dev/null

kill -9 $MUJOCO_PID

## Cleanup function
#cleanup() {
#    echo "Shutting down MuJoCo simulation..."
#    kill ${MUJOCO_PID} 2>/dev/null
#    wait ${MUJOCO_PID} 2>/dev/null
#}
#
#trap cleanup EXIT
#
## Keep script running
#wait ${MUJOCO_PID}
