#!/usr/bin/env bash
set -e
set -o pipefail

ROBOT_IP="192.168.2.2"
KINEMATICS_FILE="$HOME/ros2_ws/src/config/my_robot_calibration.yaml"

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/ros2_ws/install/local_setup.bash"

PY_SERVER_DIR="$HOME/ros2_ws/src/py_server"
UI_DIR="$HOME/ros2_ws/src/robot-ui"

UI_HOST_IP="192.168.2.100"
UI_PORT="5173"
BROWSER_URL="http://${UI_HOST_IP}:${UI_PORT}/"

SESSION="ur3e_stack"
SCRIPTDIR="/tmp/${SESSION}_scripts"

command -v tmux >/dev/null 2>&1 || { echo "ERROR: tmux not found. Install: sudo apt install -y tmux"; exit 1; }
command -v ss   >/dev/null 2>&1 || { echo "ERROR: ss not found. Install: sudo apt install -y iproute2"; exit 1; }
command -v xdg-open >/dev/null 2>&1 || { echo "ERROR: xdg-open not found"; exit 1; }

mkdir -p "$SCRIPTDIR"

# Common helpers + failure handling
cat > "$SCRIPTDIR/common.sh" <<'EOF'
#!/usr/bin/env bash
set -e
set -o pipefail

die_to_shell() {
  local code=$?
  echo
  echo "=============================="
  echo "Command failed (exit=$code)"
  echo "You are now in an interactive shell in this pane."
  echo "=============================="
  exec bash
}

wait_for_exact_node() {
  local node="$1"
  local timeout="${2:-240}"
  local start="$(date +%s)"
  echo "Waiting for node: '${node}' (timeout ${timeout}s)"
  while true; do
    if ros2 node list 2>/dev/null | grep -qx "$node"; then
      echo "Node ready: '${node}'"
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout" ]; then
      echo "ERROR: Timeout waiting for node: '${node}'"
      ros2 node list 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

wait_for_port() {
  local port="$1"
  local timeout="${2:-240}"
  local start="$(date +%s)"
  echo "Waiting for port: ${port} (timeout ${timeout}s)"
  while true; do
    if ss -lnt 2>/dev/null | grep -q ":${port} "; then
      echo "Port open: ${port}"
      return 0
    fi
    if [ $(( $(date +%s) - start )) -ge "$timeout" ]; then
      echo "ERROR: Timeout waiting for port: ${port}"
      ss -lnt 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
}

wait_for_controller_active() {
  local controller="$1"
  local timeout="${2:-600}"
  local start="$(date +%s)"

  echo "Waiting for controller ACTIVE: '${controller}' (timeout ${timeout}s)"

  while true; do
    # Get controller list and strip ANSI color codes
    if ros2 control list_controllers 2>/dev/null \
      | sed -r 's/\x1B\[[0-9;]*[mK]//g' \
      | awk -v c="$controller" '$1==c && $3=="active" {found=1} END{exit !found}'
    then
      echo "Controller active: '${controller}'"
      return 0
    fi

    if [ $(( $(date +%s) - start )) -ge "$timeout" ]; then
      echo "ERROR: Timeout waiting for controller: '${controller}'"
      return 1
    fi

    sleep 1
  done
}
EOF
chmod +x "$SCRIPTDIR/common.sh"

# 1) UR driver
cat > "$SCRIPTDIR/driver.sh" <<EOF
#!/usr/bin/env bash
set -e
set -o pipefail
source "$SCRIPTDIR/common.sh"
trap die_to_shell ERR

source "$ROS_SETUP"
echo "[driver] ros2=\$(command -v ros2 || echo NOT_FOUND)"
echo "[driver] Starting UR3e robot driver..."
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=${ROBOT_IP} \
  kinematics_params_file:="${KINEMATICS_FILE}" \
  launch_rviz:=false \
  use_fake_hardware:=false
EOF
chmod +x "$SCRIPTDIR/driver.sh"

# 2) MoveIt
cat > "$SCRIPTDIR/moveit.sh" <<EOF
#!/usr/bin/env bash
set -e
set -o pipefail
source "$SCRIPTDIR/common.sh"
trap die_to_shell ERR

source "$ROS_SETUP"
echo "[moveit] ros2=\$(command -v ros2 || echo NOT_FOUND)"
echo "[moveit] Starting MoveIt..."
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  launch_rviz:=false
EOF
chmod +x "$SCRIPTDIR/moveit.sh"

# 3) Env + calibration + action server
cat > "$SCRIPTDIR/stack_ros.sh" <<EOF
#!/usr/bin/env bash
set -e
set -o pipefail
source "$SCRIPTDIR/common.sh"
trap die_to_shell ERR

source "$ROS_SETUP"
source "$WS_SETUP"
echo "[stack_ros] ros2=\$(command -v ros2 || echo NOT_FOUND)"

wait_for_exact_node /move_group 300

echo "[stack_ros] Waiting for pendant START (controller activation)..."
wait_for_controller_active scaled_joint_trajectory_controller 600

echo "[stack_ros] Adding environment collision..."
ros2 run stl_models enviroment_collision

echo "[stack_ros] Running calibration..."
ros2 run ur_action_server_cpp callibrate

echo "[stack_ros] Starting action server..."
ros2 run ur_action_server_cpp move_piece_server
EOF
chmod +x "$SCRIPTDIR/stack_ros.sh"

# 4) Python server (IMPORTANT: no exec, so errors stay visible)
cat > "$SCRIPTDIR/pyserver.sh" <<EOF
#!/usr/bin/env bash
set -e
set -o pipefail
source "$SCRIPTDIR/common.sh"
trap die_to_shell ERR
wait_for_exact_node /move_piece_server 300

source "$WS_SETUP"

echo "[pyserver] python=\$(command -v python3 || echo NOT_FOUND)"
echo "[pyserver] uvicorn=\$(command -v uvicorn || echo NOT_FOUND)"
echo "[pyserver] PWD before cd: \$PWD"
cd "$PY_SERVER_DIR"
echo "[pyserver] PWD after cd: \$PWD"

# If you use a venv, uncomment and set the correct path:
# source "$PY_SERVER_DIR/.venv/bin/activate"

uvicorn server:app --host 0.0.0.0 --port 8000
EOF
chmod +x "$SCRIPTDIR/pyserver.sh"

# 5) UI (also no exec)
cat > "$SCRIPTDIR/ui.sh" <<EOF
#!/usr/bin/env bash
set -e
set -o pipefail
source "$SCRIPTDIR/common.sh"
trap die_to_shell ERR
wait_for_exact_node /web_action_client 300

cd "$UI_DIR"
echo "[ui] PWD=\$PWD"
npm run dev -- --host 0.0.0.0
EOF
chmod +x "$SCRIPTDIR/ui.sh"

# ---- Fechar processos antigos (opcional, mas recomendado) ----
# Podes matar processos antigos se necessário, ex: pkill -f ros2

echo "A abrir janelas de terminal independentes..."

# 1. Driver
gnome-terminal --title="1. UR3e Driver" -- bash -c "$SCRIPTDIR/driver.sh; exec bash"

# 2. MoveIt
gnome-terminal --title="2. MoveIt" -- bash -c "$SCRIPTDIR/moveit.sh; exec bash"

# 3. Stack ROS (Environment + Calibration + Server)
gnome-terminal --title="3. Stack ROS" -- bash -c "$SCRIPTDIR/stack_ros.sh; exec bash"

# 4. Python Server
gnome-terminal --title="4. Python Server" -- bash -c "$SCRIPTDIR/pyserver.sh; exec bash"

# 5. UI (NPM)
gnome-terminal --title="5. UI Frontend" -- bash -c "$SCRIPTDIR/ui.sh; exec bash"

echo "Tudo lançado! Verifica as janelas abertas."
