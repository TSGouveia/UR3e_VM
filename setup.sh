#!/usr/bin/env bash
set -e

REPO_URL="https://github.com/TSGouveia/UR3e_VM"
WS_DIR="$HOME/ros2_ws"
UI_DIR="$WS_DIR/src/robot-ui"

echo "🚀 [Sincronização] Iniciando Reconstrução do Ambiente..."

# --- 0. Limpeza inicial para evitar conflitos de rede/Node ---
echo "🧹 [0/9] Limpando pacotes antigos..."
sudo apt-get remove --purge -y nodejs npm libnode-dev || true
sudo apt-get autoremove -y

# --- 1. Set locale (Comandos Oficiais) ---
echo "🌍 [1/9] Configurando Locales..."
locale  # check for UTF-8
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

# --- 2. Setup Sources (Comandos Oficiais) ---
echo "📦 [2/9] Configurando Repositórios..."
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# --- 3. Install ROS 2 packages (Comandos Oficiais) ---
echo "🤖 [3/9] Instalando ROS 2 Humble..."
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-ros-base
sudo apt install -y ros-dev-tools

# --- 4. Drivers e Dependências Específicas (UR e MoveIt) ---
echo "🦾 [4/9] Instalando drivers Universal Robots e MoveIt..."
sudo apt install -y \
    ros-humble-ur \
    ros-humble-ur-msgs \
    ros-humble-moveit \
    ros-humble-moveit-visual-tools \
    ros-humble-ros2controlcli \
    ros-humble-controller-manager \
    tmux python3-pip iproute2 gnome-terminal

# --- 5. Node.js v20 (Essencial para o Vite) ---
echo "🔄 [5/9] Configurando NodeSource v20..."
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs

# --- 6. Dependências Python ---
echo "🐍 [6/9] Instalando Uvicorn e FastAPI..."
sudo pip3 install uvicorn fastapi --break-system-packages || sudo pip3 install uvicorn fastapi

# --- 7. Sincronização do Workspace e Git ---
echo "📂 [7/9] Sincronizando código do GitHub..."
mkdir -p "$WS_DIR"
cd "$WS_DIR"

if [ ! -d ".git" ]; then
    git init .
    git remote add origin "$REPO_URL"
    git fetch
    git checkout -t origin/main -f
else
    git pull origin main
fi

# --- 8. UI e Compilação ---
echo "🛠️ [8/9] Configurando UI e Compilando..."
if [ -d "$UI_DIR" ]; then
    cd "$UI_DIR"
    rm -rf node_modules package-lock.json
    npm install
    sudo npm install -g vite
    cd "$WS_DIR"
fi

find "$WS_DIR" -name "*.sh" -exec chmod +x {} +
[ -f "startproject" ] && chmod +x startproject
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# --- 9. Persistência ---
echo "📝 [9/9] Configurando Bashrc..."
if ! grep -q "ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo "=================================================="
echo "✅ SETUP COMPLETO COM DRIVERS UR E DOC OFICIAL!"
echo "=================================================="
