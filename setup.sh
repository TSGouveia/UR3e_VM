#!/usr/bin/env bash
set -e

REPO_URL="https://github.com/TSGouveia/UR3e_VM"
WS_DIR="$HOME/ros2_ws"
UI_DIR="$WS_DIR/src/robot-ui"

echo "🚀 [Sincronização] Iniciando Reconstrução do Ambiente..."

# --- 0. Limpeza de conflitos e Atualização de Repositórios ---
echo "🧹 [0/8] Limpando conflitos e atualizando APT..."
sudo dpkg --purge --force-all libnode-dev nodejs-dev 2>/dev/null || true
sudo apt update

# --- 1. Instalação de Dependências de Sistema e ROS 2 Control ---
echo "📦 [1/8] Instalando ferramentas de sistema e ROS2 Control..."
sudo apt install -y \
    tmux \
    python3-pip \
    iproute2 \
    gnome-terminal \
    nodejs \
    npm \
    ros-humble-ros2controlcli \
    ros-humble-controller-manager

# --- 2. Atualizar Node.js para v20 (Essencial para o Vite) ---
echo "🔄 [2/8] Verificando versão do Node.js..."
if [[ $(node -v 2>/dev/null) != v20* ]]; then
    echo "Instalando NodeSource v20 LTS..."
    curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
    sudo apt-get install -y nodejs
else
    echo "✅ Node.js já está na versão correta."
fi

# --- 3. Instalar dependências Python ---
echo "🐍 [3/8] Instalando Uvicorn e FastAPI..."
# Instalamos como root para garantir que os binários fiquem no PATH global
sudo pip3 install uvicorn fastapi

# --- 4. Configuração do Workspace e Git ---
echo "📂 [4/8] Sincronizando código do GitHub..."
mkdir -p "$WS_DIR"
cd "$WS_DIR"

if [ ! -d ".git" ]; then
    echo "🆕 Inicializando repositório no root..."
    git init .
    git remote add origin "$REPO_URL"
    git fetch
    git checkout -t origin/main -f
else
    echo "📥 Atualizando código existente..."
    git pull origin main
fi

# --- 5. Instalação da UI (Vite) ---
echo "🌐 [5/8] Configurando dependências do Frontend..."
if [ -d "$UI_DIR" ]; then
    cd "$UI_DIR"
    rm -rf node_modules package-lock.json
    npm install
    sudo npm install -g vite
    cd "$WS_DIR"
else
    echo "❌ ERRO: A pasta $UI_DIR não existe!"
fi

# --- 6. Permissões de Execução ---
echo "🔐 [6/8] Ajustando permissões dos scripts..."
find "$WS_DIR/src" -name "*.sh" -exec chmod +x {} +

# --- 7. Compilação ROS 2 ---
echo "🛠️ [7/8] Limpando e Compilando ambiente ROS2..."
rm -rf build/ install/ log/
# Importante fazer o source do ROS antes de compilar
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# --- 8. Persistência no Bashrc ---
echo "📝 [8/8] Configurando Bashrc..."
if ! grep -q "ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    echo "✅ Configurações adicionadas ao ~/.bashrc"
fi

echo "=================================================="
echo "✅ SETUP CONCLUÍDO COM SUCESSO!"
echo "=================================================="
echo "⚠️  ATENÇÃO: Fecha este terminal e abre um NOVO"
echo "=================================================="
