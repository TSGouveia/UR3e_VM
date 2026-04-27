#!/usr/bin/env bash

# Sair imediatamente se algum comando falhar
set -e

echo "🚀 INICIANDO INSTALAÇÃO COMPLETA: ROS 2 HUMBLE + UR DRIVER + WORKSPACE"

# --- 1. CONFIGURAÇÃO DE LOCALE ---
echo "🌍 Configurando Locale..."
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# --- 2. CONFIGURAÇÃO DE REPOSITÓRIOS ROS 2 ---
echo "📦 Configurando fontes do APT para ROS 2..."
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# --- 3. INSTALAÇÃO DO ROS 2 E FERRAMENTAS ---
echo "🤖 Instalando ROS 2 Humble e Dev Tools..."
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop ros-dev-tools python3-colcon-common-extensions -y

# --- 4. INSTALAÇÃO DO DRIVER UNIVERSAL ROBOTS ---
echo "🦾 Instalando Driver Oficial da Universal Robots (Binários)..."
sudo apt install ros-humble-ur -y

# --- 5. CONFIGURAÇÃO DO WORKSPACE E GIT ---
echo "🏗️ Criando Workspace e baixando código..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Verifica se a pasta já existe, se não, clona
if [ ! -d "UR3e_VM" ]; then
    echo "📥 Clonando repositório UR3e_VM..."
    git clone https://github.com/TSGouveia/UR3e_VM.git
else
    echo "📥 Repositório já existe. Atualizando..."
    cd UR3e_VM && git pull origin main && cd ..
fi

# --- 6. COMPILAÇÃO (COLCON) ---
echo "🛠️ Limpando caches antigos e compilando..."
cd ~/ros2_ws
rm -rf build/ install/ log/

# Carregar ambiente ROS para a compilação
source /opt/ros/humble/setup.bash

# Instalar dependências em falta via rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Compilar
colcon build --symlink-install

# --- 7. CONFIGURAÇÃO FINAL DO BASH ---
echo "⚙️ Configurando sourcing automático no .bashrc..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo "✅ TUDO CONCLUÍDO COM SUCESSO!"
echo "Reinicie o terminal ou execute: source ~/.bashrc"
