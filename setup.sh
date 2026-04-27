#!/usr/bin/env bash
set -e

echo "refrescando o workspace..."

# 1. Entrar na pasta do projeto
cd ~/ros2_ws/src/UR3e_VM 2>/dev/null || cd ~/ros2_ws/src

# 2. Baixar as novidades (o "download")
echo "📥 Baixando código do GitHub..."
git pull origin main

# 3. Limpar pastas de build para evitar erros de cache
echo "🧹 Limpando ambiente..."
rm -rf ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log

# 4. Compilar tudo de novo
echo "🛠️ Compilando..."
cd ~/ros2_ws
colcon build --symlink-install

source install/setup.bash
echo "✅ Download e compilação concluídos!"
