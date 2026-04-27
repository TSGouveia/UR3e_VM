# 🤖 UR3e ROS 2 Workspace Setup

Este repositório contém as instruções e scripts necessários para configurar um ambiente de desenvolvimento **ROS 2 Humble** para o robô **Universal Robots UR3e** numa Máquina Virtual (VM) Ubuntu 22.04.

---

## 📋 Fluxo de Configuração

O processo de instalação segue a lógica de preparar o sistema base, instalar as ferramentas robóticas e sincronizar o código de controlo específico.



---

## 🚀 Passo a Passo

### 1. Criar a Máquina Virtual
Instale o Ubuntu 22.04 LTS (Jammy Jellyfish) na sua plataforma de virtualização favorita (VirtualBox, VMware, etc.). 
* **Recomendação:** Configure a rede em modo **Bridge** para que a VM consiga comunicar diretamente com o IP do robô.

### 2. Instalação do ROS 2 Humble
O sistema utiliza a distribuição Humble. A instalação oficial via pacotes Debian garante estabilidade:
* [Documentação Oficial: ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 3. Instalação do UR Driver
O driver oficial da Universal Robots permite o controlo em tempo real e a leitura de estados do braço robótico:
* [Repositório Oficial: Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

---

## 🛠️ Configuração Automática (Recomendado)

Para automatizar os passos acima e resolver conflitos comuns de chaves GPG ou repositórios corrompidos, utiliza o script `setup.sh` incluído neste repositório.

### Como executar:

1. Abra o terminal na sua VM.
2. Execute os seguintes comandos:

```bash
# Baixar o script diretamente do repositório
curl -O [https://raw.githubusercontent.com/TSGouveia/UR3e_VM/main/setup.sh](https://raw.githubusercontent.com/TSGouveia/UR3e_VM/main/setup.sh)

# Dar permissão de execução ao ficheiro
chmod +x setup.sh

# Iniciar o processo de instalação e configuração
./setup.sh
