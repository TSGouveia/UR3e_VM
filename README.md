# 🤖 UR3e ROS 2 Workspace Setup

Este repositório contém as instruções e scripts necessários para configurar um ambiente de desenvolvimento **ROS 2 Humble** para o robô **Universal Robots UR3e** numa Máquina Virtual Ubuntu 22.04.

---

## 🚀 Passo a Passo de Instalação

Nesta primeira fase, a VM deve manter o acesso total à internet (modo NAT) para descarregar todas as dependências.

### Configuração do Workspace (Setup)
Execute o script de automação para preparar o ambiente:

```bash
# Download do script de setup
wget https://raw.githubusercontent.com/TSGouveia/UR3e_VM/main/setup.sh

# Atribuir permissões e executar
chmod +x setup.sh
./setup.sh
```
## 🌐 Configuração de Rede e Hardware

Após a instalação do software, segue estes passos para estabelecer a ligação com o UR3e:

### 1. Alterar Definições da VM
1. **Desliga a VM**.
2. Nas definições de **Rede** (Network) do teu software de virtualização (VirtualBox/VMware):
   * Altera o adaptador para **Bridged Adapter** (Placa em modo Bridge).
   * Ativa a opção **"Replicate physical network connection state"**.
3. Liga a VM novamente.

### 2. Configurar IP Estático no Ubuntu
Dentro da VM, acede às Definições de Rede e configura o IPv4 manualmente:
* **Método:** Manual
* **Endereço (Address):** 192.168.2.X (onde X é um número à tua escolha, ex: 192.168.2.100)
* **Máscara (Netmask):** 255.255.0.0
* **Gateway:** (Deixar em branco)

---

## 🏎️ Execução do Projeto

Com o cabo de rede ligado ao router do robô e a configuração acima concluída, corre o comando de arranque:

```bash
# No diretório ros2_ws/src
chmod +x startproject
./startproject
```
