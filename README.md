# vision-controller-entrega

#  UFVision Controller - Pouso Autônomo com ROS 2 e YOLOv8

Este pacote contém o  controle para pouso autônomo de precisão utilizando ROS 2 e YOLOv8.

---

## Estrutura do Pacote de Entrega

Os arquivos deste repositório são organizados e distribuídos automaticamente nas pastas corretas durante a instalação:

* `launch/mission.launch.py`: Launch file do ROS 2 que inicializa o nó principal de missão (`autonomus_landing_mission`).
* `models/best.pt`: Pesos treinados do modelo YOLOv8 para detecção de marcas e plataformas de pouso (instalado em `~/ros2_ws/src/ufvision_controller/models/`).
* `ufvision_controller/missions/autonomus_landing_mission.py`: Script Python com a lógica de controle de voo, estados da missão e pouso de precisão.
* `run_autonomo.py`: Script mestre de orquestração do ambiente de simulação (instalado na raiz do workspace em `~/ros2_ws/run_autonomo.py`).
* `setup_env.sh`: Script de configuração automática do ambiente e compilação do pacote.

---

## Instalação 
1. Clone este repositório dentro da pasta `src` do seu Workspace ROS 2:
cd ~/ros2_ws/src
git clone <https://github.com/robert0400/vision-controller-entrega.git> ufvision_controller

2. Execute o script de configuração automática (ele moverá os arquivos para as pastas corretas e compilará o workspace):
cd ~/ros2_ws/src/ufvision_controller
bash setup_env.sh

diretorio dos arquivos ou pastas:

usetup_env.sh                 --> ~/ros2_ws/src/ufvision_controller/setup_env.sh
run_autonomo.py               --> ~/ros2_ws/run_autonomo.py
package.xml                   --> ~/ros2_ws/src/ufvision_controller/package.xml
setup.py                       --> ~/ros2_ws/src/ufvision_controller/setup.py
launch/mission.launch.py       --> ~/ros2_ws/src/ufvision_controller/launch/mission.launch.py
models/best.pt                 --> ~/ros2_ws/src/ufvision_controller/models/best.pt
fvision_controller/missions/autonomus_landing_mission.py --> ~/ros2_ws/src/ufvision_controller/ufvision_controller/missions/autonomus_landing_mission.py

---

## Executar a Simulação

Com a instalação concluída, para iniciar todo o ambiente de simulação (Gazebo, PX4 SITL, MAVROS, ROS-Gazebo Bridge e a interface TMUX) basta executar o comando abaixo na raiz do seu workspace:

python3 ~/ros2_ws/run_autonomo.py
./QGroundControl.AppImage ou abrir o QGroundControl.AppImage atraves da ISO

###  Controle de Missão via TMUX
Ao executar o comando acima, a interface TMUX será aberta com as opções:
* **Painel da Câmera (Menu 1):** Digite `1` para abrir a janela OpenCV e visualizar as detecções do YOLOv8 em tempo real.
* **Painel da Missão (Menu 2):** Digite `3` e insira o alvo desejado (ex: `triangulo`, etc.) para iniciar a decolagem, mapeamento e o pouso autônomo.
*
