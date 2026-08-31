#!/usr/bin/env python3
import os
import sys
import subprocess
import time

def setup_workspace():
    pkg_path = os.path.expanduser('~/ros2_ws/src/ufvision_controller')
    if pkg_path not in sys.path:
        sys.path.append(pkg_path)

def main():
    setup_workspace()
    
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        
        if mode == "--menu-camera":
            while True:
                os.system('clear')
                print("==========================================")
                print("       MENU 1: VISÃO / CÂMERA            ")
                print("==========================================")
                print("1) Ligar Câmera com Detecção YOLOv8")
                print("2) Ligar Câmera Simples (rqt_image_view)")
                print("------------------------------------------")
                opc = input("Escolha a câmera [1 ou 2]: ").strip()
                
                if opc == '1':
                    os.system("pkill -f rqt_image_view 2>/dev/null")
                    os.system("pkill -f yolo_node.py 2>/dev/null")
                    cmd = "source /opt/ros/jazzy/setup.bash && export PYTHONPATH=$PYTHONPATH:~/ros2_ws/src/ufvision_controller && python3 ~/ros2_ws/src/ufvision_controller/ufvision_controller/yolo_node.py"
                    subprocess.run(cmd, shell=True, executable='/bin/bash')
                elif opc == '2':
                    os.system("pkill -f yolo_node.py 2>/dev/null")
                    os.system("pkill -f rqt_image_view 2>/dev/null")
                    cmd = "source /opt/ros/jazzy/setup.bash && ros2 run rqt_image_view rqt_image_view /camera/image_raw"
                    subprocess.run(cmd, shell=True, executable='/bin/bash')
                else:
                    print("Opção inválida!")
                    time.sleep(1)

        elif mode == "--menu-missao":
            while True:
                os.system('clear')
                print("==========================================")
                print("       MENU 2: MISSÃO AUTÔNOMA           ")
                print("==========================================")
                print("3) Iniciar Missão de Pouso Autônomo")
                print("4) Sair / Encerrar Simulação")
                print("------------------------------------------")
                opc = input("Escolha uma opção [3 ou 4]: ").strip()
                
                if opc == '3':
                    while True:
                        print("\n------------------------------------------")
                        print("Escolha a plataforma pelo  NOME:")
                        print("  1) circulo     5) pentagono")
                        print("  2) triangulo   6) seta")
                        print("  3) quadrado    7) cruz")
                        print("  4) hexagono    8) estrela")
                        print("------------------------------------------")
                        target = input("Alvo [1-8 ou nome]: ").strip().lower()
                        
                        validos = ['1','2','3','4','5','6','7','8',
                                   'circulo','circulo','triangulo','triângulo',
                                   'quadrado','hexagono','hexagôno','pentagono',
                                   'pentágono','seta','cruz','estrela']
                        
                        if target in validos:
                            os.system("pkill -f autonomus_landing_mission.py 2>/dev/null")
                            time.sleep(0.5)
                            print(f"\nDisparando Missão para o alvo: {target}...")
                            cmd = f"source /opt/ros/jazzy/setup.bash && export PYTHONPATH=$PYTHONPATH:~/ros2_ws/src/ufvision_controller && python3 ~/ros2_ws/src/ufvision_controller/ufvision_controller/missions/autonomus_landing_mission.py --ros-args -p target_platform:='{target}'"
                            subprocess.run(cmd, shell=True, executable='/bin/bash')
                            
                            # Loop pós-pouso para continuar executando ou fechar
                            print("\n" + "="*50)
                            print("     POUSO CONCLUÍDO E MOTORES DESARMADOS     ")
                            print("="*50)
                            resp = input("Deseja executar outra missão? (s/n): ").strip().lower()
                            if resp in ['s', 'sim', 'y', 'yes']:
                                continue
                            else:
                                break
                        else:
                            print("Opção inválida! Digite um número de 1 a 8 ou o nome.")
                            time.sleep(1)
                
                elif opc == '4':
                    print("Encerrando simulação e fecho de todos os processos...")
                    os.system("pkill -f px4 2>/dev/null")
                    os.system("pkill -f gz 2>/dev/null")
                    os.system("pkill -f mavros 2>/dev/null")
                    os.system("pkill -f QGroundControl 2>/dev/null")
                    os.system("tmux kill-session -t ufvision_sim 2>/dev/null")
                    sys.exit(0)
                else:
                    print("Opção inválida!")
                    time.sleep(1)
        return

    SESSION = "ufvision_sim"
    GZ_CAM_TOPIC = "/world/trainee/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
    WORLDS_DIR = os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/worlds")

    print("=======================================================")
    print("       UFVISION - SIMULAÇÃO INTEGRADA (PYTHON)         ")
    print("=======================================================")
    print("Embaralhando plataformas no Gazebo...")
    os.system(f"cd {WORLDS_DIR} && python3 randomiza_trainee.py 2>/dev/null")

    print("Limpando sessões antigas do TMUX...")
    os.system(f"tmux kill-session -t {SESSION} 2>/dev/null")
    time.sleep(1)

    os.system("QGroundControl.AppImage &>/dev/null &")

    # Criando os 5 painéis no TMUX
    os.system(f"tmux new-session -d -s {SESSION} -n 'SITL_PANES'")
    os.system(f"tmux split-window -t {SESSION}:0 -h")
    os.system(f"tmux split-window -t {SESSION}:0.0 -v")
    os.system(f"tmux split-window -t {SESSION}:0.1 -v")
    os.system(f"tmux split-window -t {SESSION}:0.2 -v")
    os.system(f"tmux select-layout -t {SESSION}:0 tiled")

    # Painel 0.0: Gazebo + PX4
    os.system(f"tmux send-keys -t {SESSION}:0.0 'cd ~/PX4-Autopilot && PX4_GZ_WORLD=trainee make px4_sitl gz_x500_mono_cam_down' C-m")

    # Aguarda o PX4 inicializar o MAVLink
    print("Aguardando PX4 inicializar porta MAVLink...")
    time.sleep(12)

    # Painel 0.1: Image Bridge
    os.system(f"tmux send-keys -t {SESSION}:0.1 'source /opt/ros/jazzy/setup.bash && cd ~/PX4-Autopilot && ros2 run ros_gz_image image_bridge {GZ_CAM_TOPIC} --ros-args -r {GZ_CAM_TOPIC}:=/camera/image_raw' C-m")

    # Painel 0.2: MAVROS
    os.system(f"tmux send-keys -t {SESSION}:0.2 'source /opt/ros/jazzy/setup.bash && ros2 launch mavros px4.launch fcu_url:=\"udp://:14540@127.0.0.1:14557\"' C-m")

    # Painel 0.3: Menu 1 - Câmeras
    os.system(f"tmux send-keys -t {SESSION}:0.3 'python3 ~/ros2_ws/run_autonomo.py --menu-camera' C-m")

    # Painel 0.4: Menu 2 - Missão
    os.system(f"tmux send-keys -t {SESSION}:0.4 'python3 ~/ros2_ws/run_autonomo.py --menu-missao' C-m")

    # Foco no Menu 2
    os.system(f"tmux select-pane -t {SESSION}:0.4")

    # Anexa a sessão
    os.system(f"tmux attach-session -t {SESSION}")

if __name__ == '__main__':
    chmod_cmd = "chmod +x ~/ros2_ws/run_autonomo.py"
    subprocess.run(chmod_cmd, shell=True)
    main()
