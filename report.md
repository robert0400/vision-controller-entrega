ARQUITETURA DO SOFTWARE

O projeto foi feito em ros2 jazzy e o gazebo,alem do PX4 e a biblioteca yolov8
O run_autonomo.py é o gerenciador das sessoes do tmux que inicializa todos ambientes exceto qgcontrol,e o mission.launch.py é o gerenciador dos nós.
no nó da missao é enviado setpoints para a interface do PX4 para garantir um lop de controle fechado

A arquitetura de execução foi projetada para ser direta (standalone script). Em vez de exigir a compilação de um pacote ROS 2 tradicional via colcon build, a lógica de controle (autonomus_landing_mission.py) utiliza a biblioteca rclpy para comunicação MAVROS/PX4 de forma standalone, sendo orquestrada e inicializada diretamente pelo script mestre run_autonomo.py via TMUX.


METODO DE IDENTIFICAÇÂO DE FIGURAS

para detecção das figuras geometricas foi usado o algoritmo de IA yolov8 alimentado por pesos customizados produzindos no site roboflow(roboflow.com) cujo o dataset foi criado atraves de um script desenvolvido para capturar videos da camera do drone no gazebo.
para o calculo do erro de alinhamentos o algoritmo extrai coordenadas do centro da bounding box da plataforma e compara com o centro geometrico da camera, entao o vetor resultante é tranformado em comandos de velocidade para deslocar o drone na horizontal ate que a marca esteja centralizada com uma tolerancia de 8 pixels


MAQUINA DE ESTADOS

o script do arquivo autonomus_landing_mission.py gerencia o seguinte ciclo:
1-Arm e Takeoff:  muda o modo do PX4 para offboard e sobe para uma altura programada de 2 metros
2-Search: o drone faz uma orbita ao redor do mapa fazendo uma busca visual para mapeamento das plataformas e o calculo de sua porcentagem de exatidão com a plataforma pedida
3-Escolha do alvo: aṕos mapear todas as plataformas o drone irá se direcionar para a platafrma com maior porcentagem de similaridade com o comando dado.
4-Alinhamento e descida: apos a decisao o drone alinha ao centro da plataforma e inicia o processo de pouso  enquanto tenta manter o alinhamento vertical.
5-Land e desarm: ao atingir o ponto mínimo e pousar o drone envia o comando de desarme dos motores.
