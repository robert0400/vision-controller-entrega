#!/usr/bin/env python3
import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, WaypointPull
from cv_bridge import CvBridge
import cv2
import os
import math
import numpy as np
from ultralytics import YOLO

class AutonomousLandingMission(Node):
    def __init__(self):
        super().__init__('autonomus_landing_mission')

        self.wp_pull_cli = self.create_client(WaypointPull, "/mavros/mission/pull")
        if self.wp_pull_cli.wait_for_service(timeout_sec=2.0):
            self.wp_pull_cli.call_async(WaypointPull.Request())

        self.PLATFORM_MAP = {
            '1': 'circulo', '2': 'triangulo', '3': 'quadrado', '4': 'hexagono',
            '5': 'pentagono', '6': 'seta', '7': 'cruz', '8': 'estrela'
        }

        self.declare_parameter('target_platform', '1')
        raw_target = str(self.get_parameter('target_platform').get_parameter_value().string_value).strip()
        
        if raw_target in self.PLATFORM_MAP:
            self.target_platform = self.PLATFORM_MAP[raw_target]
        else:
            self.target_platform = self.normalizar_nome(raw_target)

        self.get_logger().info(f"[MISSAO] Alvo Selecionado: {self.target_platform.upper()}")

        # Geometria Rígida
        self.ALTURA_ORBITA = 2.0
        self.RAIO_ORBITA = 3.5  
        self.orbit_speed = 0.06  
        self.NUM_PERIPHERAL_SLOTS = 7
        self.SLOT_ANGLE_STEP = (2 * math.pi) / self.NUM_PERIPHERAL_SLOTS
        self.ALTURA_POUSO_FINAL = 0.20

        self.MODEL_PATH = os.path.expanduser('~/ros2_ws/src/ufvision_controller/models/best.pt')

        self.get_logger().info(f"[YOLO] Carregando modelo em: {self.MODEL_PATH}")
        self.model = YOLO(self.MODEL_PATH)
        self.bridge = CvBridge()

        self.reset_mission_state()

        # SUBSCRIÇÕES E PUBLICADORES
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile_system_default)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile_sensor_data)
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)

        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.timer = self.create_timer(0.05, self.control_loop)

    def reset_mission_state(self):
        self.state = 'TAKEOFF_SAFETY'
        self.current_state = State()
        self.current_pose = PoseStamped()

        self.img_center_x = 320
        self.img_center_y = 240
        self.target_found = False
        self.target_confidence = 0.0
        self.target_x = 0
        self.target_y = 0

        self.analyzed_platforms = []

        self.orbit_angle = 0.0
        self.orbit_start_time = None
        self.offboard_counter = 0
        self.hover_start_time = None
        self.align_start_time = None
        
        self.winner_target_x = 0.0
        self.winner_target_y = 0.0
        self.descent_z = self.ALTURA_ORBITA

    def normalizar_nome(self, nome):
        subst = {'círculo': 'circulo', 'triângulo': 'triangulo', 'hexágono': 'hexagono', 'pentágono': 'pentagono'}
        nome = str(nome).lower().strip()
        return subst.get(nome, nome)

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = frame.shape
            self.img_center_x = w // 2
            self.img_center_y = h // 2

            results = self.model(frame, conf=0.35, device='cpu', verbose=False)
            alvo_encontrado = False
            maior_conf = 0.0

            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = self.model.names[cls_id]
                    label_clean = self.normalizar_nome(label)

                    if label_clean == self.target_platform:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        self.target_x = int((x1 + x2) / 2)
                        self.target_y = int((y1 + y2) / 2)
                        self.target_confidence = conf
                        alvo_encontrado = True
                        
                        if conf > maior_conf:
                            maior_conf = conf

                        curr_x = self.current_pose.pose.position.x
                        curr_y = self.current_pose.pose.position.y
                        dist_centro = math.hypot(curr_x, curr_y)

                        if dist_centro < 1.5:
                            slot_id = "CENTRO"
                            exact_x, exact_y = 0.0, 0.0
                        else:
                            ang_pos = math.atan2(curr_y, curr_x)
                            if ang_pos < 0: ang_pos += 2 * math.pi
                            
                            slot_index = int(round(ang_pos / self.SLOT_ANGLE_STEP)) % self.NUM_PERIPHERAL_SLOTS
                            slot_id = f"SLOT_{slot_index + 1}"
                            
                            exact_angle = slot_index * self.SLOT_ANGLE_STEP
                            exact_x = self.RAIO_ORBITA * math.cos(exact_angle)
                            exact_y = self.RAIO_ORBITA * math.sin(exact_angle)

                        encontrada_no_banco = False
                        for p in self.analyzed_platforms:
                            if p['id'] == slot_id:
                                encontrada_no_banco = True
                                if conf > p['confidence']:
                                    p['confidence'] = conf
                                break
                        
                        if not encontrada_no_banco:
                            self.analyzed_platforms.append({
                                'id': slot_id,
                                'confidence': conf,
                                'x': exact_x,
                                'y': exact_y
                            })
                            self.get_logger().info(f"[MAPA FIXO] Alvo registrado no {slot_id} ({conf*100:.1f}%) -> Pos: ({exact_x:.2f}, {exact_y:.2f})")
                        break

            self.target_found = alvo_encontrado

        except Exception as e:
            self.get_logger().error(f"[ERRO CAMERA]: {e}")

    def control_loop(self):
        # 1. DECOLAGEM
        if self.state == 'TAKEOFF_SAFETY':
            pose = PoseStamped()
            pose.pose.position.x = self.current_pose.pose.position.x
            pose.pose.position.y = self.current_pose.pose.position.y
            pose.pose.position.z = self.ALTURA_ORBITA
            self.local_pos_pub.publish(pose)

            self.offboard_counter += 1
            if self.offboard_counter % 5 == 0:
                if not self.current_state.armed:
                    arm_cmd = CommandBool.Request()
                    arm_cmd.value = True
                    self.arming_client.call_async(arm_cmd)

                if self.current_state.mode != "OFFBOARD":
                    mode_cmd = SetMode.Request()
                    mode_cmd.custom_mode = 'OFFBOARD'
                    self.set_mode_client.call_async(mode_cmd)

            if self.current_pose.pose.position.z >= (self.ALTURA_ORBITA - 0.3):
                self.get_logger().info("[INÍCIO] Mapeando slots com coordenadas fixas em órbita de 3.5m...")
                self.orbit_start_time = self.get_clock().now().nanoseconds / 1e9
                self.state = 'ORBIT_ANALYSIS'

        # 2. ÓRBITA
        elif self.state == 'ORBIT_ANALYSIS':
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed = now - self.orbit_start_time
            self.orbit_angle = self.orbit_speed * elapsed

            target_x = self.RAIO_ORBITA * math.cos(self.orbit_angle)
            target_y = self.RAIO_ORBITA * math.sin(self.orbit_angle)

            pose = PoseStamped()
            pose.pose.position.x = target_x
            pose.pose.position.y = target_y
            pose.pose.position.z = self.ALTURA_ORBITA
            self.local_pos_pub.publish(pose)

            if self.orbit_angle >= (2 * math.pi):
                self.get_logger().info("[ÓRBITA FIM] Indo ao centro checar a base central...")
                self.state = 'ANALYZE_CENTER'

        # 3. BASE CENTRAL
        elif self.state == 'ANALYZE_CENTER':
            pose = PoseStamped()
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = self.ALTURA_ORBITA
            self.local_pos_pub.publish(pose)

            dist_centro = math.hypot(self.current_pose.pose.position.x, self.current_pose.pose.position.y)
            if dist_centro < 0.4:
                if self.align_start_time is None:
                    self.align_start_time = self.get_clock().now().nanoseconds / 1e9
                
                now = self.get_clock().now().nanoseconds / 1e9
                if (now - self.align_start_time) > 1.5:
                    self.get_logger().info("[DECISÃO] Selecionando o Slot de maior confiança...")
                    
                    if len(self.analyzed_platforms) > 0:
                        winner = max(self.analyzed_platforms, key=lambda p: p['confidence'])
                        self.winner_target_x = winner['x']
                        self.winner_target_y = winner['y']
                        self.get_logger().info(f"[VENCEDORA] Pousando no {winner['id']} ({winner['confidence']*100:.1f}%) em ({winner['x']:.2f}, {winner['y']:.2f})!")
                    else:
                        self.get_logger().warn("[AVISO] Nenhuma base mapeada com precisão. Pousando no centro.")
                        self.winner_target_x = 0.0
                        self.winner_target_y = 0.0

                    self.align_start_time = None
                    self.state = 'GOTO_WINNER'

        # 4. NAVEGAÇÃO
        elif self.state == 'GOTO_WINNER':
            pose = PoseStamped()
            pose.pose.position.x = self.winner_target_x
            pose.pose.position.y = self.winner_target_y
            pose.pose.position.z = self.ALTURA_ORBITA
            self.local_pos_pub.publish(pose)

            dist = math.hypot(self.current_pose.pose.position.x - self.winner_target_x,
                              self.current_pose.pose.position.y - self.winner_target_y)
            
            if dist < 0.35:
                self.get_logger().info("[NAVEGAÇÃO] Chegou ao Slot exato! Alinhando câmera (precisão fina)...")
                self.align_start_time = self.get_clock().now().nanoseconds / 1e9
                self.state = 'ALIGNING'

        # 5. ALINHAMENTO DE ALTA PRECISÃO (< 8 PIXELS DE ERRO)
        elif self.state == 'ALIGNING':
            err_x = self.target_x - self.img_center_x
            err_y = self.target_y - self.img_center_y

            kp_pos = 0.0012
            step_x = float(err_y) * kp_pos
            step_y = -float(err_x) * kp_pos

            step_x = max(-0.10, min(0.10, step_x))
            step_y = max(-0.10, min(0.10, step_y))

            pose = PoseStamped()
            pose.pose.position.x = self.current_pose.pose.position.x + step_x
            pose.pose.position.y = self.current_pose.pose.position.y + step_y
            pose.pose.position.z = self.ALTURA_ORBITA
            self.local_pos_pub.publish(pose)

            now = self.get_clock().now().nanoseconds / 1e9
            elapsed_align = now - self.align_start_time

            # Tolerância reduzida de 20 para 8 pixels de erro
            if (abs(err_x) <= 8 and abs(err_y) <= 8) or elapsed_align > 6.0:
                self.get_logger().info("[ALINHAMENTO] Drone perfeitamente centralizado sobre a plataforma. Iniciando descida com ajuste fino...")
                self.descent_z = self.ALTURA_ORBITA
                self.hover_start_time = self.get_clock().now().nanoseconds / 1e9
                self.state = 'HOVER_STABILIZE'

        # 6. HOVER RÁPIDO
        elif self.state == 'HOVER_STABILIZE':
            pose = PoseStamped()
            pose.pose.position.x = self.current_pose.pose.position.x
            pose.pose.position.y = self.current_pose.pose.position.y
            pose.pose.position.z = self.ALTURA_ORBITA
            self.local_pos_pub.publish(pose)

            now = self.get_clock().now().nanoseconds / 1e9
            if (now - self.hover_start_time) >= 1.5:
                self.get_logger().info("[POUSO] Descida vertical com alinhamento visual ativo...")
                self.state = 'VERTICAL_DESCENT'

        # 7. DESCIDA VERTICAL COM REAJUSTE DE ALINHAMENTO
        elif self.state == 'VERTICAL_DESCENT':
            self.descent_z -= 0.020

            err_x = self.target_x - self.img_center_x
            err_y = self.target_y - self.img_center_y

            kp_pos = 0.0010
            step_x = float(err_y) * kp_pos if self.target_found else 0.0
            step_y = -float(err_x) * kp_pos if self.target_found else 0.0

            step_x = max(-0.05, min(0.05, step_x))
            step_y = max(-0.05, min(0.05, step_y))

            pose = PoseStamped()
            pose.pose.position.x = self.current_pose.pose.position.x + step_x
            pose.pose.position.y = self.current_pose.pose.position.y + step_y
            pose.pose.position.z = max(self.ALTURA_POUSO_FINAL, self.descent_z)
            self.local_pos_pub.publish(pose)

            if self.current_pose.pose.position.z <= (self.ALTURA_POUSO_FINAL + 0.10):
                self.get_logger().info("[SOLO] Pouso no centro da plataforma finalizado. Desarmando...")
                self.state = 'AUTO_LAND_DISARM'

        # 8. DESARME
        elif self.state == 'AUTO_LAND_DISARM':
            set_mode = SetMode.Request()
            set_mode.custom_mode = 'AUTO.LAND'
            self.set_mode_client.call_async(set_mode)

            arm_cmd = CommandBool.Request()
            arm_cmd.value = False
            self.arming_client.call_async(arm_cmd)

            if not self.current_state.armed:
                self.get_logger().info("[SUCESSO TOTAL] MOTORES DESARMADOS E POUSO CONCLUÍDO!")
                self.state = 'FINISHED'

        elif self.state == 'FINISHED':
            rclpy.shutdown()

def main():
    rclpy.init()
    node = AutonomousLandingMission()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
