#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros

from tf2_ros import TransformException
from geometry_msgs.msg import Vector3, Twist, PoseWithCovarianceStamped


# ============================================================
# Funções Auxiliares
# ============================================================

def normalize(v: np.ndarray) -> np.ndarray:
    """Normaliza um vetor numpy."""
    norm = np.linalg.norm(v)
    return v if norm == 0 else v / norm


def quaternion_to_yaw(q_ros) -> float:
    """Converte um quaternion em ângulo yaw (rotação em torno de Z)."""
    x, y, z, w = q_ros.x, q_ros.y, q_ros.z, q_ros.w
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


# ============================================================
# Classe Principal do Nó
# ============================================================

class VectorFollowerNode(Node):
    """Nó ROS 2 responsável por seguir um vetor de velocidade publicado em /vec_to_follow."""

    def __init__(self):
        super().__init__("vector_follower_node")

        # -------------------------
        # Parâmetros
        # -------------------------
        self.declare_parameters(
            namespace="",
            parameters=[
                ("distancia_ponto_controle", 0.15),
                ("const_vel", 0.3),
                ("const_omega", 2.0),
                ("stuck_timeout", 3.0),
                ("escape_duration", 2.0),
                ("noise_magnitude", 0.4),
            ],
        )

        self.distancia_ponto_controle = self.get_parameter("distancia_ponto_controle").value
        self.const_vel = self.get_parameter("const_vel").value
        self.const_omega = self.get_parameter("const_omega").value
        self.STUCK_TIMEOUT = self.get_parameter("stuck_timeout").value
        self.ESCAPE_DURATION = self.get_parameter("escape_duration").value
        self.NOISE_MAGNITUDE = self.get_parameter("noise_magnitude").value

        # -------------------------
        # TF Listener
        # -------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -------------------------
        # Publishers e Subscribers
        # -------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Vector3, "/vec_to_follow", self.vector_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)

        # -------------------------
        # Variáveis de estado
        # -------------------------
        self.current_vector = None
        self.theta = None
        self.stuck_timer = 0.0
        self.escape_timer = 0.0
        self.escape_vector = np.zeros(2)

        # -------------------------
        # Timer de controle
        # -------------------------
        self.timer_period = 0.5  # segundos
        self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info("✅ Nó Vector Follower iniciado com sucesso.")

    # ============================================================
    # Callbacks
    # ============================================================

    def vector_callback(self, msg: Vector3):
        """Atualiza o vetor de velocidade desejada."""
        self.current_vector = msg

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Atualiza o ângulo de orientação (theta) do robô."""
        self.theta = quaternion_to_yaw(msg.pose.pose.orientation)

    # ============================================================
    # Lógica Principal
    # ============================================================

    def control_loop(self):
        """Loop de controle que converte o vetor desejado em comandos de velocidade."""

        if self.current_vector is None:
            self.get_logger().info("Aguardando vetor em /vec_to_follow...", throttle_duration_sec=5)
            return

        if self.theta is None:
            self.get_logger().info("Aguardando primeira pose de /amcl_pose...", throttle_duration_sec=5)
            return

        dt = self.timer_period
        psi_des = np.array([self.current_vector.x, self.current_vector.y])

        # --- Manobra de escape ---
        if self.escape_timer > 0.0:
            psi_des += self.escape_vector
            self.escape_timer = max(0.0, self.escape_timer - dt)
            if self.escape_timer == 0.0:
                self.get_logger().info("Manobra de escape concluída.")

        # --- Conversão para comandos lineares e angulares ---
        c, s = np.cos(self.theta), np.sin(self.theta)
        V_final = psi_des[0] * c + psi_des[1] * s
        w_final = (-psi_des[0] * s + psi_des[1] * c) / self.distancia_ponto_controle

        # --- Detecção de travamento ---
        if self._is_stuck(psi_des, V_final):
            self._trigger_escape()

        # --- Saturação das velocidades ---
        V_final = np.clip(V_final, -self.const_vel, self.const_vel)
        w_final = np.clip(w_final, -self.const_omega, self.const_omega)

        # --- Publica o comando ---
        twist = Twist()
        twist.linear.x = float(V_final)
        twist.angular.z = float(w_final)
        self.cmd_vel_pub.publish(twist)

    # ============================================================
    # Funções Auxiliares Internas
    # ============================================================

    def _is_stuck(self, psi_des: np.ndarray, V_final: float) -> bool:
        """Retorna True se o robô estiver preso."""
        velocidade_desejada = np.linalg.norm(psi_des) > 0.1
        velocidade_nula = abs(V_final) < 0.05

        if velocidade_desejada and velocidade_nula and self.escape_timer == 0.0:
            self.stuck_timer += self.timer_period
        else:
            self.stuck_timer = 0.0

        return self.stuck_timer > self.STUCK_TIMEOUT

    def _trigger_escape(self):
        """Inicia uma manobra de escape aleatória."""
        self.get_logger().warn("⚠️ Robô possivelmente preso! Iniciando manobra de escape.")
        self.stuck_timer = 0.0
        self.escape_timer = self.ESCAPE_DURATION
        random_vec = normalize(np.random.randn(2))
        self.escape_vector = random_vec * self.NOISE_MAGNITUDE


def main(args=None):
    rclpy.init(args=args)
    node = VectorFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
