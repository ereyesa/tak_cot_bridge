import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import socket
import yaml
import os
import threading
import time

from tak_cot_bridge.cot_utils import build_cot_event


class CotPublisherNode(Node):
    def __init__(self):
        super().__init__('cot_publisher_node')

        # Parámetros ROS (con valores por defecto)
        self.declare_parameter('tak_server_ip', '127.0.0.1')
        self.declare_parameter('tak_server_port', 8087)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('retry_timeout_sec', 5.0)
        self.declare_parameter('max_retries', 10)
        self.declare_parameter('robot_uid', self.get_name())

        # Obtener parámetros
        self.tak_ip = self.get_parameter('tak_server_ip').get_parameter_value().string_value
        self.tak_port = self.get_parameter('tak_server_port').get_parameter_value().integer_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.retry_timeout_sec = self.get_parameter('retry_timeout_sec').get_parameter_value().double_value
        self.max_retries = self.get_parameter('max_retries').get_parameter_value().integer_value
        self.uid = self.get_parameter('robot_uid').get_parameter_value().string_value

        self.get_logger().info(f"Configuración TAKServer -> IP: {self.tak_ip}, Puerto: {self.tak_port}")
        self.get_logger().info(f"Frecuencia publicación: {self.publish_rate_hz} Hz, Retry timeout: {self.retry_timeout_sec} s, Max retries: {self.max_retries}")

        # Socket UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.retry_timeout_sec)

        # Variables para almacenar último GPS recibido
        self.latest_gps_msg = None
        self.gps_lock = threading.Lock()

        # Suscripción al tópico GPS MAVROS
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)

        # Timer para publicar eventos CoT periódicamente
        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Contador de intentos de conexión
        self.retry_count = 0

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            self.get_logger().warn("GPS fix inválido, no se actualizará posición para CoT")
            return
        with self.gps_lock:
            self.latest_gps_msg = msg

    def timer_callback(self):
        with self.gps_lock:
            msg = self.latest_gps_msg

        if msg is None:
            self.get_logger().warn("No hay datos GPS recibidos aún, no se enviará evento CoT")
            return

        cot_msg = build_cot_event(self.uid, msg.latitude, msg.longitude)
        self.get_logger().debug(f"Preparando para enviar CoT: {cot_msg}")

        sent = self.send_cot_with_retry(cot_msg)
        if sent:
            self.retry_count = 0
        else:
            self.retry_count += 1
            self.get_logger().warn(f"Intento {self.retry_count}/{self.max_retries} para enviar CoT fallido")

        if self.retry_count >= self.max_retries:
            self.get_logger().error(f"No se pudo conectar con TAKServer después de {self.max_retries} intentos. Se suspende envío temporalmente.")

    def send_cot_with_retry(self, cot_msg: str) -> bool:
        """
        Intenta enviar el mensaje CoT al TAKServer con reintentos y timeout.
        Retorna True si se envió correctamente, False si falló.
        """
        try:
            self.sock.sendto(cot_msg.encode(), (self.tak_ip, self.tak_port))
            # No hay respuesta directa por UDP, consideramos enviado si no lanza excepción
            self.get_logger().info(f"Evento CoT enviado a {self.tak_ip}:{self.tak_port}")
            return True
        except socket.timeout:
            self.get_logger().warn(f"Timeout enviando CoT a {self.tak_ip}:{self.tak_port}")
        except Exception as e:
            self.get_logger().error(f"Error enviando CoT: {e}")
        return False

    # ======================
    # Estructura para futuras funciones, se pueden implementar de forma similar:
    #
    # def battery_callback(self, msg: BatteryState):
    #     # Convertir estado batería a evento CoT y enviarlo
    #     pass
    #
    # def state_callback(self, msg: State):
    #     # Convertir estado del vehículo a evento CoT y enviarlo
    #     pass
    #
    # def imu_callback(self, msg: Imu):
    #     # Convertir datos IMU a evento CoT o enriquecimiento
    #     pass
    #
    # Estas funciones se suscribirían a sus tópicos y la lógica de envío puede 
    # compartirse con send_cot_with_retry para mantener consistencia.
    # ======================

def main(args=None):
    import rclpy
    from rclpy.node import Node
    from tak_cot_bridge.cot_publisher_node import CotPublisherNode

    rclpy.init(args=args)
    node = CotPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()