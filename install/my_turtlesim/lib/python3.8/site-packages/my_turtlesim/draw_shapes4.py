import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
import math

class TurtleController:
    def __init__(self, node, name, x, y, sides):
        self.node = node
        self.name = name
        self.sides = sides
        self.pub = None
        self.step = 0
        self.phase = 'move'
        self.timer = None

        self.linear_speed = 1.0       # m/s
        self.angular_speed = 1.0      # rad/s

        # Kenar uzunluğunu şekle göre ayarla 
        if sides == 3:
            self.move_distance = 2.5
        elif sides == 4:
            self.move_distance = 2.0    
        elif sides == 5:
            self.move_distance = 1.5    
        elif sides == 6:
            self.move_distance = 1.2    
        elif sides == 'star':
            self.move_distance = 1.5
        else:
            self.move_distance = 2.0

        # Açılar ve dönüş yönleri
        if sides == 'star':
            self.total_steps = 5
            self.angle = 4 * math.pi / 5  # 144 derece yıldız dış açısı
            self.turn_direction = 1
        else:
            self.total_steps = sides
            self.angle = 2 * math.pi / sides  # Tam açı
            if sides == 4:
                self.turn_direction = -1  # kare için saat yönü
            else:
                self.turn_direction = 1

        self.spawn_and_start(x, y)

    def spawn_and_start(self, x, y):
        client = self.node.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'{self.name} için spawn servisi bekleniyor...')
        req = Spawn.Request()
        req.name = self.name
        req.x = float(x)
        req.y = float(y)
        req.theta = 0.0

        future = client.call_async(req)

        def after_spawn(fut):
            self.pub = self.node.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
            self.node.get_logger().info(f"{self.name} oluşturuldu.")
            self.start_timer()

        future.add_done_callback(after_spawn)

    def start_timer(self):
        self.action_time = 0.0
        self.phase = 'move'
        self.step = 0
        self.timer = self.node.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        if self.step >= self.total_steps:
            self.pub.publish(Twist())
            self.timer.cancel()
            self.node.get_logger().info(f"{self.name} çizimini tamamladı.")
            return

        msg = Twist()
        dt = 0.05
        self.action_time += dt

        if self.phase == 'move':
            move_duration = self.move_distance / self.linear_speed
            if self.action_time <= move_duration:
                msg.linear.x = self.linear_speed
            else:
                msg.linear.x = 0.0
                self.pub.publish(msg)
                self.phase = 'pause_after_move'
                self.action_time = 0.0

        elif self.phase == 'pause_after_move':
            if self.action_time >= 0.1:
                self.phase = 'rotate'
                self.action_time = 0.0

        elif self.phase == 'rotate':
            rotate_duration = self.angle / self.angular_speed
            if self.action_time <= rotate_duration:
                msg.angular.z = self.turn_direction * self.angular_speed
            else:
                msg.angular.z = 0.0
                self.pub.publish(msg)
                self.phase = 'pause_after_rotate'
                self.action_time = 0.0
                self.step += 1

        elif self.phase == 'pause_after_rotate':
            if self.action_time >= 0.1:
                self.phase = 'move'
                self.action_time = 0.0

        self.pub.publish(msg)


class ShapeDrawer(Node):
    def __init__(self):
        super().__init__('shape_drawer')

        self.kill_turtle1()

        self.turtles = [
            TurtleController(self, 't1', 2.0, 2.0, 3),
            TurtleController(self, 't2', 6.0, 2.0, 4),
            TurtleController(self, 't3', 9.0, 2.0, 5),
            TurtleController(self, 't4', 4.0, 8.0, 6),
            TurtleController(self, 't5', 10.0, 9.0, 'star')
        ]

    def kill_turtle1(self):
        client = self.create_client(Kill, 'kill')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("kill servisi bekleniyor...")
        req = Kill.Request()
        req.name = 'turtle1'
        future = client.call_async(req)

        def after_kill(fut):
            self.get_logger().info("Varsayılan turtle1 silindi.")

        future.add_done_callback(after_kill)


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__=='__main__':
	main()

if __name__ == '__main__':
    main()
