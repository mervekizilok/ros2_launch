
import rclpy                               # ROS 2 Python kütüphanesi
from rclpy.node import Node                # Node (düğüm) sınıfı
from std_msgs.msg import Int32MultiArray   # Birden çok int32 göndermek için mesaj tipi
import random                              # Rastgele sayı üretmek için

class RandomFivePublisher(Node):
    def __init__(self):
        
        # Node (düğüm) adını verelim.
        super().__init__('random_five_publisher')

        # Publisher oluşturalım.
        self.publisher_ = self.create_publisher(Int32MultiArray, 'random_five_numbers', 10)

        # Kaç saniyede bir yayın yapılacağını belirleyen timer yazalım.(1.0 sn = 1 Hz)
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('RandomFivePublisher çalışıyor...')

    def timer_callback(self):
        
        # 5 farklı sayı seçmek için
        numbers = random.sample(range(1, 1001), 5)

        # Mesajı oluşturalım ve sayıları data alanına koyalım.
        msg = Int32MultiArray()
        msg.data = numbers

        # Mesajı yayınlayalım.
        self.publisher_.publish(msg)

        self.get_logger().info(f'data: {numbers}')

def main(args=None):
    
    # ROS 2’yi başlatmak için
    rclpy.init(args=args)

    # Node oluşturmak için
    node = RandomFivePublisher()

    # Node'u çalışır halde tutmak için
    rclpy.spin(node)

    # Çıkışta kaynakları serbest bırakalım.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
