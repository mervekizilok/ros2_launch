
import rclpy                               
from rclpy.node import Node                
from std_msgs.msg import Int32MultiArray   # Publisher’ın gönderdiği mesaj tipi

class RandomFiveSubscriber(Node):
    def __init__(self):
        
        super().__init__('random_five_subscriber')

        # Subscriber oluşturalım: konu adı, mesaj tipi, callback fonksiyonu
        self.subscription = self.create_subscription(
            Int32MultiArray,               # Mesaj tipi
            'random_five_numbers',         # Publisher’ın yayınladığı konu
            self.listener_callback,        # Callback fonksiyonu
            10                             # queue size
        )

        self.get_logger().info('RandomFiveSubscriber çalışıyor...')

    def listener_callback(self, msg):
        
        numbers = msg.data  # Publisher’dan gelen 5 sayı

        # Kaç tanesi çift?
        even_count = sum(1 for n in numbers if n % 2 == 0)

        # Toplam ve ortalamayı hesaplayalım.
        total = sum(numbers)
        average = total / len(numbers)

        # Sonuçları loglayalım.
        self.get_logger().info(
            f'Gelen sayılar: {numbers} | Çift: {even_count} | Toplam: {total} | Ortalama: {average:.2f}'
        )

def main(args=None):
    
    rclpy.init(args=args)

    node = RandomFiveSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
