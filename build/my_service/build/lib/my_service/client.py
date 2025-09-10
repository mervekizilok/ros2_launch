import rclpy
from rclpy.node import Node
from triangle_interfaces.srv import CalculateArea, CalculatePerimeter

class TriangleClient(Node):
    def __init__(self):
        super().__init__('triangle_client')
        self.area_client = self.create_client(CalculateArea, 'calculate_area')
        self.perimeter_client = self.create_client(CalculatePerimeter, 'calculate_perimeter')
        while not self.area_client.wait_for_service(timeout_sec=1.0) or not self.perimeter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Hizmetler mevcut değil, bekleniyor...')
        self.get_logger().info('Üçgen hizmetlerine bağlanıldı.')

    def send_area_request(self, base, height):
        request = CalculateArea.Request()
        request.base = base
        request.height = height
        future = self.area_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_perimeter_request(self, side1, side2, side3):
        request = CalculatePerimeter.Request()
        request.side1 = side1
        request.side2 = side2
        request.side3 = side3
        future = self.perimeter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = TriangleClient()
    while True:
        print("\nSeçenekler: 1 - Alan Hesapla, 2 - Çevre Hesapla, 3 - Çıkış")
        choice = input("Seçim girin: ").strip()
        if choice == '1':
            try:
                base = float(input("Taban girin: "))
                height = float(input("Yükseklik girin: "))
                response = client.send_area_request(base, height)
                print(f"Alan: {response.area}, Mesaj: {response.message}")
            except ValueError:
                print("Geçersiz giriş; lütfen sayı girin.")
        elif choice == '2':
            try:
                side1 = float(input("Kenar 1 girin: "))
                side2 = float(input("Kenar 2 girin: "))
                side3 = float(input("Kenar 3 girin: "))
                response = client.send_perimeter_request(side1, side2, side3)
                print(f"Çevre: {response.perimeter}, Mesaj: {response.message}")
            except ValueError:
                print("Geçersiz giriş; lütfen sayı girin.")
        elif choice == '3':
            print("Programdan çıkılıyor.")
            break
        else:
            print("Geçersiz seçim; tekrar deneyin.")
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
