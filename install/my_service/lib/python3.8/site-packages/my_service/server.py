import rclpy
from rclpy.node import Node
from triangle_interfaces.srv import CalculateArea, CalculatePerimeter

class TriangleServer(Node):
    def __init__(self):
        super().__init__('triangle_server')
        self.area_srv = self.create_service(CalculateArea, 'calculate_area', self.calculate_area_callback)
        self.perimeter_srv = self.create_service(CalculatePerimeter, 'calculate_perimeter', self.calculate_perimeter_callback)
        self.get_logger().info('Üçgen hizmetleri hazır.')

    def calculate_area_callback(self, request, response):
        if request.base <= 0 or request.height <= 0:
            response.area = 0.0
            response.message = "Geçersiz giriş: taban ve yükseklik pozitif olmalı."
        else:
            response.area = (request.base * request.height) / 2.0
            response.message = "Başarılı"
        self.get_logger().info(f'Alan isteği: taban={request.base}, yükseklik={request.height}')
        return response

    def calculate_perimeter_callback(self, request, response):
        sides = [request.side1, request.side2, request.side3]
        if any(s <= 0 for s in sides):
            response.perimeter = 0.0
            response.message = "Geçersiz giriş: kenarlar pozitif olmalı."
        elif (request.side1 + request.side2 <= request.side3 or
              request.side1 + request.side3 <= request.side2 or
              request.side2 + request.side3 <= request.side1):
            response.perimeter = 0.0
            response.message = "Üçgen oluşturmaz (üçgen eşitsizliği ihlali)."
        else:
            response.perimeter = sum(sides)
            response.message = "Başarılı"
        self.get_logger().info(f'Çevre isteği: kenarlar={sides}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TriangleServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
