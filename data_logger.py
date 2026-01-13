import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import csv
import datetime
import math

class NavLogger(Node):
    def __init__(self):
        super().__init__('nav_logger')
        
        # Dosya ismi (Tarihli olsun ki karışmasın)
        self.filename = f"log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        # CSV Başlıklarını Yaz
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Tarih_Saat", "Konum_X", "Konum_Y", "Yon_Z", "Yon_W"])
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Saniyede 2 kez kayıt al (0.5 sn)
        self.timer = self.create_timer(0.5, self.log_position)
        self.get_logger().info(f"Kayıt Başladı! Dosya: {self.filename}")

    def log_position(self):
        try:
            # Harita (map) üzerindeki robot (base_link) konumunu sor
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            now = self.get_clock().now()
            timestamp = now.nanoseconds / 1e9
            readable_time = datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.rotation.z
            w = t.transform.rotation.w

            # Veriyi dosyaya ekle
            with open(self.filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([f"{timestamp:.2f}", readable_time, f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", f"{w:.4f}"])
                
            print(f"Loglandı: X={x:.2f}, Y={y:.2f}", end='\r')

        except Exception as e:
            pass # Henüz TF gelmediyse sessizce bekle

def main(args=None):
    rclpy.init(args=args)
    node = NavLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()