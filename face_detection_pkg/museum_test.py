import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

def calculate_distance(x1, y1, x2, y2):
        distance = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
        return distance

class AreaChecker(Node):
    def __init__(self):
        super().__init__('area_checker')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        self.subscription  

        self.areas = {
            # "area_1": {"id": "Monalisa", "x": 9.036503791809082, "y": -0.02560051530599594},
            # "area_2": {"id": "test", "x": 9.0110111236572, "y": 5.947264671325684},
            "area_2": {"id": "Starry Night", "x": 2.28, "y": -1.77},
            "area_3": {"id": "The Persistence of Memory", "x": 5.27, "y": 0.01},
            "area_4": {"id": "The Scream", "x": 8.16, "y": 1.84},
            # "area_2": {"id": "id 2", "x": 0.0, "y": 2.0},
            # Add more areas as needed
        }
        self.facts = {
            # "fact_1": {"id": "Monalisa", "fact": "Monalisa is also known as Gioconda and was painted by Leonardo DaVinci. It is thought that she was his lover at the time!"},
            # "fact_2": {"id": "test", "fact": "TEST"},
           
            "fact_2": {"id": "Starry Night", "fact": "Vincent van Gogh painted Starry Night while he was in a mental asylum in Saint-Rémy-de-Provence, France. He considered it a failure at the time."},
            "fact_3": {"id": "The Persistence of Memory", "fact": "The melting clocks in The Persistence of Memory by Salvador Dali were inspired by the surrealist perception of Camembert cheese melting in the sun."},
            "fact_4": {"id": "The Scream", "fact": "Edvard Munch was inspired to paint The Scream after experiencing a vivid hallucination of the sky turning blood red while he was out walking at sunset. He described the experience as hearing the enormous, infinite scream of nature."},
        }

        self.last_zone = None
    
    
    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_zone = None
        min_distance = float('inf')

        for area_name, area in self.areas.items():
            poi_x = area["x"]
            poi_y = area["y"]
            threshold = area.get("threshold", 1.5)  # Default threshold to 1.5

            # Calculate Euclidean distance
            distance = calculate_distance(x, y, poi_x, poi_y)

            if distance <= threshold and distance < min_distance:
                min_distance = distance
                current_zone = area["id"]

        if current_zone is not None:
            if self.last_zone != current_zone:
                self.last_zone = current_zone
                self.get_logger().info(f'Robot entered {current_zone}')
                for fact_name, fact in self.facts.items():
                    if fact["id"] == current_zone:
                        self.get_logger().info(f'Fact: {fact["fact"]}')
        else:
            self.last_zone = None


def main(args=None):
    rclpy.init(args=args)
    area_checker = AreaChecker()
    rclpy.spin(area_checker)
    area_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
