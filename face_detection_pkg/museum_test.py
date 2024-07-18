import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

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
            "area_1": {"id": "Monalisa", "x": 9.036503791809082, "y": -0.02560051530599594},
            # "area_2": {"id": "id 2", "x": 0.0, "y": 2.0},
            # Add more areas as needed
        }
        self.facts = {
            "fact_1": {"id": "Monalisa", "fact": "Monalisa is also known as Gioconda and was painted by Leonardo DaVinci. It is thought that she was his lover at the time!"}
        }

        self.last_zone = None

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_zone = None

        for area_name, area in self.areas.items():
            # print(area_name)
            # print(area)


            if ( x-2 <= x <= area["x"] ):
                # print("here x")    
                if (area["y"] - 1 <= y <= area["y"] + 1 ):
                    # Here means that we entered an area\
                    current_zone = area["id"]
                    if self.last_zone != current_zone: 
                        self.last_zone = current_zone
                        self.get_logger().info(f'Robot entered {area["id"]}')
                        for fact_name, fact in self.facts.items():
                            if fact["id"] == area["id"]:
                                self.get_logger().info(f'Fact: {fact["fact"]}')
                    break
                
        if current_zone is None:
            self.last_zone = None 


def main(args=None):
    rclpy.init(args=args)
    area_checker = AreaChecker()
    rclpy.spin(area_checker)
    area_checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
