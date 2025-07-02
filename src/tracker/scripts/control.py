import rospy

from geometry_msgs.msg import PoseStamped

from mavros_msgs.msg import RCIn
from quadrotor_msgs.msg import TakeoffLand

__author__ = "YueLin"

TAKEOFF, LAND = 1, 2
UP, MID, DOWN = 0x3E2, 0x5D6, 0x7CA  # The states of the remote controller


class RemoteController:
    """For the real-world remote controller"""
    def __init__(self, node: str):
        
        # State of the remote controler
        self.init = False
        self.state = DOWN

        # Initialize ROS
        rospy.init_node(node)

        # Messages
        self.flight = TakeoffLand()
        self.trigger = PoseStamped()

        # Publishers
        self.start = rospy.Publisher(
            "/triger", PoseStamped, queue_size=1
        )
        self.stop = rospy.Publisher(
            "/back_trigger", PoseStamped, queue_size=1
        )
        self.publisher = rospy.Publisher(
            "/l_ctrl/takeoff_land", TakeoffLand, queue_size=1
        )

        # Subscriber
        rospy.Subscriber("/mavros/rc/in", RCIn, self.callback, queue_size=10)
    
    def callback(self, state: RCIn) -> None:
        # Get the state of the 8th channel of the remote controller
        state = state.channels[7]

        # The initial state must be DOWN
        if not self.init and self.state != DOWN:
            rospy.logerr("The initial state of channle 8 must be DOWN")
        else:
            self.init = True

            # DOWN -> MID: Takeoff
            if self.state == DOWN and state == MID:
                self.flight.takeoff_land_cmd = TAKEOFF
                self.publisher.publish(self.flight)
                rospy.loginfo("Takeoff")
                self.state = state
            
            # MID -> DOWN: Land
            elif self.state == MID and state == DOWN:
                self.flight.takeoff_land_cmd = LAND
                self.publisher.publish(self.flight)
                rospy.logwarn("Landing")
                self.state = state
            
            # MID -> UP: Start tracking
            elif self.state == MID and state == UP:
                self.trigger.header.stamp = rospy.Time.now()
                self.start.publish(self.trigger)
                rospy.loginfo("Start tracking")
                self.state = state
            
            # UP -> MID: Stop tracking
            elif self.state == UP and state == MID:
                self.trigger.header.stamp == rospy.Time.now()
                self.stop.publish(self.trigger)
                rospy.logwarn("Stop tracking")
                self.state = state
    
    @staticmethod
    def run():
        rospy.spin()


if __name__ == "__main__":
    RemoteController("remote_control").run()
