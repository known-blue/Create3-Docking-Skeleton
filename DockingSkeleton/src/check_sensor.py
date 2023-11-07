import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String

import irobot_create_msgs
from irobot_create_msgs.msg import DockStatus


class DockStatusPublisher(Node):

    def __init__(self, namespace):
        super().__init__('publisher')
        
        # This is what 'sends' the data to the 'main' node
        self.publisher_ = self.create_publisher(String, 'check_dock_status', 10) #return type, topic name, queue size
        # This subscribes to the /dock_status topic
        self.subscriber_ = self.create_subscription(DockStatus, f'/{namespace}/dock_status', 
            self.listener, qos_profile_sensor_data)
        # This will hold the current value from the topic
        self.curValue = '' # Blank inital string
        
    def listener(self, msg):
        """
        This will be called everytime the subscriber recieves a new message from the topic
        """
    	self.curValue = str(msg.is_docked)
    	
    def poll(self):
        """
        This will be called by the 'main' node to prevent overloading it with messages
        """
        msg = String()
        msg.data = self.curValue
        self.publisher_.publish(msg)
