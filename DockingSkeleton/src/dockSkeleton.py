# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# Create3 packages
import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock
# Your ROS Node packages
from check_sensor import DockStatusPublisher
# Garrett packages (Easy program start)
from pynput.keyboard import KeyCode
from key_commander import KeyCommander
# Python packages
from std_msgs.msg import String

# Globals
rclpy.init()
namespace = 'create3_03B8'
sensor = DockStatusPublisher(namespace)

class Roomba(Node):
	def __init__(self, namespace):
		super().__init__("robot")
		
		# Callback Groups
		cb_Subscripions = MutuallyExclusiveCallbackGroup()
		cb_Actions = MutuallyExclusiveCallbackGroup()
		
		# Subscriptions
		self.subscription = self.create_subscription(String,'/check_dock_status',
		    self.listener_callback, 10, callback_group=cb_Subscripions)

		# Actions
		self.undock_ac = ActionClient(self, Undock, f'/{namespace}/undock',
			callback_group=cb_Actions)
		self.drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance',
			callback_group=cb_Actions)
	
	def listener_callback(self,msg):
		"""
		This function will run when the subscription receives a message from the publisher
		"""
		print("I got: ",msg.data)
		
	def drive(self):
		sensor.poll() # Read current dock status
		# Undock
		self.undock_ac.wait_for_server() # Wait till its ready
		undock_goal = Undock.Goal() # Make goal
		self.undock_ac.send_goal(undock_goal) # Send goal blocking
		# Drive
		self.drive_ac.wait_for_server()
		drive_goal = DriveDistance.Goal()
		drive_goal.distance = 1.0 # 1 meter
		self.drive_ac.send_goal(drive_goal)
		
		sensor.poll() # Read current dock status
		
		
		
		
if __name__ == '__main__':
	roomba = Roomba(namespace)
	exec = MultiThreadedExecutor(3)
	exec.add_node(roomba)
	exec.add_node(sensor)
    	
	keycom = KeyCommander([
		(KeyCode(char='s'), roomba.drive),
		])
	print("S")

	# Try/Except to shutdown "gracefully"
	try:
		exec.spin()
	except KeyboardInterrupt:
		rclpy.shutdown()
		
