import numpy as np
import rclpy
from random import randrange
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped



class Autopilot(Node):

    def __init__(self):
        super().__init__('autopilot')
        """Initialize autopilot parameters:

        potential_pos (OccupancyGrid): Occupancy grid index associated with next potential position

        occupancy_grid (OccupancyGrid): Current occupancy grid information received from '/map'. References callback function 'next_waypoint'
        
        """
        #Allow callback functions to be called in parallel
        self.parallel_callback_group = ReentrantCallbackGroup()

        #Initializing x and y coordinates of Turtlebot in space, to be populated later
        self.new_waypoint = PoseStamped()
        self.new_waypoint.header.frame_id = 'map'
        self.new_waypoint.pose.position.x = 0.0
        self.new_waypoint.pose.position.y = 0.0
        self.new_waypoint.pose.orientation.w = 1.0

        #Initialize variable to capture behavior tree state
        self.ready = True

        #Subscribe to /behavior_tree_log to determine when Turtlebot is ready for a new waypoint
        self.behaviortreelogstate = self.create_subscription(BehaviorTreeLog, 'behavior_tree_log', self.readiness_check, 10, callback_group=self.parallel_callback_group)

        #Subscribe to OccupancyGrid type topic "/map"
        self.potential_pos = OccupancyGrid()
        self.occupancy_grid = self.create_subscription(OccupancyGrid, '/map', self.next_waypoint, 10)
        
        #Create publisher to publish next waypoint parameters to
        self.waypoint_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        #Track number of waypoints sent
        self.waypoint_counter = 0.0

    def readiness_check(self, msg:BehaviorTreeLog):
        #If latest node state of /behavior_tree_log is "NavigateRecovery" and event status is "IDLE", send next waypoint
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and event.current_status =='IDLE':
                self.ready = True
                #self.next_waypoint(occupancy_data=self.occupancy_grid)

            elif event.node_name == 'ComputePathToPose' and event.current_status =='FAILURE':
                self.ready = True

            elif event.node_name == 'FollowPath' and event.current_status =='SUCCESS':
                self.ready = True

            elif event.node_name == 'FollowPath' and event.current_status =='FAILURE':
                self.ready = True
                
            else:
                self.get_logger().info('Event Node Name:')
                self.get_logger().info(event.node_name)
                self.get_logger().info('Event Node Status:')
                self.get_logger().info(event.current_status)
                return
            
    def frontier_check(self, occupancy_data, random_index):
        """
        Checks if the point we've selected is on the edge of the frontier, but isn't very close to obstacles
        """
        uncertain_indexes = 0
        obstacle_indexes = 0
        #Verifies points in a 6x6 grid around the selected point

        for x in range(-2,3):
            for y in range(-2,3):
                row_index = x * self.width + y
                try:
                    if occupancy_data.data[random_index + row_index] == -1:
                        uncertain_indexes += 1
                    elif occupancy_data.data[random_index + row_index] > 85:
                        obstacle_indexes += 1
                #the index of a point next to the random_index may not be within the range of occupancy_data.data, so the IndexError is handled below
                except IndexError:
                    pass
        #if the point in question (random_index) is next to at least one uncertain_index and not next to between 2 and 4 obstacles, then this is a valid index along the frontier
        if uncertain_indexes > 1 and 2 < obstacle_indexes < 4:
            return True
        else:
            return False

    def next_waypoint(self, occupancy_data):
        """Callback function to choose next waypoint when new occupancy grid is received, and old goal is either destroyed or achieved

        Args:
        self (Node): Autopilot node currently running and storing waypoint decisions 
        occupancy_data (OccupancyGrid): map data array from OccupancyGrid type

        """
        #Publish new waypoint to '/goal_pose' if behavior tree is ready
        if self.ready == False:
            self.get_logger().info('Waiting for last command to execute')
            return

        resolution = 0.05
        origin_x = occupancy_data.info.origin.position.x
        self.width = occupancy_data.info.width
        isthisagoodwaypoint = False

        while isthisagoodwaypoint == False:
            random_index = randrange(len(occupancy_data.data))
            self.potential_pos = occupancy_data.data[random_index]
            frontier_detection= self.frontier_check(occupancy_data, random_index)

            if self.potential_pos != -1 and self.potential_pos <= 20 and frontier_detection == True:
                self.get_logger().info('Found Good Point with OccupancyData:')
                self.get_logger().info(str(self.potential_pos))
                isthisagoodwaypoint = True

            else:
                self.get_logger().info('Bad Point Finding New')

        row_index = random_index / self.width
        col_index = random_index % self.width

        self.new_waypoint.pose.position.x = (col_index * resolution) + origin_x + (resolution/2)
        self.new_waypoint.pose.position.y = (row_index * resolution) + origin_x + (resolution/2)

        self.waypoint_publisher.publish(self.new_waypoint)
        self.ready = False

def main():
    rclpy.init()
    autopilot_node = Autopilot()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(autopilot_node)
    executor.spin()
    autopilot_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()