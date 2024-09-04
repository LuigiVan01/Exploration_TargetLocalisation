import numpy as np
import rclpy
import math
from random import randrange
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseWithCovarianceStamped
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

        #Initilizing the probablity at which we consider there to be an obstacle
        self.obstacle_probability = 55

        #Initializing x and y coordinates of Turtlebot in space, to be populated later
        self.new_waypoint = PoseStamped()
        self.new_waypoint.header.frame_id = 'map'
        self.new_waypoint.pose.position.x = 0.0
        self.new_waypoint.pose.position.y = 0.0
        self.new_waypoint.pose.orientation.w = 1.0

        #Initializing current position variable
        self.current_position = PoseStamped()

        #Initializing behaviortreelog node name and status
        self.last_node_name = 'string'
        self.last_node_status = 'string'
    
        #Initializing waiting counter
        self.waiting_counter = 0

        #Initializing current state of waypoint searching
        self.searching_for_waypoint = False

        #Initialize variable to capture behavior tree state
        self.ready = True

        #Subscribe to /behavior_tree_log to determine when Turtlebot is ready for a new waypoint
        self.behaviortreelogstate = self.create_subscription(BehaviorTreeLog, 'behavior_tree_log', self.readiness_check, 10, callback_group=self.parallel_callback_group)

        #Subscribe to OccupancyGrid type topic "/map"
        self.potential_pos = OccupancyGrid()
        self.occupancy_grid = self.create_subscription(OccupancyGrid, '/map', self.next_waypoint, 10)

        #Subscribe to /pose to determine position of Turtlebot
        self.position_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.current_position_callback, 10, callback_group=self.parallel_callback_group)

        
        #Create publisher to publish next waypoint parameters to
        self.waypoint_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        #Track number of waypoints  sent
        self.waypoint_counter = 0.0



    def next_waypoint(self, occupancy_data):
        """Callback function to choose next waypoint when new occupancy grid is received, and old goal is either destroyed or achieved

        Args:
        self (Node): Autopilot node currently running and storing waypoint decisions 
        occupancy_data (OccupancyGrid): map data array from OccupancyGrid type

        """
        #DEBUGGING CONDITIONAL TO FIGURE OUT THE BEHAVIORTREELOG STATUS WHEN SYSTEM IS STUCK. MAYBE AN IMPLEMENTATION OF CHOOSING A NEW WAYPOINT WHEN ONE HAS NOT BEEN CHOSEN IN X AMOUNT OF TIME IS A BETTER IDEA
        if self.ready == False:
            self.get_logger().info('Waiting for last command to execute')
            self.waiting_counter += 1 
            if self.waiting_counter > 3:
                self.get_logger().info(str(self.last_node_name))
                self.get_logger().info(str(self.last_node_status))
            return

        resolution = 0.05
        origin_x = occupancy_data.info.origin.position.x
        self.width = occupancy_data.info.width
        isthisagoodwaypoint = False
        min_distance = 8
        max_distance = float('inf')
        self.searching_for_waypoint = True
        occupancy_data_np = np.array(occupancy_data.data)
        occupancy_data_np_checked = []
    
        while isthisagoodwaypoint == False:

            self.get_logger().info('Searching for good point...')

            # Taking a random cell  
            random_index = randrange(occupancy_data_np.size)
            if random_index in occupancy_data_np_checked:
                continue

            # Check that the cell is not unknown or an obstacle
            self.potential_pos = occupancy_data_np[random_index]
            if self.potential_pos==-1:
                continue
            if self.potential_pos>= self.obstacle_probability:
                self.get_logger().info('Point was an obstacle')
                occupancy_data_np_checked = np.append(occupancy_data_np, random_index)
                continue
            
            # Check that the point is on the frontier
            frontier_detection= self.frontier_check(occupancy_data_np, random_index)
            if frontier_detection==False:
                self.get_logger().info('Point was not on frontier')
                occupancy_data_np_checked = np.append(occupancy_data_np, random_index)
                continue

            #CRITERIA FOR A 'GOOD' WAYPOINT  
            if  self.potential_pos <= 20 :
                self.get_logger().info('Found Good Point')
                row_index = random_index / self.width
                col_index = random_index % self.width

                self.get_logger().info('Checking Point Distance')
                #Find straightline distance between Turtlebot and current point, using pythag
                distance = math.sqrt((((row_index*resolution) + origin_x +(resolution/2)) - self.current_position.pose.position.x)**2 + (((col_index*resolution) + origin_x + (resolution/2)) - self.current_position.pose.position.y)**2)

                if min_distance < distance < max_distance:
                    self.new_waypoint.pose.position.x = (col_index * resolution) + origin_x + (resolution/2)
                    self.new_waypoint.pose.position.y = (row_index * resolution) + origin_x + (resolution/2)
                    self.get_logger().info('Point Distance:')
                    self.get_logger().info(str(distance))
                    isthisagoodwaypoint = True
                else:
                    self.get_logger().info('Too far point, Finding New...')
                    isthisagoodwaypoint = False
                
            


        self.get_logger().info('Publishing waypoint...')
        self.waypoint_publisher.publish(self.new_waypoint)
        self.ready = False

        #DEBUGGING CONDITIONAL AS ROBOT WAS GETTING STUCK DURING STARTUP, AND SELF.READY WOULD NEVER CHANGE BECAUSE THE BEHAVIORTREELOG TOPIC WAS NOT BEING PUBLISHED TO
        if self.last_node_name == 'string':
            self.ready = True

        self.waiting_counter = 0







    def frontier_check(self, occupancy_data_np, random_index):
        """
        Checks if the point we've selected is on the edge of the frontier, but isn't very close to obstacles
        """
        uncertain_indexes = 0
        obstacle_indexes = 0
        #Inspects the nature of points in a grid around the selected point
        for x in range(-4,5):
            for y in range(-4,5):
                row_index = x * self.width + y
                try:
                    if occupancy_data_np[random_index + row_index] == -1:
                        uncertain_indexes += 1
                    elif occupancy_data_np[random_index + row_index] > self.obstacle_probability:
                        obstacle_indexes += 1
                #the index of a point next to the random_index may not be within the range of occupancy_data.data, so the IndexError is handled below
                except IndexError:
                    continue
        #Code to determine how many uncertain and obstacle indices need to be near our point
        if uncertain_indexes > 2 and 0 < obstacle_indexes:
            return True
        else:
            return False
        





    def current_position_callback(self, msg:PoseWithCovarianceStamped):
        #Return current robot pose, unless searching_for_waypoint

        if self.searching_for_waypoint == False:
            self.current_position.pose.position.x = msg.pose.pose.position.x
            self.current_position.pose.position.y = msg.pose.pose.position.y








    def readiness_check(self, msg:BehaviorTreeLog):
        #If latest node state of /behavior_tree_log is "NavigateRecovery" and event status is "IDLE", send next waypoint
        for event in msg.event_log:
            if event.node_name == 'IsGoalReached' and event.current_status =='SUCCESS':
                self.ready = True
                #self.next_waypoint(occupancy_data=self.occupancy_grid)

            elif event.node_name == 'RateController' and event.current_status == 'RUNNING':
                self.ready = False
            
            elif event.node_name == 'FollowPath' and event.current_status =='SUCCESS':
                self.ready = True

            elif event.node_name == 'ComputePathToPose' and event.current_status == "FAILURE":
                self.ready = True

            elif event.node_name == 'GoalUpdated' and event.current_status == "FAILURE":
                self.ready = True
                
            else:
                #self.get_logger().info('Event Node Name:')
                #self.get_logger().info(event.node_name)
                self.last_node_name = event.node_name
                #self.get_logger().info('Event Node Status:')
                #self.get_logger().info(event.current_status)
                self.last_node_status = event.current_status
                return
            
    

   

def main():
    rclpy.init()
    autopilot_node = Autopilot()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(autopilot_node)
    executor.spin()
    autopilot_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()