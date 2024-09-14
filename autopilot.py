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
from geometry_msgs.msg import PointStamped



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
        self.obstacle_probability = 60

        self.start=True

        #Initializing x and y coordinates of Turtlebot in space, to be populated later
        self.new_waypoint = PoseStamped()
        self.new_waypoint.header.frame_id = 'map'
        self.new_waypoint.pose.position.x = 0.0
        self.new_waypoint.pose.position.y = 0.0
        self.new_waypoint.pose.orientation.w = 1.0

        #Initializing potential_coordinate for debugging
        self.potential_coordinate = PointStamped()
        self.potential_coordinate.header.frame_id = 'map'
        self.potential_coordinate.point.x = 0.0
        self.potential_coordinate.point.y = 0.0

        #Initializing current position variable
        self.current_position = PointStamped()
        self.current_position.header.frame_id = 'map'

        #Initializing old waypoint variable
        self.old_point = PointStamped()
        self.old_point.header.frame_id = 'map'
        self.old_point.point.x = float('inf')
        self.old_point.point.y = float('inf')


        #Subscribe to /behavior_tree_log to determine when Turtlebot is ready for a new waypoint
        self.behaviortreelogstate = self.create_subscription(BehaviorTreeLog, 'behavior_tree_log', self.readiness_check, 10, callback_group=self.parallel_callback_group)

        #Subscribe to OccupancyGrid type topic "/map"
        self.potential_pos = OccupancyGrid()
        self.occupancy_grid = self.create_subscription(OccupancyGrid, '/map', self.store_grid, 10)

        #Subscribe to /pose to determine position of Turtlebot
        self.position_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.current_position_callback, 10, callback_group=self.parallel_callback_group)

        
        #Create publisher to publish next waypoint parameters to
        self.waypoint_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        #Publisher for publishing potential waypoints to for bug fixing
        self.potential_publisher = self.create_publisher(PointStamped, 'potential_point', 10)

        #Publisher for publishing current coordinate for bug fixing
        self.current_publisher = self.create_publisher(PointStamped, 'current_point', 10)


        #Track number of waypoints  sent
        self.waypoint_counter = 0.0


    def store_grid(self,grid:OccupancyGrid):
        """ 
            Callback function of /map topic.
            Everytime it receive te OccupacyGrid message it stores it. 
            At the start it launches the next_point() method since no message is received from the behavior_tree_log.
        """
        self.current_grid=grid
        self.get_logger().info('Grid Received')

        if self.start:

            self.start=False
            self.next_waypoint()
            
            

    def next_waypoint(self):
        """
        Function to choose next waypoint when new occupancy grid is received, and old goal is either destroyed or achieved

        Args:
        self (Node): Autopilot node currently running and storing waypoint decisions 
        """

        resolution = 0.05
        origin_x = self.current_grid.info.origin.position.x
        origin_y = self.current_grid.info.origin.position.y
        self.width = self.current_grid.info.width
        isthisagoodwaypoint = False

        #TODO: Tune these values 
        min_distance = 1
        max_distance = 5

        self.still_looking = False
        occupancy_data_np = np.array(self.current_grid.data)
        occupancy_data_np_checked = []
    
        while isthisagoodwaypoint == False:

            #Added this so that the terminal isn't filled with messages so it's easier to read
            if self.still_looking == False:
                self.get_logger().info('Searching for good point...')
                self.still_looking = True

            # Taking a random cell 
            random_index = randrange(occupancy_data_np.size)
            if random_index in occupancy_data_np_checked:
                continue

            # Check that the cell is not unknown 
            self.potential_pos = occupancy_data_np[random_index]
            if self.potential_pos==-1:
                continue

            # Check that the cell is not an obstacle
            if self.potential_pos>= self.obstacle_probability:
                self.get_logger().info('Point was an obstacle')
                occupancy_data_np_checked = np.append(occupancy_data_np_checked, random_index)
                continue
            
            # Check that the point is on the frontier
            frontier_detection = self.frontier_check(occupancy_data_np, random_index)
            if frontier_detection==False:
                self.get_logger().info('Point was not on frontier')
                occupancy_data_np_checked = np.append(occupancy_data_np_checked, random_index)
                continue

            #CRITERIA FOR A 'GOOD' WAYPOINT  
            if  self.potential_pos <= 20 :
                self.get_logger().info('Found Good Point')

                # Compute correspondent row and column of the potential cell
                slider_float = random_index / self.width
                slider = math.ceil(slider_float)
                col_index = random_index % self.width

                # Compute position with respect map frame of the potential cell
                x_coord = (col_index*resolution) + origin_x
                y_coord = (slider*resolution) + origin_y

                self.potential_coordinate.point.x = x_coord
                self.potential_coordinate.point.y = y_coord

                self.potential_publisher.publish(self.potential_coordinate)

                self.current_publisher.publish(self.current_position)

                self.get_logger().info('Checking Point Distance')
                distance2new = math.sqrt((x_coord - self.current_position.point.x)**2 + (y_coord - self.current_position.point.y)**2)


                if min_distance < distance2new < max_distance:
                    self.new_waypoint.pose.position.x = x_coord
                    self.new_waypoint.pose.position.y = y_coord
                    self.get_logger().info('Point Distance:')
                    self.get_logger().info(str(distance2new))
                    isthisagoodwaypoint = True

                else:
                    self.get_logger().info('Point not in range, Finding New...')
                    isthisagoodwaypoint = False
                    self.still_looking = False

        self.get_logger().info('Publishing waypoint...')
        self.waypoint_publisher.publish(self.new_waypoint)

        #Put a box arond the published point in the array of the already checked points
        occupancy_data_np_checked=self.box_checked(occupancy_data_np_checked,random_index) 

        #Storing waypoint for comparison during next loop        
        self.old_point.point.x = self.new_waypoint.pose.position.x
        self.old_point.point.y = self.new_waypoint.pose.position.y


    def frontier_check(self, occupancy_data_np, random_index):
        """
        Checks if the point we've selected is on the edge of the frontier, but isn't very close to obstacles
        """
        uncertain_indexes = 0
        obstacle_indexes = 0

        #Inspects the nature of points in a grid around the selected point
        #TODO: Tune these values 
        for x in range(-4,5):
            for y in range(-4,5):
                slider = x * self.width + y
                try:
                    if occupancy_data_np[random_index + slider] == -1:
                        uncertain_indexes += 1
                    elif occupancy_data_np[random_index + slider] > self.obstacle_probability:
                        obstacle_indexes += 1
                #the index of a point next to the random_index may not be within the range of occupancy_data.data, so the IndexError is handled below
                except IndexError:
                    continue
        #Code to determine how many uncertain and obstacle indices need to be near our point
        if uncertain_indexes > 1 and 0 < obstacle_indexes:
            return True
        else:
            return False
        

    def box_checked(self, occupancy_data_np_checked,new_waypoint_index):
        """ The indexes of a box of 1.5m^2 around the new waypoint are added to the occupancy_data_np_checked,"""

        for x in range(-1,1):
            for y in range(-1,1):
                slider= x * self.width + y
                occupancy_data_np_checked = np.append(occupancy_data_np_checked, new_waypoint_index+slider)

        return occupancy_data_np_checked



    def current_position_callback(self, msg:PoseWithCovarianceStamped):
        #Return current robot pose, unless searching_for_waypoint
        self.current_position.point.x = msg.pose.pose.position.x
        self.current_position.point.y = msg.pose.pose.position.y
        self.current_position.header.frame_id = msg.header.frame_id
        #if self.searching_for_waypoint == False:
            




    def readiness_check(self, msg:BehaviorTreeLog):
        """
        Call the next_waypoint() when specific conditions of the BehaviorTreeLog message are satisfied
        """

        for event in msg.event_log:

            if event.node_name == 'NavigationRecovery' and event.current_status =='IDLE':
                #self.ready = True
                self.next_waypoint()

            elif event.node_name == 'NavigateRecovery' and event.current_status =='IDLE':
                #self.ready = True
                self.next_waypoint()

            elif event.node_name == 'GoalUpdated' and event.current_status == "FAILURE":
                self.next_waypoint()
        '''
            if event.node_name == 'IsGoalReached' and event.current_status =='SUCCESS':
                self.ready = True
                #self.next_waypoint(occupancy_data=self.occupancy_grid)

            #elif event.node_name == 'RateController' and event.current_status == 'RUNNING':
            #    self.ready = False
            
            elif event.node_name == 'FollowPath' and event.current_status =='SUCCESS':
                self.ready = True

            elif event.node_name == 'ComputePathToPose' and event.current_status == "FAILURE":
                self.ready = True

            
            else:
                #self.get_logger().info('Event Node Name:')
                #self.get_logger().info(event.node_name)
                self.last_node_name = event.node_name
                #self.get_logger().info('Event Node Status:')
                #self.get_logger().info(event.current_status)
                self.last_node_status = event.current_status
                return
                '''
            
    

   

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