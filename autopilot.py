import numpy as np
import rclpy
from random import randrange
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped



class Autopilot(Node):

    def __init__(self):
        """Initialize autopilot parameters:

        potential_pos (OccupancyGrid): Occupancy grid index associated with next potential position

        occupancy_grid (OccupancyGrid): Current occupancy grid information received from '/map'. References callback function 'next_waypoint'
        
        """
        #Initializing x and y coordinates of Turtlebot in space, to be populated later
        self.new_waypoint = PoseStamped()
        self.new_waypoint.header.frame_id = 'map'
        self.new_waypoint.pose.position.x = 0
        self.new_waypoint.pose.position.y = 0
        self.new_waypoint.pose.orientation.w = 1.0

        #Subscribe to OccupancyGrid type topic "/map"
        self.potential_pos = OccupancyGrid()
        self.occupancy_grid = self.create_subscription(OccupancyGrid, '/map')
        
        #Subscribe to /behavior_tree_log to determine when Turtlebot is ready for a new waypoint
        self.behaviortreelogstate = self.create_subscription(BehaviorTreeLog, 'behavior_tree_log', self.readiness_check, 10)

        #Create publisher to publish next waypoint parameters to
        self.waypoint_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        #Track number of waypoints sent
        self.waypoint_counter = 0


    def readiness_check(self, msg:BehaviorTreeLog):
        #If latest node state of /behavior_tree_log is "NavigateRecovery" and event status is "IDLE", send next waypoint

        for event in msg.event_log:
            if event.node.name == 'NavigateRecovery' and event.current_status =='IDLE':
                self.next_waypoint(occupancy_data=self.occupancy_grid)

        
    def next_waypoint(self, occupancy_data):
        """Callback function to choose next waypoint when new occupancy grid is received, and old goal is either destroyed or achieved

        Args:
        self (Node): Autopilot node currently running and storing waypoint decisions 
        occupancy_data (OccupancyGrid): map data array from OccupancyGrid type

        """
        isthisagoodwaypoint = False

        while isthisagoodwaypoint == False:
            random_index = randrange(len(occupancy_data.data))
            self.potential_pos = occupancy_data.data[random_index]
            frontier_detection= self.frontier_check(occupancy_data, random_index)

            if self.potential_pos != -1 and self.potential_pos <= 0.2 and frontier_detection == True:
                isthisagoodwaypoint = True

###############################################################################
###########COMMENTS HERE TO EXPLAIN HOW THIS ACTUALLY WORKS, WHY HARDCODE 384? IS IT SOMETHING TO DO WITH OCCUPANCY GRID TOTAL SIZE?#####################
        row_index = random_index / 384
        col_index = random_index % 384

        self.new_waypoint.pose.position.x = col_index * 0.05 - 10 # column * resolution + origin_x
        self.new_waypoint.pose.position.y = row_index * 0.05 -10 # row * resolution + origin_x

        #Publish new waypoint to '/goal_pose'
        self.waypoint_publisher.publish(self.new_waypoint)


    def frontier_check(self, occupancy_data, random_index):
        """
        Checks if the point we've selected is on the edge of the frontier, but isn't very close to obstacles
        """
        uncertain_indexes = 0
        obstacle_indexes = 0
        ###################################I'M ASSUMING THE 6X6 GRID IS THE AMOUNT OF SPACE TAKEN UP BY THE TURTLEBOT3

        for x in range(-3,4):
            for y in range(-3,4):
                row_index = x * 384 + y
                

# Hello Luigi


    def main():
        rclpy.init(args=args)

        autopilot_node = Autopilot()

        rclpy.spin(autopilot_node)

if __name__=='__main__':
    main()