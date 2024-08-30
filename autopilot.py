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
        resolution = 0.05
        origin = occupancy_data.info.origin.position
        width = occupancy_data.info.width



        isthisagoodwaypoint = False

        while isthisagoodwaypoint == False:
            random_index = randrange(len(occupancy_data.data))
            self.potential_pos = occupancy_data.data[random_index]
            frontier_detection= self.frontier_check(occupancy_data, random_index)

            if self.potential_pos != -1 and self.potential_pos <= 20 and frontier_detection == True:
                isthisagoodwaypoint = True

###############################################################################
###########COMMENTS HERE TO EXPLAIN HOW THIS ACTUALLY WORKS, WHY HARDCODE 384? IS IT SOMETHING TO DO WITH OCCUPANCY GRID TOTAL SIZE?#####################
        row_index = random_index / width
        col_index = random_index % width

        self.new_waypoint.pose.position.x = (col_index * resolution) + origin + (resolution/2)
        self.new_waypoint.pose.position.y = (row_index * resolution) + origin + (resolution/2)

        #Publish new waypoint to '/goal_pose'
        self.waypoint_publisher.publish(self.new_waypoint)


    def frontier_check(self, occupancy_data, random_index):
        """
        Checks if the point we've selected is on the edge of the frontier, but isn't very close to obstacles
        """
        uncertain_indexes = 0
        obstacle_indexes = 0
        #Verifies points in a 6x6 grid around the selected point

        for x in range(-3,4):
            for y in range(-3,4):
                row_index = x * 384 + y
                try:
                    if occupancy_data.data[random_index + row_index] == -1:
                        uncertain_indexes += 1
                    elif occupancy_data.data[random_index + row_index] > 65:
                        obstacle_indexes += 1
                #the index of a point next to the random_index may not be within the range of occupancy_data.data, so the IndexError is handled below
                except IndexError:
                    pass
        #if the point in question (random_index) is next to two uncertain_indexes and not next to more than 3 obstacle_indexes, then this is a valid index along the frontier
        if uncertain_indexes > 1 and obstacle_indexes < 2:
            return True
        else:
            return False


    def main():
        rclpy.init(args=args)

        autopilot_node = Autopilot()

        rclpy.spin(autopilot_node)

if __name__=='__main__':
    main()