import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import numpy as np


class TBug(Node):
    def __init__(self):
        super().__init__('TBug_controller')
        self.goalSeeking = True
        self.wallFollowing = False
        self.pos = [] #from callback gps
        self.goal = []
        self.obstacle_points = [] #get lidar message 
        self.scanning = True # turn of lidar when in wall following and going to a boundary point in scanning it should be True, if False dont take lidar's data 
        self.dmin = np.inf

    def distance(self, c1, c2):
        return math.sqrt(sum((a-b)**2 for a, b in zip(c1, c2)))
    
    def goal_seeking(self):
        speed = Twist()
        self.scanning = True
        self.go_to_position(self.goal)

        if len(self.obstacle_points) == 0:
            self.goalSeeking = True
            self.wallFollowing = False

        else:
            self.goalSeeking = False
            self.wallFollowing = True


    def wall_following(self):

        tangent_points = self.get_tangent_points() #take points from lidar list of 2 point
        self.scanning = False 

        if self.check_line_of_sight():
           if self.distance(self.pos,self.goal)<self.dmin:
                self.goalSeeking = True
                self.wallFollowing = False
                return

        heuristic0 = self.distance(self.pos, tangent_points[0]) + self.distance(tangent_points[0], self.goal)
        heuristic1 = self.distance(self.pos, tangent_points[1]) + self.distance(tangent_points[1], self.goal)

        if heuristic0 < heuristic1:
            temp_dmin = self.distance(tangent_points[0], self.goal)

            if temp_dmin < self.dmin:
                self.dmin = temp_dmin

            self.go_to_position(tangent_points[0])
            self.scanning = True

            
        else:
            temp_dmin = self.distance(tangent_points[1], self.goal)

            if temp_dmin < self.dmin:
                self.dmin = temp_dmin

            self.go_to_position(tangent_points[1])
            self.scanning = True


        

    def get_tangent_points(self):

    def check_line_of_sight(self):
        #check in the constraint of 15 degree angle something 
        #return True or False
    
    def go_to_position(self, position):
        #add pure pursuit or something 

        speed = Twist()
        speed.linear.x
        

    def main(self):

        while self.goalSeeking:
            self.goal_seeking()

        while self.wallFollowing:
            self.wall_following()