import rclpy
import math
import numpy as np
import sys
    import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class TBug(Node):
    def __init__(self, args):
        super().__init__('TBug_controller')
        
        # initialization Variable
        self.goal_coordinates = [float(args[0]),float(args[1])]
        self.pos = [] #from callback gps
        self.goal = []
        self.dmin = np.inf
        self.lidar_data=[]
        self.obstacle_angle = [] #FIX THIS
        self.obstacle_points = [] #each index has (angle, distance)
        self.obstacle_coords = [] #list, in global frame  x,y,angle
        self.odom = [0,0,0] #x,y,yaw
        self.w = 0
        self.v = 0 
        self.best_tangent_point=None

        #Subscription
        self.lidar_subscription = self.create_subscription(LaserScan, '/bcr_bot/scan', self.lidar_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry,'/bcr_bot/odom',self.odom_callback,10)

        #publisher
        self.vel_publisher = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)

        #threshold Variable
        self.angle_threshold = 90
        self.obstacle_distance_threshold = 4
        self.cluster_threshold = 1.2 #maximum distance between two consecutie obstracle to be consider as 1
        self.ray_threshold = [0.5,1.5] #threshold for how much space should be there from the los ray to pass the rover, threshold to compensate the parallel distance measurement inacuracy
        self.rotation_threshold = 2*math.pi/180 # 5 degrees
        self.rotation_Kp = 1.5
        self.max_rotation_w = 4
        self.drive_threshold = 1.0
        self.drive_Kp = 1.5
        self.max_drive_v = 4

        # Flag Variable
        self.goalSeeking = True
        self.wallFollowing = False
        self.scanning = True # turn of lidar when in wall following and going to a boundary point in scanning it should be True, if False dont take lidar's data 
        self.rotation_complete = False
        self.position_reached = False
        self.drive_start = False

        #parameter Variiable

        self.timer = self.create_timer(0.1, self.run)


## CALLBACKS
    def lidar_callback(self,msg):
        # this filters lidar message by angle 
        self.lidar_raw_data = msg.ranges
        self.lidar_raw_data.pop(45) # hack: removing faulty beam
        posetive_part = self.lidar_raw_data[:(self.angle_threshold)]
        negative_part = self.lidar_raw_data[-(self.angle_threshold+1):]
        negative_part.pop(-1) 
        self.lidar_data = negative_part + posetive_part
        # print (self.lidar_data) 
        if self.scanning:
            self.get_obstacle()

    def odom_callback(self,msg):
        self.odom[0] = msg.pose.pose.position.x
        self.odom[1] = msg.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.odom[2] = yaw

#helper functionws
    def distance(self, c1, c2):
        return math.sqrt(sum((a-b)**2 for a, b in zip(c1, c2)))
    
    def third_side(self, a, b, C):
        return np.sqrt(a*a + b*b - 2*a*b*np.cos(C))
  
    def get_obstacle(self):
        # this takes filtered lidar message and filters obstacle points
        print("getting obstacles now")
        if(self.scanning==False):
          return
        prev_x = None
        prev_y = None
        c = 0
        self.obstacle_clusters = {}
        self.obstacle_coords = []
        for i, distance in enumerate(self.lidar_data):
            if abs(distance) < self.obstacle_distance_threshold:
                angle = i
                #clustering
                dy = distance*(math.cos(np.deg2rad(angle)))
                dx = distance*(math.sin(np.deg2rad(angle)))
                x = dx + self.odom[0]
                y = (-1)*dy + self.odom[1]
                if prev_x is not None and prev_y is not None:
                    if (self.distance([x,y],[prev_x,prev_y])>self.cluster_threshold):
                        c += 1
                self.obstacle_coords.append([x, y, np.deg2rad(angle)])
                if c not in self.obstacle_clusters:
                    self.obstacle_clusters[c] = []
                self.obstacle_clusters[c].append((x, y))
                prev_x = x
                prev_y = y
        for c, points in self.obstacle_clusters.items():
            x0, y0 = points[0]
            points[0] = (x0, y0)   # replace tuple

            x1, y1 = points[-1]
            points[-1] = (x1, y1)  # replace tuple
        # print(self.obstacle_clusters)
    
    def get_tangent_points(self):
        tangent_points = []
        for c, points in self.obstacle_clusters.items(): #.items() gives us (c, points) pairs
            # x1, y1= points[0]
            # points[0] = [x1 , y1 + 0.2]
            # x2, y2 = points[-1]
            # points[-1] = [x2 , y2 - 0.2]
            left_tangent = points[-1]
            right_tangent = points[0]
            tangent_points.append((c, left_tangent, right_tangent))

            ''' output would look like this
            [
            (c1, (x1, y1), (x2, y2))
            (c2, (x3, y3), (x4, y4))
            ]
            used tuples inside because they wont change, but tangent_points is list because it can grow
            '''

        print(f"this is the tangent = {tangent_points}")
        return tangent_points

    def check_line_of_sight(self):
        a = [self.odom[0],self.odom[1]]
        b = self.goal_coordinates
        theta = math.atan2(b[1] - a[1], b[0] - a[0])
        obstacles = np.round(np.array(self.obstacle_coords, dtype=np.float32).reshape(-1, 3), 3)
        ox, oy, otheta = obstacles[:,0], obstacles[:,1], obstacles[:,2]
        otheta = otheta - (math.pi/2)
        dx, dy = ox - a[0], oy - a[1]
        r = np.sqrt(dx**2 + dy**2)

        d_perpendicular = r * np.sin(otheta - theta)
        d_parallel = r * np.cos(otheta - theta)

        if np.all(np.abs(d_perpendicular) > self.ray_threshold[0]) and np.all(np.abs(d_parallel) < (self.distance(a,b)+self.ray_threshold[1])):
            print("Los not blocked")
            return True
        else:
            print("Los blocked")
            return False

#logic functions
    def goal_seeking(self):
        print("GOAL SEEKING MODE")

        self.pos  = [self.odom[0], self.odom[1]]
        self.scanning = True
        if self.check_line_of_sight():
            if not self.drive_start: 
                self.rotation_complete = False
            self.go_to_position(self.goal_coordinates)
            self.goalSeeking = True
            self.wallFollowing = False
        else:
            self.drive_start = False
            self.rotation_complete = False #doubtful
            self.goalSeeking = False
            self.wallFollowing = True
            self.dmin = self.distance(self.pos, self.goal_coordinates)#updating dmin as the distance between current pos and final goal coords 

    def wall_following(self):    
        print("WALL FOLLWING MODE")

        current= self.distance([self.odom[0], self.odom[1]], self.goal_coordinates)

        if self.check_line_of_sight() and current < self.dmin:
            self.goalSeeking = True
            self.wallFollowing = False
            print("leaving wall following mode")
            return

        # update dmin after checking LOS not before
        if current<self.dmin:
            self.dmin = current
         
        #now get tangent points and move 
        tangent_points = self.get_tangent_points()
        
        if len(tangent_points) == 0:
            # self.scan_360()
            print("no tangent points")
            self.goalSeeking = True
            self.wallFollowing = False
            return

        best_distance=float('inf')  #assigning infinity value 
        for i, left, right in tangent_points:   
             for pt, side in [(left, True), (right, False)]:
                cost = self.distance(self.pos, pt) + self.distance(pt, self.goal_coordinates) #fixed this from self.goal to self.goal_coordinates
                if cost < best_distance:
                    best_distance  = cost
                    self.best_tangent_point = pt
                    best_side = side

        if self.best_tangent_point is not None:
            self.scanning = False #let it reach the tangent point then we will again scan
            padded_target = self.push_tangent_perpendicular(self.pos, self.best_tangent_point, best_side, offset=2) # moving the targets a bit
            print(f" this is the padded tangent = {padded_target}")
            self.go_to_position(padded_target) #one possible issue = we are not checking LOS to tangent point here
            self.scanning = False
            return 



    #chatgpt fix        
    def push_tangent_perpendicular(self, robot_pos, tangent_point, left, offset):
        """
        Offset tangent point perpendicular to line (robot -> tangent).
        If left=True, push to the left side; else push to the right.
        """
        rx, ry = robot_pos
        tx, ty = tangent_point

        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)

        if dist == 0:
            return (tx, ty)

        if left:
            # Left perpendicular
            nx, ny = -dy / dist, dx / dist
        else:
            # Right perpendicular
            nx, ny = dy / dist, -dx / dist

        pushed_x = tx + offset * nx
        pushed_y = ty + offset * ny

        return (pushed_x, pushed_y)


    def go_to_position(self, position):
        
        if not self.rotation_complete:
            req_yaw = math.atan2((position[1] - self.odom[1]),(position[0] - self.odom[0]))
            self.rotate(req_yaw)
            print(f"roation complete? = {self.rotation_complete}")
            

        # if not self.check_line_of_sight(position):
        #     self.goalSeeking = False
        #     self.wallFollowing = True
        #     self.rotation_complete = False
        #     return

        # to add above snippet we need to modif LOS to take any target
        # this is needed because while driving towards goal we are not checking LOS 
        if self.rotation_complete:
            print("started_driveing")
            self.drive(position)

    def drive(self,position):
        error = self.distance([self.odom[0],self.odom[1]],position)
        if error < self.drive_threshold:
            self.get_logger().warn(f'drive done :: Error = {error:.3f} m')
            self.v = 0.0
            self.goalSeeking = True
            self.wallFollowing = False
            self.drive_start = False
            #self.rotation_complete = False # clearing old flags | we cant clear this otherwise it reaches tangent and again tries to rotate and gets stuck 
            self.best_tangent_point = None #clearing old data
            self.scanning = True # allow re scan after reaching tangent (although i added in wall following also but here it makes more sense)
            if position == self.goal_coordinates:
                self.position_reached = True
                print("goal reached")
            else:
                self.scanning = True
                req_yaw = req_yaw = math.atan2((self.goal_coordinates[1] - self.odom[1]),(self.goal_coordinates[0] - self.odom[0]))
                self.rotate(req_yaw)
                
        else:
            self.get_logger().info(f"Error :: {error:.3f} m")
            self.v = self.drive_Kp*error
            self.v = np.clip(self.v,0,self.max_drive_v)
        msg = Twist ()
        msg.linear.x = self.v
        self.vel_publisher.publish(msg)

    def rotate(self,req_yaw):
        error = req_yaw - self.odom[2]
        
        if abs(error) < self.rotation_threshold:
            self.get_logger().warn(f'rotation done :: Error = {(abs(error))*180/(math.pi):.3f} degrees')
            self.w = 0.0
            self.rotation_complete = True
            self.drive_start = True
        else:
            self.get_logger().info(f"Error :: {(abs(error))*180/(math.pi):.3f} degrees")
            self.w = self.rotation_Kp*error
            self.w = np.clip(self.w,-1*(self.max_rotation_w),self.max_rotation_w)
        msg = Twist ()
        msg.linear.x = 0.0
        msg.angular.z = self.w
        self.vel_publisher.publish(msg) 

    # def scan_360(self, t):
    #     self.scanning = True
    #     v= Twist()
    #     v.angular.z = 2.0
    #     T = 2 * math.pi / v.angular.z 
    #     self.vel_publisher.publish(v)
    #     print("360 ROTATING ")
                

    def run(self):
        if self.goalSeeking:
            self.goal_seeking()

        elif self.wallFollowing:
            self.wall_following()

        
def main(args=None):
    rclpy.init(args=args)

    cli_args = sys.argv[1:]  # sys.argv[0] is the script itself
    node=TBug(cli_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  































        #ros2 launch bcr_bot ign.launch.py camera_enabled:=True stereo_camera_enabled:=True two_d_lidar_enabled:=True position_x:=0.0 position_y:=0.0 orientation_yaw:=0.0 odometry_source:=world world_file:=warehouse.sdf