#!/usr/bin/env python
import roslib
roslib.load_manifest('demo_greent')
import rospy
import tf
from geometry_msgs.msg import Twist
from sys import argv
from time import sleep


BASE_FRAME = '/openni_depth_frame'
FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]
LAST = rospy.Duration()


class Kinect:

    def __init__(self, name='greenT_kinect', user=1):
        try:
            if len(argv) == 1:
                print("Going with default user...")
                self.user = user
            elif len(argv) > 1 and not isinstance(argv[1],int):
                raise TypeError("Argument introduzed is not an Integer...\nExiting")
            else:
                print("User set to %d" % int(argv[1]))
                self.user = int(argv[1])
        except TypeError as error:
            print(error)
            exit(1)
        rospy.init_node(name, anonymous=True)
        self.listener = tf.TransformListener()
        self.rate = rospy.get_param('~rate',30) #publica a 30Hz
        self.lastcoords = None
        self.flag = True
        self.inity = 0.0
        self.twist = Twist()
        global pub
        pub = rospy.Publisher('twist', Twist, queue_size=10)

    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            
            self.get_posture()

            if self.user > 5:
                self.user = 1
            r.sleep()
    
    def get_posture(self):
        try:
            for frame in FRAMES:
                self.listener.waitForTransform(BASE_FRAME, "/%s_%d" % (frame, self.user), rospy.Time(), rospy.Duration(4.0))
                (trans, rot) = self.listener.lookupTransform(BASE_FRAME,"/%s_%d" % (frame, self.user), LAST)
                
                if self.flag:
                    self.inity = float(trans[1])
                    self.flag = False

                if frame is "torso":
                    #print(frame, "X: %s" % (trans[0]))
                    try:
                        self.calculate_twist(trans[0],trans[1])
                    except Exception as error:
                        print(error)
                        pass
                    self.lastcoords = [trans[0],trans[1]]
                
            pub.publish(self.twist)
            
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            raise IndexError
    
    def check_lateral_movement(self,cy):
        """
        Check if there was lateral movement according to the initial Y position of the user.\n
        :param cy: User's current Y coordinate\n
        :return: True + [Direction, Speed] if there was movement\n
        :return: False + [Stop,0] if there was no movement
        """
        interval = 0.04
        if cy > self.inity:
            return True,["left",int(round(self.valmap(cy, 0.0, 2.0, 0, 204)))]
        elif cy < self.inity:
            return True, ["right",int(round(self.valmap(cy, -2.0, 0.0, -204, 0)))]
        elif (cy <= (self.inity+interval)) and (cy >= (self.inity-interval)):
            return True, ["stop",0]
        else:
            return False, ["ERROR",0,""]
    
    def calculate_twist(self, cx, cy):
        """
        Calculate Twist for both Linear movement and Angular movement\n
        :param cx: Current X coordinate\n
        :param cy: Current Y coordinate\n
        :return: Only returns if there's no last coordinates 
        """
        direction = None
        if self.lastcoords is None: # if no last coordinates then exit function
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            raise ArithmeticError("No last coordinates")

        if self.lastcoords[0] == cx:
            self.twist.linear.x = 0
        else:
            self.twist.linear.x = int(round(self.valmap(cx, 0.0, 2.0, 0, 150))) # map wheel target speed according to the delta recieved
            
        interval = 0.06
        if cx < 0.6: # create a safety distance for no collision with user
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            

        lat =  self.check_lateral_movement(cy)

        if not lat[0]:
            print(lat[1][0], "Cause:", lat[1][2])
            self.twist.angular.z = lat[1][1]
            sleep(1)
        else:
            direction = lat[1][0]
            if lat[1][0] is "left":
                self.twist.angular.z = lat[1][1]
            elif lat[1][0] is "right":
                self.twist.angular.z = lat[1][1]
            elif lat[1][0] is "stop":
                self.twist.angular.z = lat[1][1]
            else:
                raise Exception("Unknown error occured")

        print("-----------------------------------------------------------------------")
        print("X:",cx,"Y:",cy)
        print("linear:",self.twist.linear.x,"Direction:",direction,"Angular:", self.twist.angular.z)

    def valmap(self, x, in_min, in_max, out_min, out_max):
        "Map values like Arduino map function"
        return float((x - in_min) / (in_max-in_min) * (out_max-out_min)+out_min)
        
if __name__ == '__main__':
    try:
        kin = Kinect()
        kin.spin()
    except KeyboardInterrupt:
        kin.twist.linear.x = 0
        kin.twist.angular.z = 0
        print("Keyboard Interrupt | Linear:", kin.twist.linear.x,"Angular:",kin.twist.angular.z)
