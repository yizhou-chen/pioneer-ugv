# pioneer-ugv

#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
class UGVControl:
    def __init__(self):
        rospy.init_node('coordinates_listener', anonymous=True)
        self.rate = rospy.Rate(30)
        self.pub_vel = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
        self.sub_pose = rospy.Subscriber('vrpn_client_node/leo/pose', PoseStamped,self.callback)

    def run(self):
        
        t1 = rospy.get_time()
        while not rospy.is_shutdown():
            t2 = rospy.get_time()
            t = t2 - t1
            vel_msg = self.get_msg_v_cmd(t)
            self.pub_vel.publish(vel_msg)
        self.rate.sleep()

    def callback(self, msg):
        self.position_msg = msg

    def get_msg_v_cmd(self,t):  
        current_positon_x = self.position_msg.pose.position.x
        current_positon_y = self.position_msg.pose.position.y
        print(current_positon_x)
        print(self.position_msg)

        # Get the yaw from quaternions
        q_x = self.position_msg.pose.orientation.x
        q_y = self.position_msg.pose.orientation.y
        q_z = self.position_msg.pose.orientation.z
        q_w = self.position_msg.pose.orientation.w
        #t1 = +2.0 * (qua_w * qua_z + qua_x * qua_y)
        #t2 = +1.0 - 2.0 * (qua_y * qua_y + qua_z * qua_z)
        t0 = +2.0 * (q_w * q_x + q_y * q_z)
        t1 = +1.0 - 2.0 * (q_x * q_x + q_y * q_y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (q_w * q_y - q_z * q_x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (q_w * q_z + q_x * q_y)
        t4 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(t3, t4)
        # Define a reference trajectory
        ref_x = math.sin(t)
        ref_y = math.cos(t)

        x_dot = math.cos(t)
        y_dot = -math.sin(t)

        ref_w = 1
        ref_v = 1
        ref_theta = math.atan2(y_dot,x_dot)

        x_error = ref_x - current_positon_x
        y_error = ref_y - current_positon_y
        theta_error = ref_theta - yaw

        error_x = (ref_x - current_positon_x) * math.cos(yaw) + (ref_y - current_positon_y) * math.sin(yaw)
        error_y = -(ref_x - current_positon_x) * math.sin(yaw) +(ref_y -current_positon_y) * math.cos(yaw)
        error_theta = theta_error

        print('e_x=%f'%(x_error))
        print('e_y=%f'%(y_error))
        print('e_theta=%f'%(theta_error))
        c1 = 100
        c2 = 0.5
        c3 = 0.4

        # Input control rule
        w = ref_w + c1 * ref_v * error_y * (math.sin(error_theta)/error_theta) + c2 * error_theta
        v = ref_v * math.cos(error_theta) + c3 * error_x
        #w = ref_w + c2 * ref_v * (error_y*math.cos(error_theta/2)-error_x*math.sin(error_theta/2)) / math.sqrt(1+math.pow(error_x,2)+math.pow(error_y,2)) + c3 * math.sin(error_theta/2)
        #v = ref_v + c1 * error_x / math.sqrt(1 + math.pow(error_x,2) + math.pow(error_y,2))
        # Publish the topic
        
        vel_msg = Twist()
        vel_msg.linear.x = 0.0 #abs(v)
        vel_msg.angular.z = 0.0#abs(w)
        return vel_msg

if __name__ == '__main__':
    my_ugv = UGVControl()
    my_ugv.run()
