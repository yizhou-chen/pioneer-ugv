# pioneer-ugv
for single pioneer

    
    #!/usr/bin/env python
    import rospy
    import math
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import PoseStamped
    PI = 3.1415926535897
    class UGVControl:
        def __init__(self):
            rospy.init_node('coordinates_listener', anonymous=True)
            self.rate = rospy.Rate(30)
            self.pub_vel = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
            self.sub_pose = rospy.Subscriber('vrpn_client_node/leo/pose', PoseStamped,self.callback)
            self.position_msg = PoseStamped()

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
            #print(current_positon_x)
            # print(self.position_msg)

            # Get the yaw from quaternions
            q_x = self.position_msg.pose.orientation.x
            q_y = self.position_msg.pose.orientation.y
            q_z = self.position_msg.pose.orientation.z
            q_w = self.position_msg.pose.orientation.w
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

            print('yaw=%f'%(yaw*180/PI))
            # Define a reference trajectory
            #circle
            # ref_x = 1 * math.cos(t)
            # ref_y = 1 * math.sin(t)

            # x_dot = -1 * math.sin(t)
            # y_dot = 1 * math.cos(t)
            # x_2dot = -1 * math.cos(t)
            # y_2dot = -1 * math.sin(t)

            #straightline
            # ref_x = 0.01*t
            # ref_y = 0.02*t

            # x_dot = 0.01
            # y_dot = 0.02
            # x_2dot = 0
            # y_2dot = 0

            #sin
            # ref_x = 0.1*t
            # ref_y = 0.1*math.sin(ref_x)
            # x_dot = 0.1
            # y_dot = 0.1*math.cos(ref_x)
            # x_2dot = 0
            # y_2dot = -0.01*math.sin(ref_x)

            #'8'
            L=0.5
            mu=1
            ref_x = L*math.cos(mu*t)
            ref_y = 2*L*math.sin(mu*t)*math.cos(mu*t)
            x_dot = -mu*L*math.sin(mu*t)
            y_dot = 2*mu*L*(math.cos(mu*t)*math.cos(mu*t)-math.sin(mu*t)*math.sin(mu*t))
            x_2dot = -math.pow(mu,2)*L*math.cos(mu*t)
            y_2dot = -8*L*math.pow(mu,2)*math.sin(mu*t)*math.cos(mu*t)
            # print('ref_x=%f'%(ref_x))
            # print('ref_y=%f'%(ref_y))
            # print('t=%f'%(t))

            ref_w = (x_dot*y_2dot-x_2dot*y_dot)/(math.pow(x_dot,2)+math.pow(y_dot,2))
            ref_v = math.sqrt(math.pow(x_dot,2)+math.pow(y_dot,2))

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
            k1 = 0.04
            k2 = 1.0
            k3 = 0.3

            # Input control rule
            v = ref_v * math.cos(error_theta) + k1 * error_x
            w = ref_w + k2 * ref_v * error_y * (math.sin(error_theta)/error_theta) + k3 * error_theta
            print('w=',w)
            print('v=',v)
            print('ref=',ref_w)
            # c1 = 0.5
            # c2 = 10
            # c3 = 5


            # v = ref_v + (c1 * error_x / math.sqrt(1 + math.pow(error_x,2) + math.pow(error_y,2)))
            # w = ref_w + (c2 * ref_v * (error_y*math.cos(error_theta/2)-error_x*math.sin(error_theta/2))) / (math.sqrt(1+math.pow(error_x,2)+math.pow(error_y,2))) + c3 * math.sin(error_theta/2)
            # Publish the topic

            vel_msg = Twist()
            vel_msg.linear.x = v
            vel_msg.angular.z = w
            return vel_msg

    if __name__ == '__main__':
        my_ugv = UGVControl()
        my_ugv.run()

