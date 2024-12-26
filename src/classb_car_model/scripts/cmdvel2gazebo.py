#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

# 将速度命令转换为 Gazebo 兼容的控制命令
class CmdVel2Gazebo:
    # 类的初始化方法，构造函数，用于设置初始状态和ROS节点
    def __init__(self):
        rospy.init_node('cmdvel2gazebo', anonymous=True)            # 创建Python节点 anonymous=True 确保节点名称唯一
        # 订阅 /classb_car/cmd_vel 话题，接收 Twist 类型的消息，并在接收到消息时调用 callback 方法，queue_size=1 设置消息队列大小
        rospy.Subscriber('/classb_car/cmd_vel', Twist, self.callback, queue_size=1)         # 创建接受者，接受话题，触发callback回调函数

        # 创建发布者
        # 用于控制前左轮的转向
        self.pub_steerL = rospy.Publisher('/classb_car/front_left_steering_position_controller/command', Float64, queue_size=1)
        # 发布到前右轮的控制话题
        self.pub_steerR = rospy.Publisher('/classb_car/front_right_steering_position_controller/command', Float64, queue_size=1)
        # 发布到后左轮的速度控制话题
        self.pub_rearL = rospy.Publisher('/classb_car/rear_left_velocity_controller/command', Float64, queue_size=1)
        # 发布到后右轮的速度控制话题
        self.pub_rearR = rospy.Publisher('/classb_car/rear_right_velocity_controller/command', Float64, queue_size=1)

        # initial velocity and tire angle are 0  设置初始线速度和角速度为0
        self.x = 0
        self.z = 0

        # car Wheelbase (in m)      车的轴距
        self.L = 1.868

        # car Tread     设置前轮和后轮的轮距 self.T_front 和 self.T_rear            
        self.T_front = 1.284
        self.T_rear = 1.284 #1.386

        # how many seconds delay for the dead man's switch
        # 设置一个超时值 self.timeout 为0.2秒，并记录最后一次接收到消息的时间
        self.timeout=rospy.Duration.from_sec(0.2)
        self.lastMsg=rospy.Time.now()

        # maximum steer angle of the "inside" tire      最大转角
        self.maxsteerInside=0.6

        # turning radius for maximum steer angle just with the inside tire
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        # 求解最大转弯半径
        rMax = self.L/math.tan(self.maxsteerInside)

        # radius of inside tire is rMax, so radius of the ideal middle tire (rIdeal) is rMax+treadwidth/2
        # 计算理想中轮胎的半径 rIdeal，考虑到轮距的一半
        rIdeal = rMax+(self.T_front/2.0)

        # maximum steering angle for ideal middle tire
        # tan(angle) = wheelbase/radius
        # 计算理想中轮胎的最大转向角度 self.maxsteer，利用 atan2 计算
        self.maxsteer=math.atan2(self.L,rIdeal)

        # loop
        rate = rospy.Rate(10) # run at 10Hz  创建一个速率控制器 rate，使循环以10Hz的频率运行
        while not rospy.is_shutdown():
            self.publish()      # 在每次循环中调用 self.publish() 方法，负责将计算出的命令发布到相应的话题
            rate.sleep()        # 暂停循环以保持设定的频率
        

    def callback(self,data):
        # w = v / r
        self.x = data.linear.x / 0.3
        # constrain the ideal steering angle such that the ackermann steering is maxed out
        self.z = max(-self.maxsteer,min(self.maxsteer,data.angular.z))
        self.lastMsg = rospy.Time.now()

    def publish(self):
        # now that these values are published, we
        # reset the velocity, so that if we don't hear new
        # ones for the next timestep that we time out; note
        # that the tire angle will not change
        # NOTE: we only set self.x to be 0 after 200ms of timeout
        delta_last_msg_time = rospy.Time.now() - self.lastMsg
        msgs_too_old = delta_last_msg_time > self.timeout
        if msgs_too_old:
            self.x = 0
            msgRear = Float64()
            msgRear.data = self.x
            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)
            msgSteer = Float64()
            msgSteer.data = 0
            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)

            return

        # The self.z is the delta angle in radians of the imaginary front wheel of ackerman model.
        if self.z != 0:
            T_rear = self.T_rear
            T_front = self.T_front
            L=self.L
            # self.v is the linear *velocity*
            r = L/math.fabs(math.tan(self.z))

            rL_rear = r-(math.copysign(1,self.z)*(T_rear/2.0))
            rR_rear = r+(math.copysign(1,self.z)*(T_rear/2.0))
            rL_front = r-(math.copysign(1,self.z)*(T_front/2.0))
            rR_front = r+(math.copysign(1,self.z)*(T_front/2.0))
            msgRearR = Float64()
            # the right tire will go a little faster when we turn left (positive angle)
            # amount is proportional to the radius of the outside/ideal
            msgRearR.data = self.x*rR_rear/r
            msgRearL = Float64()
            # the left tire will go a little slower when we turn left (positive angle)
            # amount is proportional to the radius of the inside/ideal
            msgRearL.data = self.x*rL_rear/r

            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)

            msgSteerL = Float64()
            msgSteerR = Float64()
            # the left tire's angle is solved directly from geometry
            msgSteerL.data = math.atan2(L,rL_front)*math.copysign(1,self.z)
            self.pub_steerL.publish(msgSteerL)
    
            # the right tire's angle is solved directly from geometry
            msgSteerR.data = math.atan2(L,rR_front)*math.copysign(1,self.z)
            self.pub_steerR.publish(msgSteerR)
        else:
            # if we aren't turning
            msgRear = Float64()
            msgRear.data = self.x
            self.pub_rearL.publish(msgRear)
            # msgRear.data = 0;
            self.pub_rearR.publish(msgRear)

            msgSteer = Float64()
            msgSteer.data = self.z

            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)


if __name__ == '__main__':
    try:
        CmdVel2Gazebo()
    except rospy.ROSInterruptException:
        pass


