#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim_move.msg import Cen_pose
from math import pow, atan2, sqrt


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)
        self.pose_subscriber = rospy.Subscriber('center_position',
                                                Cen_pose, self.update_center_pose)


        self.goal = Cen_pose()
        self.pose = Pose()
        self.rate = rospy.Rate(15)


    def update_center_pose(self, data):
        self.goal = data
        print(self.goal)


    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    # def euclidean_distance(self, goal_pose):
    #     return sqrt(pow((goal_pose.x - self.pose.x), 2) +
    #                 pow((goal_pose.y - self.pose.y), 2))
    def euclidean_distance(self):
        return sqrt(pow((self.goal.x - self.pose.x), 2) +
                    pow((self.goal.y - self.pose.y), 2))

    # def linear_vel(self, goal_pose, constant=1.5):
    #     return constant * self.euclidean_distance(goal_pose)
    def linear_vel(self, constant=1.5):
        print("linear_vel")
        return constant * self.euclidean_distance()

    # def steering_angle(self, goal_pose):
    #     return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
    def steering_angle(self):
        print("steering angle")
        return atan2(self.goal.y - self.pose.y, self.goal.x - self.pose.x)

    def angular_vel(self, constant=6):
        print("angular vel")
        return constant * (self.steering_angle() - self.pose.theta)

    def move2goal(self):
        """Moves the turtle to the goal."""
        # goal_pose = Pose()
        #
        # # Get the input from the user.
        # goal_pose.x = input("Set your x goal: ")
        # goal_pose.y = input("Set your y goal: ")
        #self.goal = data

        distance_tolerance = 0.01

        vel_msg = Twist()

        while self.euclidean_distance() >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel()
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            # vel_msg.angular.z = self.angular_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel()

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    x = TurtleBot()
    while not rospy.is_shutdown():
        try:
            x.move2goal()
            #rospy.spin()
        except rospy.ROSInterruptException:
            pass
