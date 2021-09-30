#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from unitair.msg import Actuation

NUM_CHANNELS = 16

def actuator_sweep():
    pub = rospy.Publisher('/control/actuation', Actuation, queue_size=10)
    rospy.init_node('actuator_sweep', anonymous=True)
    rate = rospy.Rate(10) # Hz
    actuator_command = -1.0

    while not rospy.is_shutdown():
        if(actuator_command > 1.0):
            actuator_command = -1.0
        actuator_commands = [actuator_command] * NUM_CHANNELS

        actuation_msg = Actuation()
        actuation_msg.efforts = actuator_commands
        pub.publish(actuation_msg)

        actuator_command += 0.05;
        rate.sleep()

if __name__ == '__main__':
    try:
        actuator_sweep()
    except rospy.ROSInterruptException:
        pass
