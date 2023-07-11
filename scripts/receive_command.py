#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess


class ReceiveCommand:
    def __init__(self):
        # Allow up to one second to connection
        rospy.sleep(1)

        rospy.Subscriber('/run_command', String, self.process_command)
    
    def process_command(self, data):
        
        rospy.loginfo('I received %s' %data.data)

        if data.data == "hello":
            # Run your face recognition attendance code
            rc = subprocess.call(['rosrun','facebot_package','my_facerec.py','--test'])

if __name__ == '__main__':
    try:
        # Initialize
        rospy.init_node('facebot_subscriber_node', anonymous=True)
        ReceiveCommand()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

