#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr

def send_command():

    rospy.init_node('facebot_publisher_node', anonymous=True)
    pub = rospy.Publisher('/run_command', String, queue_size=10)
    
    while not rospy.is_shutdown():
        # Obtain audio from the microphone
        r = sr.Recognizer()

        with sr.Microphone() as source:
            print(">>> Say something!")
            audio = r.record(source, duration=3)

        # Recognize speech using Google Speech Recognition
        result = ''
        try:
            result = r.recognize_google(audio)
            print("Speech Recognition result: " + result)
        except sr.UnknownValueError:
            print("Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))

        pub.publish(result)

if __name__ == '__main__':
    try:
        send_command()
    except rospy.ROSInterruptException:
        pass