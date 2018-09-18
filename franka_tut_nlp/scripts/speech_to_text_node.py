#!/usr/bin/env python

import rospy
import speech_recognition as sr

from std_msgs.msg import String

raw_input_publisher = rospy.Publisher('speech_to_text', String, queue_size=10)


def transmit(audio):
    if audio:
        print audio
        raw_input_publisher.publish(audio)

def recordAudio():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source)
        print("Say something!")
        audio = r.listen(source, phrase_time_limit=10)

    data = ""
    try:
        # Uses the default API key
        # To use another API key: `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        data = r.recognize_google(audio)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

    return data

if __name__ == '__main__':
    try:
        rospy.init_node('speech_to_text_input')
        while not rospy.is_shutdown():
            data = recordAudio()
            transmit(data)
    except rospy.ROSInterruptException:
        pass
