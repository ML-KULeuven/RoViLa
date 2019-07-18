#! /usr/bin/env python

import rospy
import pyaudio as pa
import wave
from std_msgs.msg import String
from speech_node.srv import *
from recognizer_node.srv import *

def record_speech_client():
   
    print("Initializing main node...")
    rospy.init_node('centralnode',anonymous=True)

    chunk_size = 1024
    sample_format = pa.paInt16
    channels = 2
    frequency = 44100
    seconds = 3

    print("Waiting for SpeechStream service to come online")
    rospy.wait_for_service('speech_service')

    try:
        record_speech = rospy.ServiceProxy('speech_service', SpeechStream)
        response_speech_service = record_speech(chunk_size,sample_format,channels,frequency,seconds)
        frames = response_speech_service.stream
        print("Received the frames, ready to construct WAV file")

    except rospy.ServiceException, e:
        print("Service call to the speech_service failed: %s"%e)

    print("Waiting for AudioToText service to come online")
    rospy.wait_for_service('recognizer_service')

    try: 
        recognize_audio = rospy.ServiceProxy('recognizer_service', AudioToText)
        response_recognizer_service = recognize_audio(frames,channels,sample_format,frequency)
        text = response_recognizer_service.text

        print("What was said:")
        print(text)
    except rospy.ServiceException, e:
        print("Service call to the recognizer_service failed: %s"%e)


if __name__ == "__main__":
    record_speech_client()


