#! /usr/bin/env python

import rospy
import pyaudio as pa
import wave
from std_msgs.msg import String
from speech_node.srv import *

def record_speech_client():
   
    print("Initializing main node...")
    rospy.init_node('centralnode',anonymous=True)

    chunk_size = 1024
    sample_format = pa.paInt16
    channels = 2
    frequency = 44100
    seconds = 3

    p = pa.PyAudio()

    print("Waiting for SpeechStream service to come online")
    rospy.wait_for_service('speech_service')

    try:
        record_speech = rospy.ServiceProxy('speech_service', SpeechStream)
        response = record_speech(chunk_size,sample_format,channels,frequency,seconds)
        frames = response.stream
        print("Received the frames, ready to construct WAV file")
        
        wf = wave.open("output.wav", 'wb')
        wf.setnchannels(channels)
        wf.setsampwidth(p.get_sample_size(sample_format))
        wf.setframerate(frequency)
        wf.writeframes(b''.join(frames))
        wf.close()

        print("WAV file constructed, ready for further processing")

    except rospy.ServiceException, e:
        print("Service call failes: %s"%e)

if __name__ == "__main__":
    record_speech_client()


