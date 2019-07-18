#! /usr/bin/env python

import rospy
import pyaudio as pa
from std_msgs.msg import String
from speech_node.srv import *


def handle_record_speech(request):

    p = pa.PyAudio()

    print("RECORDING")

    stream = p.open(format=request.sample_format,channels=request.channels,rate=request.frequency,frames_per_buffer=request.chunk_size,input=True)

    frames = []

    for i in range(0, int(request.frequency / request.chunk_size*request.seconds)):
        data = stream.read(request.chunk_size)
        frames.append(data)
    
    print(type(frames))
    print(type(frames[0]))
    print(len(frames))

    stream.stop_stream()
    stream.close()
    p.terminate()

    return SpeechStreamResponse(frames)

def record_speech_server():
    rospy.init_node('speechnode')
    service = rospy.Service('speech_service', SpeechStream, handle_record_speech)
    print "Ready to start speech service."
    rospy.spin()

if __name__ == "__main__":
    record_speech_server()


