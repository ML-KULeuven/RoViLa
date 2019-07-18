#! /usr/bin/env python

import rospy
import pyaudio as pa
import wave
import speech_recognition as sr
from std_msgs.msg import String
from recognizer_node.srv import *

def construct_wav_file(request):
    filename = "audio_files.wav"

    p = pa.PyAudio()

    wf = wave.open(filename, 'wb')
    wf.setnchannels(request.channels)
    wf.setsampwidth(p.get_sample_size(request.sample_format))
    wf.setframerate(request.frequency)
    wf.writeframes(b''.join(request.frames))
    wf.close()

    return filename

def transform_audio_to_text(filename):

    r = sr.Recognizer()

    audiofile = sr.AudioFile(filename)

    with audiofile as source:
        audio = r.record(source)
    
    try:
        print("recognizing audio...")
        text = r.recognize_sphinx(audio)
        print(text)
    except sr.UnknownValueError:
        print("Unknown Value")

    return text

def handle_recognize_speech(request):

    print("Constructing WAV file...")
    audiofilename = construct_wav_file(request)
    print("WAV file constructed")

    #text = transform_audio_to_text(audiofilename)
    text = transform_audio_to_text("audio.wav")

    return AudioToTextResponse("CONSTRUCTED")

def recognize_speech_server():

    print("Initializing recognizernode")
    rospy.init_node('recognizernode', anonymous=True)

    service = rospy.Service('recognizer_service', AudioToText, handle_recognize_speech)
    print("Service is now online")
    rospy.spin()

if __name__ == "__main__":
    recognize_speech_server()


