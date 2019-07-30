#! /usr/bin/env python

import rospy
import subprocess
from std_msgs.msg import String 
from parser_node.srv import *
from os.path import expanduser

def handle_parse_text(request):

    user = expanduser("~")
    path = user + "/DTAI_Internship/src/parser_node/scripts/jar/"

    parsed = subprocess.check_output(['java', '-jar', path + "parsing.jar", path + "trained_vectors/trained_fold_0", request.text])
    
    print("The deduced logic form: ")
    print(parsed)

    return ParseTextToLogicFormResponse(parsed)


def parse_text_server():
    rospy.init_node('parsernode',anonymous=True)
    service = rospy.Service('parser_service', ParseTextToLogicForm, handle_parse_text)
    print("parser_service started")
    rospy.spin()

if __name__ == "__main__":
    parse_text_server()


