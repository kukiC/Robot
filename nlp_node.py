#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Int32, Int16
import time
import threading as thr
import os
import speech_recognition as sr
from gtts import gTTS
import multiprocessing as mp
from socket import *

s = None
status = 0
jeff_signal = 0
chat_bot = None
pub = None
flag = 0
flag1 = 0
flag2 = 0
mutex = thr.Lock()
prev_state = 0

dict = {0: "Ready", 1: 'Grabbing', 2: 'Moving arms', 3: 'pausing for nlp', 4: 'Pausing for sonar', 5: 'Moving base to bed', 6: 'Terminate'}

# NLP Thread
def nlp_thread_func(chat_bot,pub):
    global status, mutex, prev_state, s, flag2
    termination = False
    while termination is False:
        chat_bot.speech_to_text()

        if "robot" in chat_bot.text:
            res = "Hello I am Bedbot, what can I help you with?"
            variable = 0

        elif any(i in chat_bot.text for i in ["grab", "grabbing"]):
            res = "Ok, I will grab your bed sheet"
            variable = 1

        elif any(i in chat_bot.text for i in ["make", "make bed"]):
            res = "Ok, I will make your bed"
            variable = 2

        elif any(i in chat_bot.text for i in ["stop", "stopping"]):
            res = "Stopping current action"
            variable = 3

        elif any(i in chat_bot.text for i in ["go", "go to the bed", "go to bed"]):
            res = "Ok,I will go towards your bed"
            variable = 4
            msg= '4'
            s.send(msg.encode())

        elif any(i in chat_bot.text for i in ["continue", "back to work"]):
            res = "I will now resume my previous action"
            variable = 5

        elif any(i in chat_bot.text for i in ["terminate"]):
            termination = True
            res = "Terminating"
            variable = 6

        else:
            res = "Sorry, can you repeat?"
            variable = 7

        # If a event in higher priority happens, go back to listening
        if status == 1 or (jeff_signal != 0 and variable == 7) or prev_state == variable:
            res = None
            continue

        prev_state = variable

        pub.publish(Int32(variable))
        # Block until mutex is avaivable, make sure only one message is broadcasting
        mutex.acquire()
        chat_bot.text_to_speech(res, variable)
        mutex.release()


def jeffCallback(data):
    global jeff_signal, status, pub, mutex, flag1, dict
    variable = 0
    jeff_signal = int(data.data)

    # Current action finished
    if jeff_signal == 0 and flag1 != 0:
        mutex.acquire()
        chat_bot.text_to_speech("I have finished" + dict[flag1], variable)
        mutex.release()
        flag1 = jeff_signal

    # Pause according to user's command
    elif jeff_signal == 3 and flag1 != 3:
        rospy.sleep(3)
        mutex.acquire()
        chat_bot.text_to_speech("I have paused as requested, do you want me to resume?", variable)
        mutex.release()

    flag1 = jeff_signal

# Callback for SONAR obstacle detection
def callback(data):
    global status, chat_bot, pub, flag, mutex
    if data.data == 0:
        status = 0
    elif data.data == 1:
        status = 1
    else:
        status = 0

    # If it is the first time receive the signal
    if status == 1 and flag == 0:
        res = "Please step back, you are too close"
        variable = 8

        # Block until mutex is avaivable, make sure only one message is broadcasting
        mutex.acquire()
        chat_bot.text_to_speech(res, variable)
        mutex.release()

        pub.publish(Int32(variable))
        flag = 1
        rospy.sleep(3)
    # If the problem is solved
    elif status == 0 and flag == 1:
        flag = 0
        print("resume")


class ChatBot():
    def __init__(self, name):
        print("----- starting up", name, "-----")
        self.name = name
        self.text = ""

    # Perform speech to text
    def speech_to_text(self):
         r = sr.Recognizer()
         mic = sr.Microphone()
         with mic as source:
             print("listening...")
             r.adjust_for_ambient_noise(source,duration = 0.8)
             audio = r.listen(source)
         try:
             self.text = r.recognize_google(audio)
             print("me --> ", self.text)
         except:
             print("me -->  ERROR")

    @staticmethod
    # Text to speech
    def text_to_speech(text, variable):
        print("AI --> ", text)
        speaker = gTTS(text=text, lang="en", slow=False)
        speaker.save("res.mp3")
        os.system("mpg123 res.mp3")
        os.remove("res.mp3")

    def wake_up(self, text):
        return True if self.name in text.lower() else False


def main():
    global chat_bot, pub, p, s, flag2
    rospy.init_node("main_Node", anonymous=True)
    pub = rospy.Publisher('/nlp_state', Int32, queue_size=1)
    sub = rospy.Subscriber('/sonar_state',Int32, callback)
    sub_jeff = rospy.Subscriber('baxter_state_topic', Int16 , jeffCallback)

    # TCP connection to Ridgeback robot
    s=socket()
    s.connect(("10.0.0.149",6666))
    flag2 = 1

    chat_bot = ChatBot("chat_bot")

    # Start a new NLP thread
    nlp = thr.Thread(target=nlp_thread_func, args=(chat_bot,pub))
    nlp.start()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
