#!/usr/bin/env python3
import speech_recognition as sr
import rospy
from std_msgs.msg import String
value_of_mic = sr.Microphone.list_microphone_names()

query = ""

# Voice_recognition
def mic_func():  
    print("Zadaj príkaz: ")
    audio = r.listen(source)

    try:
        # Google recognizer
        query = r.recognize_google(audio, language='sk-SK')
        rospy.loginfo(f"Prikaz: {query}")
        #print(f"Príkaz: {query}")
        return query
    except sr.UnknownValueError:
        print("Could not hear that, Try saying again")
        return None
    except sr.RequestError:
        print("Make Sure that you have a good Internet connection")
        return None
    

if __name__ == "__main__":
    #Subscriber definition
    rospy.init_node("voice_recognition")
    pub = rospy.Publisher("/voice_reg", String, queue_size=10)
    # Recognizer init.
    try:
        r = sr.Recognizer()
        with sr.Microphone(device_index=(len(value_of_mic)-1)) as source:
            r.adjust_for_ambient_noise(source, duration=0.2)
            while query != "ukonči prevádzku":
                query = mic_func()
                if query != "":
                    pub.publish(query)
    except AttributeError:
        print("Nebol detegovany mikrofon")    