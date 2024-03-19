#!/usr/bin/env python3
import speech_recognition as sr
import rospy
from std_msgs.msg import String
print(sr.Microphone.list_microphone_names())

query = ""

def mic_func():  # funkcia pre rozpoznanie hlasu
    print("Zadaj príkaz: ")
    audio = r.listen(source)

    try:
        # pouzitie google modelu pre rozpoznanie jazyka
        query = r.recognize_google(audio, language='sk-SK')
        rospy.loginfo(f"Prikaz: {query}")
        #print(f"Príkaz: {query}")
        return query
    # osetrenie pre chybami
    except sr.UnknownValueError:
        print("Could not hear that, Try saying again")
        return None
    except sr.RequestError:
        print("Make Sure that you have a good Internet connection")
        return None
    

if __name__ == "__main__":
    #definovanie publishera pre odosielanie sprav
    rospy.init_node("voice_recognition")
    pub = rospy.Publisher("/voice_reg", String, queue_size=10)
    # Inicializacia Recognizer s vybraným mikrofónom
    try:
        r = sr.Recognizer()
        with sr.Microphone(device_index=5) as source:
            # definicia zdroja mikrofonu
            r.adjust_for_ambient_noise(source, duration=0.2)
            while query != "koniec":
                query = mic_func()
                if query != "":
                    pub.publish(query)
    except AttributeError:
        print("Nebol detegovany mikrofon")    