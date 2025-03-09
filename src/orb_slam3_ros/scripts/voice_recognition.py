#!/usr/bin/env python3

import rospy
import speech_recognition as sr
from std_msgs.msg import String
import threading

class VoiceRecognition:
    def __init__(self):
        rospy.init_node('voice_recognition', anonymous=True)
        
        # Publisher
        self.voice_pub = rospy.Publisher('/voice_command', String, queue_size=1)
        
        # Recognition setup
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Calibrate for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Microphone calibrated for ambient noise")
        
        # Start recognition in a separate thread
        self.is_running = True
        self.recognition_thread = threading.Thread(target=self.recognition_loop)
        self.recognition_thread.start()
        
        rospy.loginfo("Voice recognition started. Say 'go to [object]' to navigate.")
        
        rospy.on_shutdown(self.shutdown)
        rospy.spin()
    
    def recognition_loop(self):
        while self.is_running and not rospy.is_shutdown():
            try:
                # Listen for voice command
                with self.microphone as source:
                    rospy.loginfo("Listening for commands...")
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                
                # Try to recognize the command
                command = self.recognizer.recognize_google(audio)
                rospy.loginfo(f"Recognized: {command}")
                
                # Publish the command
                self.voice_pub.publish(command)
                
            except sr.WaitTimeoutError:
                pass  # No speech detected, continue listening
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand audio")
            except sr.RequestError as e:
                rospy.logerr(f"Recognition error: {e}")
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")
    
    def shutdown(self):
        self.is_running = False
        if self.recognition_thread.is_alive():
            self.recognition_thread.join(timeout=1.0)

if __name__ == '__main__':
    try:
        VoiceRecognition()
    except rospy.ROSInterruptException:
        pass