#!/usr/bin/env python3

import speech_recognition as sr
from gtts import gTTS
import os 
import platform
import subprocess
from playsound import playsound 
import os
import pyaudio
import numpy as np
import rospy

class Receptionist:
    
    def __init__(self):
        rospy.init_node('Speech_recognition')
        self.host_name = 'Robert'
        self.host_drink = "coke"
        self.names = []
        self.drinks = []
        self.r = sr.Recognizer()
        self.freq = 1000
        self.duration = 0.5
        self.volume = 0.2
        self.sample_rate = 44100
        self.t = np.linspace(0, self.duration, int(self.duration * self.sample_rate), False)
        self.waveform = self.volume * np.sin(2 * np.pi * self.freq * self.t)
        self.stream = pyaudio.PyAudio().open(format=pyaudio.paFloat32, channels=1, rate=self.sample_rate, output=True)
    
    def beep(self, freq, duration, volume=0.2):
        self.sample_rate = 44100  # Taxa de amostragem
        self.t = np.linspace(0, duration, int(duration * self.sample_rate), False)
        self.waveform = volume * np.sin(2 * np.pi * freq * self.t) # Multiplica a amplitude por volume
        self.stream = pyaudio.PyAudio().open(format=pyaudio.paFloat32, channels=1, rate=self.sample_rate, output=True)
        self.stream.write(self.waveform.astype(np.float32).tobytes())
        self.stream.close()
        

    def record_audio(self):
        with sr.Microphone() as source:
            print("Listening...")
            audio = self.r.listen(source, phrase_time_limit=2)
            return audio

    def transcribe_audio(self, audio):
        try:
            text = self.r.recognize_google(audio, language="en-US")
            return text
        except sr.UnknownValueError:
            print("Sorry, I could not understand what you said")

    def play_audio(self, filename):
        if platform.system() == "Windows":
            os.system(f"start {filename}")
        else:
            subprocess.call(["xdg-open", filename])

    def speak_text(self, text, lang="en"):
        tts = gTTS(text=text, lang='en')
        tts.save('audio.mp3')
        playsound('audio.mp3')
    
    def rec_name(self):
        audio = self.record_audio()
        text = self.transcribe_audio(audio)
        while text is None:
            self.speak_text("Sorry, I could not understand what you said. Please, talk again.")
            self.beep(500, 0.5)
            self.beep(300, 0.5)
            audio = self.record_audio()
            text = self.transcribe_audio(audio)
        self.names.append(text)
        print(self.names)
        return self.names

    def rec_drink(self):
        audio = self.record_audio()
        text = self.transcribe_audio(audio)
        while text is None:
            self.speak_text("Sorry, I could not understand what you said. Please, talk again.")
            self.beep(500, 0.5)
            self.beep(300, 0.5)
            audio = self.record_audio()
            text = self.transcribe_audio(audio)
        self.drinks.append(text)
        print(self.drinks)
        return self.drinks

    def start(self):
        self.speak_text("Starting Receptionist Task.")
        for i in range (2):
            self.speak_text("Going to reception.")
            self.speak_text("Waiting for a new guest.")
            self.speak_text("Hello!, My name is Hera.")
            self.speak_text("Please, talk to me after the signal. What's your name?")
            self.beep(500, 0.5)
            self.beep(300, 0.5)
            self.rec_name()
            if i == 0:
                self.speak_text("Hello, " + self.names[0])
                self.speak_text("'Please, take a step back and look at me.'")
                self.speak_text("Please, talk to me after the signal. What's your favorite drink?")
                self.beep(500, 0.5)
                self.beep(300, 0.5)
                drink = self.rec_drink()
                self.speak_text("Oh, nice drink")
                self.speak_text("Ok!, just a second.")
                # vá para a posição para indicar os lugares
                self.speak_text("Please, come close and stand on my left.")
                self.speak_text("Everyone, please look at me.")
                self.speak_text("Hello," + self.host_name)
                self.speak_text("This is" + self.names[0] + "Their favorite drink is" + drink[0])
                self.speak_text(self.names[0] + ", this is" + self.host_name + ". Their favorite drink is" + self.host_drink)
                #indicar lugar para sentar
            if i == 1:
                self.speak_text("Hello, " + self.names[1])
                self.speak_text("'Please, take a step back and look at me.'")
                self.speak_text("Please, talk to me after the signal. What's your favorite drink?")
                self.beep(500, 0.5)
                self.beep(300, 0.5)
                drink = self.rec_drink()
                self.speak_text("Oh, I like" + drink[1] + "too")
                self.speak_text("Ok!, just a second.")
                # vá para a posição para indicar os lugares
                self.speak_text("Please, come close and stand on my left.")
                self.speak_text("Everyone, please look at me.")
                self.speak_text("Hello," + self.host_name + "and" + self.names[0])
                self.speak_text("This is" + self.names[1] + "Their favorite drink is" + drink[1])
                self.speak_text(self.names[1] + ", this is" + self.host_name + ". Their favorite drink is" + self.host_drink)
                self.speak_text(self.names[1]+ ", this is" + self.names[0] + ". Their favorite drink is" + drink[0])
                #indicar um lugar para sentar

if __name__ == '__main__':
    try:
        receptionist = Receptionist()
        receptionist.start()
    except rospy.ROSInterruptException():
        pass