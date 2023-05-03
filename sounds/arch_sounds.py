from gtts import gTTS
from playsound import playsound
import pyaudio
import numpy as np
from playsound import playsound


class Sounds:

    def __init__(self):
        self.freq = 1000
        self.duration = 0.5
        self.volume = 0.2
        self.sample_rate = 44100
        self.t = np.linspace(0, self.duration, int(self.duration * self.sample_rate), False)
        self.waveform = self.volume * np.sin(2 * np.pi * self.freq * self.t)
        self.stream = pyaudio.PyAudio().open(format=pyaudio.paFloat32, channels=1, rate=self.sample_rate, output=True)
        self.language = "en-US"

    def beep(self, freq, duration, volume=0.2):
        self.sample_rate = 44100  # Taxa de amostragem
        self.t = np.linspace(0, duration, int(duration * self.sample_rate), False)
        self.waveform = volume * np.sin(2 * np.pi * freq * self.t) # Multiplica a amplitude por volume
        self.stream = pyaudio.PyAudio().open(format=pyaudio.paFloat32, channels=1, rate=self.sample_rate, output=True)
        self.stream.write(self.waveform.astype(np.float32).tobytes())
        self.stream.close()

    def play_audio(self, filename):
        playsound(filename)

    def speak_text(self, text):
        tts = gTTS(text=text, lang=self.language)
        filename = 'audio.mp3'
        tts.save(filename)
        self.play_audio(filename)
    
    def click(self):
        playsound('/home/robofei/Documents/Teste_speech/catkin_ws/src/sounds/Click.mp3')
    
    def toctoc(self):
        playsound('/home/robofei/Documents/Teste_speech/catkin_ws/src/sounds/toctoc.mp3')

teste = Sounds()
click = teste.click()