import calendar
import time
from txtai.pipeline import TextToSpeech
from playsound import playsound
import soundfile as sf

current_GMT = time.gmtime()

time_stamp = calendar.timegm(current_GMT)

tts = TextToSpeech("NeuML/ljspeech-jets-onnx")

print("Olá eu sou o Falador")

while True:
    user_input = input('Escreva as frases e eu falo ')

    speech = tts(user_input)

    sf.write(f"./out{time_stamp}.wav", speech, 22050)

    playsound(f'./out{time_stamp}.wav')
