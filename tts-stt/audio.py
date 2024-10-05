import os
import time
import calendar
from pathlib import Path
from openai import OpenAI
from dotenv import load_dotenv

load_dotenv()

current_gmt = time.gmtime()
time_stamp = calendar.timegm(current_gmt)

client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))

def text_to_speech(model="tts-1", voice="alloy", text=None, file_name=f"audio_from_tts.mp3"):
    if text is None:
        raise ValueError("Text input cannot be empty/None.")

    speech_file_path = Path(__file__).parent / file_name

    response = client.audio.speech.create(
        model=model,
        voice=voice,
        input=text
    )
    response.stream_to_file(speech_file_path)

if __name__ == "__main__":
    input_text = input("Write something: ")
    text_to_speech(text=input_text)
