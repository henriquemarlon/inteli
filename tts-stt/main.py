from pathlib import Path
from openai import OpenAI
import os
from dotenv import load_dotenv

load_dotenv()

client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))

# Transcribe audio to text
def transcribe_audio(file_name="audio_from_tts.mp3", model="whisper-1", response_format="text"):
    with open(file_name, "rb") as audio_file:
        transcript = client.audio.transcriptions.create(
            model=model,
            file=audio_file,
            response_format=response_format
        )
    print(f"The audio contains the following text: {transcript}")
    return transcript

# Translate text to a specified language using ChatGPT
def chat_with_gpt(prompt, language):

    response = client.chat.completions.create(model="gpt-3.5-turbo", 
    messages=[
        {"role": "system", "content": f"You are a text translator and must translate to {language}"},
        {"role": "user", "content": prompt},
    ])

    obj = response.choices[0].message

    print(f"The text translated to {language}: {obj.content}")
    return obj.content

# Menu to choose which language to translate to
def menu(transcript):
    print("Which language do you want to translate to:")
    print("1. English")
    print("2. Japanese")
    print("3. Spanish")

    options = {
        "1": "English",
        "2": "Japanese",
        "3": "Spanish"
    }

    while True:
        choice = input("Enter the number of your desired option: ")

        if choice in ['1', '2', '3']:
            response = chat_with_gpt(transcript, options[choice])
            return response
        else:
            print("Invalid option. Please choose a valid option.")

# Convert text to speech
def create_speech_file(model="tts-1", voice="alloy", input_text=None, file_name="speech.mp3"):
    if input_text is None:
        raise ValueError("Input cannot be None.")

    speech_file_path = Path(__file__).parent / file_name

    response = client.audio.speech.create(
        model=model,
        voice=voice,
        input=input_text
    )

    print('Audio finished :)') 
    response.stream_to_file(speech_file_path)

if __name__ == "__main__":
    transcript = transcribe_audio()
    english_response = menu(transcript)
    create_speech_file(input_text=english_response)
