import os
import gradio as gr
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.schema import HumanMessage, SystemMessage

load_dotenv()

class Chat:
    def __init__(self):
        self.chat_model = ChatOpenAI()

    def get_chat_response(self, user_message):
        messages = [
            SystemMessage(content=os.getenv('CONTEXTO')),
            HumanMessage(content=user_message),
        ]
        
        response = self.chat_model(messages)
        
        return response.content

class Gradio:
    def __init__(self, chat_interface):
        self.chat_interface = chat_interface

    def launch(self):
        interface = gr.Interface(
            fn=self.chat_interface.get_chat_response,
            inputs="text",
            outputs="text"
        )
        interface.launch()

if __name__ == "__main__":
    chat_interface = Chat()
    gradio_interface = Gradio(chat_interface)
    gradio_interface.launch()