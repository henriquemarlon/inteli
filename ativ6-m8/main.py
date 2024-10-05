import os
import time
import streamlit as st
from dotenv import load_dotenv
from langchain.chains import RetrievalQA
from langchain.chat_models import ChatOpenAI
from langchain.document_loaders import WebBaseLoader
from langchain.embeddings import OpenAIEmbeddings
from langchain.prompts.chat import (ChatPromptTemplate, HumanMessagePromptTemplate, SystemMessagePromptTemplate)
from langchain.text_splitter import CharacterTextSplitter
from langchain.vectorstores import Chroma

load_dotenv()
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

def main():

    system_template = """Utilize the provided contextual information to respond to the user's inquiry. If unfamiliar with the answer, simply acknowledge not knowing and refrain from fabricating a response."""

    messages = [
        SystemMessagePromptTemplate.from_template(system_template),
        HumanMessagePromptTemplate.from_template("{question}"),
    ]

    ChatPromptTemplate.from_messages(messages)

    url = st.text_input("Insert the URL:")
    question = st.text_input("Ask a question:")

    if st.button("Submit", type="primary"):

         with st.spinner("Loading..."):

            loader = WebBaseLoader(url) 

            data = loader.load()

            text_splitter = CharacterTextSplitter( chunk_size=500, chunk_overlap=40)

            docs = text_splitter.split_documents(data)
            
            embedding_function = OpenAIEmbeddings()
            
            vectordb = Chroma.from_documents(documents=docs, embedding=embedding_function)

            retriever = vectordb.as_retriever()
            
            llm = ChatOpenAI(model_name='gpt-3.5-turbo')

            qa = RetrievalQA.from_chain_type(llm=llm, retriever=retriever)

            response = qa({"query": question})
            
            st.subheader('Answer:')
            output_container = st.empty()
            
            words = response['result'].split()
            for i in range(len(words)):
                output_container.write(" ".join(words[:i+1]))
                time.sleep(0.1) 

if __name__ == '__main__':
    main()