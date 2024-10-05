# Use a imagem base do Ubuntu
FROM ubuntu:latest

# Instale as dependências necessárias
RUN apt-get update && apt-get install -y \
    # adicione aqui qualquer dependência necessária && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Exponha a porta necessária
EXPOSE 11434

# Defina o diretório de trabalho
WORKDIR /root

# Volume para armazenar dados
VOLUME ["/root/.ollama"]

# Comando para executar o aplicativo
CMD ["ollama", "-d", "-p", "11434:11434", "--name", "ollama"]
