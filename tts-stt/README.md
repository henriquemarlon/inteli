# Ponderada 8 - Módulo 8
## Henrique Marlon

🚀 Bem-vindo ao projeto Ponderada 8 - Módulo 8! Este projeto envolve transcrição, tradução e criação de áudio usando a API da OpenAI.

## Como Executar

1. Crie um ambiente virtual:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

2. Instale os requisitos da aplicação:
   ```bash
   pip install -r requirements.txt
   ```

3. Crie um arquivo `.env` e adicione a chave da API da OpenAI:
   ```bash
   touch .env
   OPENAI_API_KEY=<sua-chave-de-api>
   ```

4. Execute o arquivo `audio.py` para criar ou editar o áudio usado como base:
   ```bash
   python3 audio.py
   ```

5. Execute o arquivo `main.py` para realizar a transcrição, tradução e criação do áudio no idioma escolhido:
   ```bash
   python3 main.py
   ```

## Vídeo Demonstrativo

https://www.loom.com/share/9d031318e8174632962c1f2c886300f9