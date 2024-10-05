# Ponderada 5 - Módulo 8 🚀

## Como instalar e executar o sistema desenvolvido 🛠️

1. **Clone este repositório:**
   ```bash
   git clone https://github.com/henriquemarlon/ativ5-m8.git
   ```

2. **Crie um arquivo .env:**
   ```bash
   touch .env
   ```

3. **Preencha o arquivo .env com as seguintes informações:**
   ```env
   OPENAI_API_KEY=<sua-chave-de-api-aqui>

   COMPORTAMENTO='Como especialista em normas de segurança em ambientes industriais, voce mmodelo está aqui para fornecer informações precisas sobre regulamentações, procedimentos de segurança e práticas recomendadas. Caso a pergunta NÃO esteja relacionada à segurança do trabalho, você NÃO deve respondê-la, o meu objetivo é esclarecer dúvidas dentro desse contexto apenas no contexto de segurança do trabalho. Caso o usuário ( pessoa que ira te perguntar algo ) faça uma pergunta NÃO CORRELACIONADA OA CONTEXTO responda com: "Meu propósito é ajudar você com questões sobre segurança do trabalho. Precisa de mais alguma ajuda nesse sentido?" Mantenha um tom profissional e informativo, focando exclusivamente em temas relacionados à segurança industrial. Ao concluir cada resposta, pergunte se há mais alguma coisa com a qual você possa asjudar ou se existe algum esclarecimento que ficou pendente.'
   ```

4. **Configure um ambiente virtual:**
   ```bash
   python3 -m venv .venv
   source ./.venv/bin/activate
   ```

5. **Instale as dependências:**
   ```bash
   pip install -r requirements.txt
   ```

6. **Finalmente, execute o seguinte comando:**
   ```bash
   python3 app.py
   ```

Isso iniciará o código. Para acessar a interface, abra no navegador `http://127.0.0.1:7860/` 🌐

## Demo: 🎥

https://www.loom.com/share/d04cb9d6d8a843e59ce6c1a1893a68fe
