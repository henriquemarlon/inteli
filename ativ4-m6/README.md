# Ponderada 4 - Backend para transmissão e armazenamento de imagens

Com o objetivo de atingir os requisitos dessa ponderada, foi feita uma implmentação de um aplicação web utilizando FastApi, Supabase e Opencv, tendo em vista respectivamente o backend, o file storage (bucket) para os arquivos e a captação da imagem da webcam.

Para rodar o projeto aqui referenciado se atente as seguintes instruções:

1º ```make env_template``` para crar um arquivo .env com as variáveis de ambiente necessárias para interagir com a api do supabase.

2º ```cd app.py``` para entrar no diretório onde se encontra o servidor.

3º ```python app.py``` para iniciar o servidor.

4º Abra a seguinte rota no seu navegador: http://0.0.0.0:8000 para ter acesso ao frontend da aplicação. Nele você poderá iniciar a rotina de captação da webcam clicando no botão da tela.

5º Para encerrar a rotina e armazenar o arquivo contido na pasta ```/view/assets``` no supabase clique na tecla "Q" do seu teclado.

Para ver um demo da implementação, fique a vontade para acessar o vídeo no seguinte link: https://www.youtube.com/watch?v=A32IqFKF7qw