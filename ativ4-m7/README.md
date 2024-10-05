# Previsão de Faixa Etária Utilizando Linear Discriminant Analysis

- Este projeto desenvolve uma aplicação FastAPI para prever a faixa etária de uma pessoa com base em três variáveis: gênero, score bancário (de 1 a 100) e renda anual (em dólares/ano). O modelo de previsão é construído utilizando Linear Discriminant Analysis e é servido através de uma API criada com FastAPI.

## Demonstração em Vídeo

https://youtu.be/8ICWlsHuSTg

## Manipulação dos dados

1. O primeiro passo realizado for criar a coluna feixa etária, criada a partir da coluna já existente "Age" e, após a criação da coluna, "Age" é eliminada.
2. O próximo passo foi passar os dados qualitativos para quantitativos através do Label Enconder.
3. Após isso é feita a normalização entre 0 e 1 dos valores a fim melhorar a performace.
4. Por fim o nome das colunas são renomeados para nomes sem espaços (por exemplo, "Annual Income (k$)" vira "annualIncome").

## Criação do moldelo

O modelo foi criado a partir da biblioteca Pycaret. Para sua criação foi definido que seria um modelo de classificação e o target a coluna "ageGroup" (faixa etária). Após execução de vários modelos pelo Pycaret, o que apresentou melhor performace comparada foi o "Linear Discriminant Analysis, em que obtive 68% de acurária. Como este foi o melhor desempenho foi o escolhido para a atividade em questão.

## Api de comunicação com modelo

A Api para comunicação com modelo possui apenas duas rotas, sendo elas `/` e `/previsao`. A rota inicial serve o frontend através do Jinja2Templates do FastAPI. Já a rota de nova previsão é acionada quando selecionados valores de input e enviados, como resposta da requisição é retornada o valor processado pelo modelo.

## Dependências

O projeto requer as seguintes dependências, que podem ser instaladas manualmente ou através do Docker (veja instruções abaixo):

        pycaret==3.0.4
        uvicorn==0.23.2
        fastapi==0.103.1
        pydantic==2.3.0
        pandas==1.5.3


## Execução da aplicação com Docker:

Para rodar a aplicação usando Docker, siga as instruções abaixo:

- Primeiro, faça o pull da imagem Docker com o seguinte comando:

    ```
    docker pull henriquemarlon/ativ4-m7:1.0.0
    ```

- Em seguida, rode a aplicação com o comando:

    ```
    docker run -p 8000:8000 henriquemarlon/ativ4-m7:1.0.0
    ```

#### ***Agora, você pode acessar a aplicação através do navegador no endereço http://0.0.0.0:8000.***

## Estrutura do Projeto:

Aqui está uma explicação detalhada da estrutura do diretório do projeto:

```
├── app.py                # Arquivo principal que contém o código FastAPI para servir o modelo
├── Dockerfile            # Arquivo Dockerfile para conteinerizar a aplicação
├── etl                   # Diretório com scripts e dados para ETL
│   ├── Exploracao_dados.ipynb  # Notebook Jupyter para a exploração de dados
│   ├── logs.log         # Arquivo de log
│   └── Mall_Customers.csv # Dados de entrada em formato CSV
├── .git                 # Diretório do sistema de controle de versão Git
├── Modelo.pkl           # Arquivo contendo o modelo treinado usando Linear Discriminant Analysis
├── README.md            # Este arquivo README com instruções e informações do projeto
├── requirements.txt     # Arquivo com as dependências necessárias para rodar o projeto
└── templates            # Diretório com templates HTML
    └── index.html      # Template HTML para a interface do usuário
```

## Como usar a interface gráfica

Na interface web da aplicação, você encontrará um formulário onde poderá inserir os detalhes:

    - Score Bancário (1-100)
    - Renda Anual (em dólares/ano)
    - Gênero (Feminino/Masculino)
    Após preencher os campos, clique em "Enviar" para obter a previsão da faixa etária, que é calculada pelo modelo Linear Discriminant Analysis e exibida na página.

![image](https://github.com/henriquemarlon/ativ4-m7/assets/89201795/8341876a-506a-45dd-bd92-df5c2eab99de)
