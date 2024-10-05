# TO-DO Tasks: API Sync

## Instruções para executar a API localmente

Entre em `m10-p1` crie um virtual environment, ative e instale as bibliotecas necessárias:

```
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Crie as tabelas caso elas não exitam ainda:

```python
python create_table.py
```

Após isso, execute a API:

```
python3 main.py
```

## Instruções para executar a API localmente

Para atender a segunda milestone da atividade foi feito um Dockerfile que virtualiza a api síncrona aqui referenciada. Para rodá-la siga as instruções abaixo:

Rode a imagem utilizando o arquivo ```compose.yaml```:

```bash
docker compose up
```

A documentação Swagger da API estará disponível em http://127.0.0.1:3000/docs

## Arvore de arquivos

```
m10-p1
├── static
│   └── swagger.json
├── api.json
├── create_tables.py
├── main.py
├── Dockerfile
├── README.md
├── requirements.txt
├── sqlite.db
```

### Descrição dos arquivos

- static/swagger.json: Arquivo JSON usado para gerar o Swagger, utilizado na documentação da API.
- insomnia.json: Coleção de Insomnia contendo configurações para testar a API.
- create_tables.py: Script executado para criar as tabelas no banco de dados SQLite.
- main.py: Arquivo principal responsável pela execução da API.
- README.md: Este arquivo contendo informações sobre o projeto.
- requirements.txt: Lista das bibliotecas necessárias para o funcionamento do sistema.
  
Teste da Aplicação:

acesse o video do [arquivo](https://drive.google.com/file/d/1DGb8XQTDOYAJtt6bUcmQ8A2cGxA45KAt/view)
