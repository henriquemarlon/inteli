---
title: MongoDB Atlas com Python
sidebar_position: 2
sidebar_class_name: autoestudo
slug: /altas-python
---

import Admonition from '@theme/Admonition';

# Utilizando o MongoDB Atlas c/ Python

## 1. Utilizando pymongo

Para interagir com um banco de dados MongoDB Atlas usando Python, você pode
usar a biblioteca `pymongo`. Abaixo está um exemplo simples que mostra como se
conectar a um cluster do MongoDB Atlas, inserir um documento em uma coleção e
recuperar documentos da coleção.

Primeiro, certifique-se de ter o `pymongo` instalado. Se não tiver, você pode
instalá-lo usando pip:

```bash
pip install pymongo
```

Em seguida, use o seguinte código Python como exemplo:

```python showLineNumbers title="python_atlas.py"
from pymongo.mongo_client import MongoClient
from pymongo.server_api import ServerApi
import os
from dotenv import load_dotenv

# Carregar variáveis de ambiente do arquivo .env
load_dotenv()

# Recuperar usuário e senha do arquivo .env
mongo_user = os.getenv('MONGO_USER')
mongo_password = os.getenv('MONGO_PASSWORD')

uri = f"mongodb+srv://{mongo_user}:{mongo_password}@cluster-teste.q2pkwbd.mongodb.net/?retryWrites=true&w=majority&appName=Cluster-teste"

# Create a new client and connect to the server
client = MongoClient(uri, server_api=ServerApi('1'))

# Send a ping to confirm a successful connection
try:
    client.admin.command('ping')
    print("Pinged your deployment. You successfully connected to MongoDB!")
except Exception as e:
    print(e)
```

:::warning

Note que, para rodar o código acima, você precisa configurar um `.env` e usar a
biblioteca `pydotenv`.

:::

## 2. Utilizando motor

```python showLineNumbers title="python_atlas_motor.py"
import os
from motor.motor_asyncio import AsyncIOMotorClient
from dotenv import load_dotenv
import asyncio

load_dotenv()

mongo_user = os.getenv('MONGO_USER')
mongo_password = os.getenv('MONGO_PASSWORD')

uri = f"mongodb+srv://{mongo_user}:{mongo_password}@cluster-teste.q2pkwbd.mongodb.net/?retryWrites=true&w=majority&appName=Cluster-teste"

async def main():
    client = AsyncIOMotorClient(uri)
    db = client.admin

    try:
        await db.command('ping')
        print("Pinged your deployment. You successfully connected to MongoDB!")
    except Exception as e:
        print(e)
    finally:
        client.close()

if __name__ == '__main__':
    asyncio.run(main())
```
