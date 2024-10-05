---
title: MongoDB com Python
sidebar_position: 3
sidebar_class_name: autoestudo
slug: /pythonmongo
---

# Utilizando mongoDB com Python

## 1. Opção síncrona (pymongo)

O PyMongo é uma biblioteca Python que fornece uma interface para trabalhar com
o MongoDB, um popular sistema de banco de dados NoSQL orientado a documentos.
Ela é a implementação síncrona (bloqueante) oficial do MongoDB para Python, o
que significa que ela espera que as operações de banco de dados sejam
concluídas antes de prosseguir com a execução do código. Isso é diferente das
abordagens assíncronas, onde o código pode continuar a ser executado sem
esperar que as operações do banco de dados sejam concluídas.

A biblioteca PyMongo oferece uma ampla gama de funcionalidades para interagir
com o MongoDB, incluindo a criação, leitura, atualização e exclusão de
documentos, bem como a realização de consultas complexas e agregações.

Aqui está um exemplo simples de como usar o PyMongo para se conectar a um banco
de dados MongoDB, inserir um documento e recuperá-lo:

```python showLineNumbers title="exemplo_mongo.py"
from pymongo import MongoClient

# Conectar ao servidor MongoDB (por padrão, na porta 27017 do localhost)
client = MongoClient('mongodb://localhost:27017/')

# Selecionar o banco de dados 'meu_banco_de_dados'
db = client['meu_banco_de_dados']

# Selecionar a coleção 'minha_colecao'
colecao = db['minha_colecao']

# Inserir um documento na coleção
documento = {'nome': 'Fulano', 'idade': 30}
resultado_insercao = colecao.insert_one(documento)
print(f'Documento inserido com o ID: {resultado_insercao.inserted_id}')

# Buscar o documento inserido
documento_encontrado = colecao.find_one({'nome': 'Fulano'})
print(f'Documento encontrado: {documento_encontrado}')
```

Neste exemplo, primeiro criamos uma instância do `MongoClient` para nos
conectar ao servidor MongoDB. Em seguida, selecionamos o banco de dados e a
coleção com os quais queremos trabalhar. Usamos o método `insert_one` para
inserir um documento na coleção e, finalmente, usamos o método `find_one` para
recuperar o documento que acabamos de inserir.

## 2. Opção assíncrona (motor)

A biblioteca Motor é uma implementação assíncrona do driver MongoDB para
Python, projetada para ser utilizada com frameworks assíncronos como o FastAPI,
Tornado e asyncio. Ela permite que as operações de banco de dados sejam
realizadas de forma não bloqueante, ou seja, sem interromper a execução do
programa enquanto aguarda a resposta do banco de dados. Isso é particularmente
útil em ambientes de servidor web assíncronos, onde o bloqueio pode levar a um
desempenho reduzido.

Aqui está um exemplo simples de como usar a biblioteca Motor para se conectar a
um banco de dados MongoDB e realizar operações básicas:

```python showLineNumbers title="exemplo_motor.py"
import motor.motor_asyncio

async def main():
    # Conectar ao banco de dados MongoDB
    client = motor.motor_asyncio.AsyncIOMotorClient('mongodb://localhost:27017')
    db = client.test_database

    # Inserir um documento
    document = {"name": "John", "age": 30}
    result = await db.test_collection.insert_one(document)
    print(f'Documento inserido com o ID: {result.inserted_id}')

    # Buscar um documento
    document = await db.test_collection.find_one({"name": "John"})
    print(f'Documento encontrado: {document}')

    # Fechar a conexão
    client.close()

# Executar a função assíncrona principal
import asyncio
asyncio.run(main())
```

Diferenças entre PyMongo e Motor:

1. **Sincronia vs. Assincronia**: PyMongo é síncrono, o que significa que ele
   bloqueia a execução do programa até que a operação de banco de dados seja
   concluída. Motor, por outro lado, é assíncrono e não bloqueia a execução, o
   que é ideal para ambientes de servidor web assíncronos.

2. **Uso com frameworks**: Motor é projetado para ser usado com frameworks
   assíncronos como FastAPI, Tornado e asyncio, permitindo que as operações de
   banco de dados sejam integradas de forma eficiente em aplicações
   assíncronas. PyMongo é mais adequado para scripts e aplicações síncronas.

3. **API**: Embora a API do Motor seja semelhante à do PyMongo, ela é adaptada
   para operações assíncronas. Por exemplo, as operações de banco de dados em
   Motor são precedidas pela palavra-chave `await`.

Em resumo, a escolha entre PyMongo e Motor depende do contexto da aplicação. Se
você estiver trabalhando em um ambiente assíncrono, como um servidor web com
FastAPI, Motor é a escolha ideal devido à sua natureza não bloqueante e
integração eficiente com frameworks assíncronos. Para aplicações síncronas,
PyMongo é uma opção mais simples e direta.
