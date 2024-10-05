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
