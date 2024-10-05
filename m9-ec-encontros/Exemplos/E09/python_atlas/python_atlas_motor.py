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
