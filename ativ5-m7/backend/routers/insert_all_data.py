import pandas as pd
from models.mall_data import MallData
from config.client import PostgresClient
from fastapi import APIRouter, HTTPException

router = APIRouter()
client = PostgresClient()

@router.post("/insert_csv_data")
async def insert_csv_data():
    try:
        # Caminho hardcoded do arquivo CSV (ajuste conforme necessário)
        file_path = "./ai/etl/mall_customers_pos_data.csv"

        # Ler o CSV para um DataFrame
        df = pd.read_csv(file_path)

        # Iterar sobre as linhas do DataFrame e inserir os dados no banco de dados
        for index, row in df.iterrows():
            new_data = MallData(
                gender=row['Gender'],
                annual_income=row['annualIncome'],
                spending_score=row['spendingScore'],
                prediction=row['ageGroup']
            )
            client.session.add(new_data)

        # Commit para persistir as mudanças no banco de dados
        client.session.commit()

        return {"message": "Data from CSV inserted successfully"}
    except Exception as e:
        # Em caso de erro, reverta as alterações e retorne um erro HTTP
        client.session.rollback()
        raise HTTPException(status_code=500, detail=f"Error inserting data from CSV: {str(e)}")
    finally:
        # Certifique-se de fechar a sessão, independentemente de ocorrer uma exceção ou não
        client.session.close()
