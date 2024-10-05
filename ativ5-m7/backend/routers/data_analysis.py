import io
import jwt
import base64
import pandas as pd
from fastapi import Depends
from fastapi import APIRouter
import matplotlib.pyplot as plt
from models.mall_data import MallData
from config.client import PostgresClient
from fastapi.security import OAuth2PasswordBearer

router = APIRouter()
client = PostgresClient()
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

@router.get("/data_analysis")
async def data_analysis(token: str = Depends(oauth2_scheme)):

    # Verificar se o token é válido
    try:
        jwt.decode(token, "secret", algorithms=["HS256"])
    except Exception as e:
        return {"error": str(e)}
    
    # Obter os dados da tabela MallData do banco de dados
    mall_data_query = client.session.query(MallData).all()

    # Converter os resultados da consulta para um DataFrame
    data = {'Gender': [], 'spendingScore': []}
    for record in mall_data_query:
        data['Gender'].append(record.gender)
        data['spendingScore'].append(record.spending_score)

    dataframe = pd.DataFrame(data)

    # Realizar a análise de dados
    mean_data = dataframe.groupby('Gender')['spendingScore'].mean()

    # Criar o gráfico
    ax = mean_data.plot(kind='bar', color=['skyblue', 'salmon'])
    plt.xlabel('Gender')
    plt.ylabel('Mean Spending Score')
    plt.title('Média de Spending Score por Gender')

    # Converter o gráfico para bytes
    img_bytes = io.BytesIO()
    plt.savefig(img_bytes, format='png')
    img_bytes.seek(0)

    # Converter os bytes para base64
    img_base64 = base64.b64encode(img_bytes.read()).decode('utf-8')

    # Retornar a imagem em base64
    return {"image": img_base64}