import jwt
import pandas as pd
from fastapi import Depends
from fastapi import APIRouter
from pycaret.classification import *
from config.client import PostgresClient
from fastapi.security import OAuth2PasswordBearer
from models.mall_data import MallDataModel, MallData
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

router = APIRouter()
client = PostgresClient()
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

@router.post("/predict")
async def predict(item: MallDataModel, token: str = Depends(oauth2_scheme)):
    try:
        jwt.decode(token, "secret", algorithms=["HS256"])
    except Exception as e:
        return {"error": str(e)}

    modelo_carregado = load_model('./ai/Modelo')

    dados_de_entrada = pd.DataFrame({
        "spendingScore": [item.spending_score],
        "annualIncome": [item.annual_income],
        "Gender": [item.gender]
    })

    # Validar os dados antes de fazer a previsão
    if dados_de_entrada.isnull().values.any():
        return {"error": "Os dados de entrada contêm valores nulos."}

    try:
        dados_de_entrada = dados_de_entrada.astype(int)
    except ValueError as ve:
        return {"error": f"Erro ao converter dados para inteiros: {ve}"}

    previsao = modelo_carregado.predict(dados_de_entrada)

    if previsao[0] == 0:
        response = "Adolescente"
    elif previsao[0] == 1:
        response = "Adulto"
    elif previsao[0] == 2:
        response = "Idose"
    elif previsao[0] == 3:
        response = "Jovem Adulto"

    prediction = MallData(gender=item.gender, annual_income=item.annual_income, spending_score=item.spending_score, prediction=response)

    client.session.add(prediction)
    client.session.commit()
    client.session.refresh(prediction)

    return {"prediction": response}