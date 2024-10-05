from pycaret.classification import *
import pandas as pd
from pydantic import BaseModel
from fastapi import FastAPI, Request
from fastapi.templating import Jinja2Templates

app = FastAPI()

templates = Jinja2Templates(directory="templates")

class DadosEntrada(BaseModel):
    spendingScore: float
    annualIncome: float
    Gender: float

@app.get('/')
def serve_html(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.post("/previsao")
async def create_item(item: DadosEntrada):
    print(item)

    modelo_carregado = load_model('Modelo')

    dados_de_entrada = pd.DataFrame({
        "spendingScore":[item.spendingScore],
        "annualIncome":[item.annualIncome],
        "Gender":[item.Gender]
    })

    previsao = modelo_carregado.predict(dados_de_entrada)

    if previsao[0] == 0:
        response = "Adolescente"
    elif previsao[0] == 1:
        response = "Adulto"
    elif previsao[0] == 2:
        response = "Idose"
    elif previsao[0] == 3:
        response = "Jovem Adulto"

    print(response)
    return f"{response}"

