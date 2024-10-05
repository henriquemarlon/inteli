import os
import pandas as pd
from dotenv import load_dotenv
from sqlalchemy import create_engine

load_dotenv()

def csv_to_database(csv_file_path):
    database_url = os.getenv("DATABASE_URL")

    if not database_url:
        raise ValueError("A variável de ambiente 'database_url' não está configurada.")

    engine = create_engine(database_url)

    try:
        dataframe = pd.read_csv(csv_file_path)

        table_name = 'MallData'

        dataframe.to_sql(table_name, engine, index=False, if_exists='replace')

        print(f'Dados inseridos com sucesso na tabela {table_name}')
    except Exception as e:
        print(f'Ocorreu um erro: {str(e)}')
    finally:
        engine.dispose()

csv_to_database('mall_customers_pos_data.csv')
