import os
from models.base import Base
from models.users import User
from models.mall_data import MallData
from dotenv import load_dotenv
from sqlalchemy import create_engine, MetaData
from sqlalchemy.orm import sessionmaker

load_dotenv()


class PostgresClient():
    def __init__(self):
        # URL do seu banco de dados PostgreSQL
        self.url = os.getenv("DATABASE_URL")
        self.engine = create_engine(f'{self.url}', echo=True)
        self.Session = sessionmaker(bind=self.engine)
        self.session = self.Session()
        self.meta = MetaData()

        Base.metadata.create_all(self.engine, tables=[
                                 User.__table__, MallData.__table__])

    def get_session(self):
        return self.session

# client = PostgresClient()
