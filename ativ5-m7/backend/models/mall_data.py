import uuid
from models.base import Base
from sqlalchemy import Column, String, Integer
from pydantic import BaseModel

class MallDataModel(BaseModel):
    gender: int
    annual_income: int
    spending_score: int


class MallData(Base):
    __tablename__ = 'MallData'
    id = Column(String, primary_key=True, index=True, default=lambda: str(uuid.uuid4()))
    gender = Column(Integer)
    annual_income = Column(Integer)
    spending_score = Column(Integer)
    prediction = Column(String)