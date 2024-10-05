import uuid
from models.base import Base
from sqlalchemy import Column, String
from pydantic import BaseModel

class UserData(BaseModel):
    email: str
    password: str

class User(Base):
    __tablename__ = 'User'
    id = Column(String, primary_key=True, index=True, default=lambda: str(uuid.uuid4()))
    email = Column(String)
    password = Column(String)