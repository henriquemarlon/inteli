import jwt
from config.client import PostgresClient 
from fastapi import APIRouter
from models.users import User, UserData

router = APIRouter()
client = PostgresClient()

@router.post("/sign_in")
async def sign_in(user: UserData):
    db_user = client.session.query(User).filter(User.email == user.email).first()
    if db_user and db_user.password == user.password:
        token = jwt.encode({"user": db_user.email}, "secret")
        return {"token": token}
