from fastapi import APIRouter, HTTPException
from models.users import User, UserData
from config.client import PostgresClient

router = APIRouter()

client = PostgresClient()

@router.post("/sign_up")
async def sign_up(user: UserData):
    try:
        new_user = User(email=user.email, password=user.password)

        with client.session.begin():
            client.session.add(new_user)
        return True
    
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
