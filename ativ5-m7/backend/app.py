import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from routers.sign_up import router as sign_up_router
from routers.sign_in import router as sign_in_router
from routers.predict import router as predict_router
from routers.data_analysis import router as data_analysis_router
from routers.insert_all_data import router as insert_all_data_router

app = FastAPI()


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(sign_in_router)
app.include_router(sign_up_router)
app.include_router(predict_router)
app.include_router(data_analysis_router)
app.include_router(insert_all_data_router)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8001)
