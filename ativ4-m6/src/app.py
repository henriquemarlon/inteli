import os
import uvicorn
from fastapi import FastAPI
from dotenv import load_dotenv
from supabase import create_client, Client
from fastapi.responses import HTMLResponse
from controllers.record_controller import RecordController

load_dotenv()
supabase: Client = create_client(supabase_url=os.getenv(
    "SUPABASE_URL"), supabase_key=os.getenv("SUPABASE_KEY"))

app = FastAPI()

rc = RecordController()


@app.get("/")
async def index():
    return HTMLResponse(content=open("./view/index.html", "r").read(), status_code=200)


@app.post("/record")
async def record():
    output = rc.record()
    supabase.storage.from_("prova2m6").upload(f'{output}.mp4', f'./{output}')
    alert = """
    <head>
        <meta http-equiv="refresh" content="0.5; URL='http://127.0.0.1:8000/'" />
    </head>
    <script>
        alert("Video salvo com sucesso!! :)")
    </script>
    """
    return HTMLResponse(content=alert)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
