# Utilize a imagem oficial do Node.js como base
FROM python:3.10.2

# Diretório de trabalho
WORKDIR /lambda

COPY . .

# 
RUN pip install --no-cache-dir --upgrade -r requirements.txt

#
CMD ["python", "app.py"]
