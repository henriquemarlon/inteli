---
title: Integrando MQTT e Metabase
sidebar_position: 5
sidebar_class_name: autoestudo
slug: /metabase-mqtt
---

# Integração MQTT/HiveMQ/Metabase

Por se tratar de uma solução open source que já estamos rodando em um
container, o Metabase acaba sendo bem simples de integrar com outros serviços
(ou até mesmo hosteá-lo na nuvem). Para o nosso uso, nesse primeiro momento
vamos utilizar o banco de dados em Sqlite como o ponto de integração entre
nossa rede MQTT e o Metabase.

## 1. Solução mais simples

Para essa finalidade, vamos criar um script em Python que trabalha como
subscriber de um tópico vinculado ao nosso broker em nuvem e ao mesmo tempo
grava as informações recebidas em um banco de dados sqlite.

```python showLineNumbers title="backend_de_pobre.py"
import paho.mqtt.client as paho
from paho import mqtt
from dotenv import load_dotenv
import sqlite3
import os

load_dotenv() # Le variáveis de ambiente do arquivo .env

# Configurações do broker
broker_address = os.getenv("BROKER_ADDR")
port = 8883
topic = "my/test/topic"
username = os.getenv("HIVE_USER")
password = os.getenv("HIVE_PSWD")

# Conexão com o banco de dados SQLite
conn = sqlite3.connect('dados.db')
cursor = conn.cursor()

def on_connect(client, userdata, flags, rc, properties):
    print(f"CONNACK received with code {rc}")
    client.subscribe(topic, qos=1)

def on_message(client, userdata, msg):
    print(f"{msg.topic} (QoS: {msg.qos}) - {msg.payload.decode('utf-8')}")
    # Insere os dados recebidos no banco de dados
    cursor.execute("INSERT INTO dados (valor) VALUES (?)", (msg.payload.decode(),))
    conn.commit()

# Instanciação do cliente
client = paho.Client(paho.CallbackAPIVersion.VERSION2, "Subscriber", protocol=paho.MQTTv5)
client.on_connect = on_connect

# Configurações de TLS
client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
client.username_pw_set(username, password)  # Configuração da autenticação

client.on_message = on_message

# Conexão ao broker
client.connect(broker_address, port=port)

# Loop de espera por mensagens
client.loop_forever()
```

Note que o exemplo acima assume que o banco de dados já existe, portanto é uma
boa ideia criá-lo com atecedência:

```bash
sqlite3 dados.db
```

E garanta também que a tabela `dados` existe com esse script SQL:

```sql showLineNumbers title="cria_tabela.sql"
CREATE TABLE dados (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    valor TEXT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

## 2. Considerações sobre a solução

Essa solução apresenta alguns problemas. A essa altura do curso, vocês já devem
ser capaz de indentificar alguns deles. Vamos listar alguns:
1. O mais assintoso é o nível de acoplamento do código acima. A maioria dos
   sistemas que trabalha com bancos de dados vai ter uma sepração de
   perocupações entre a captura das mensagens e a interação com o banco de
   dados. Entre o subscriber e o banco deveria haver uma **api** com **rotas**
   definidas. Não vou dar esse código de mão beijada aqui, tem que sobrar algo
   para a ponderada =).
2. Desempenho - o SQLite é um banco de dados leve e não é otimizado para altas
   taxas de escrita ou para lidar com grandes volumes de dados. Isso pode ser
   um gargalo se o fluxo de dados for intenso.
3. Escalabilidade: Dado que o SQLite é um banco de dados de arquivo único, ele
   não é adequado para aplicações distribuídas ou que exigem escalabilidade
   horizontal.
4. Concorrência: O SQLite bloqueia todo o banco de dados durante a escrita, o
   que pode levar a problemas de concorrência se houver múltiplos processos
   tentando escrever no banco de dados ao mesmo tempo.
5. Integridade de Dados: Embora o SQLite seja confiável, ele pode não ser a
   melhor escolha para aplicações críticas onde a integridade dos dados é
   primordial, especialmente em ambientes com alto risco de falhas de hardware
   ou desligamentos inesperados.

A maioria desses problemas não serão resolvidos nessa sprint, mas é bom saber
das limitações das nossas soluções.
