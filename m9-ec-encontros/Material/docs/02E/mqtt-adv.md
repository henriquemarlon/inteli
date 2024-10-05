---
title: MQTT - parte 2
sidebar_position: 1
sidebar_class_name: autoestudo
slug: /mqtt2
---

import Admonition from '@theme/Admonition';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# MQTT - Parte 2 (QoS e sessões persistentes)

## 1. Quality of Service

<Admonition 
    type="info" 
    title="Autoestudo">

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/hvhtJORsE5Y" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

O perfil de qualidade de serviço (QoS) de ROS é um acordo entre os clientes e o
broker que define o nível de garantia de chegada de mensagens na comunicação.

### 1.1. Níveis de QoS existentes

<img 
  src="https://www.researchgate.net/publication/329035407/figure/fig21/AS:694644870496259@1542627651369/The-figure-shows-the-different-QoS-levels-used-in-the-MQTT-protocol.jpg"
  alt="Camadas do modelo OSI" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '40vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Os três níveis existentes de QoS são:

1. Entrega no máximo uma vez a mensagem. Aqui o remetente não espera uma
   confirmação de recebimento do destinatário.
2. Entrega ao menos uma vez a mensagem. Aqui o remetente mantem uma cópia da
   mensagem enviada até que recebe a confirmação do destinatário que ela foi
   recebida com sucesso. Caso essa confirmação não chegue em um tempo razoável,
   o remetente reenvia a mensagem (por isso é possível o envio duplicado).
3. Envia apenas uma vez. Aqui o remetente e o destinatário usam um handshake de
   quatro vias para garantir que a mensagem chegue uma e apenas uma vez ao
   destinatário.

### 1.2. Quando usar cada perfil QoS

**Use o QoS 0 quando:**

- Você tem uma conexão estável entre o remetente e o receptor. Um exemplo
  clássico para o QoS 0 é conectar um cliente de teste ou uma aplicação
  frontend a um broker MQTT através de uma conexão cabeada.
- Não importa se algumas mensagens forem perdidas ocasionalmente. A perda de
  algumas mensagens pode ser aceitável se os dados não forem tão importantes ou
  quando os dados são enviados em intervalos curtos.
- Você não precisa de enfileiramento de mensagens. As mensagens só são
  enfileiradas para clientes desconectados se eles tiverem QoS 1 ou 2 e uma
  sessão persistente.

**Use o QoS 1 quando:**

- Você precisa receber todas as mensagens e seu caso de uso pode lidar com
  duplicatas. O nível de QoS 1 é o nível de serviço mais frequentemente
  utilizado porque garante que a mensagem chegue pelo menos uma vez, mas
  permite várias entregas. Claro, sua aplicação deve tolerar duplicatas e ser
  capaz de processá-las adequadamente.
- Você não pode suportar a sobrecarga do QoS 2. O QoS 1 entrega mensagens muito
  mais rápido que o QoS 2.

**Use o QoS 2 quando:**

- É crítico para a sua aplicação receber todas as mensagens exatamente uma vez.
  Isso é frequentemente o caso se uma entrega duplicada pode prejudicar
  usuários da aplicação ou clientes inscritos. Esteja ciente da sobrecarga e de
  que a interação do QoS 2 leva mais tempo para ser concluída.

### 1.3. Usando QoS 2 com Paho e Python

<Tabs>
  <TabItem value="pub" label="Publisher" default>
  ```python showLineNumbers title="publisher.py"
import paho.mqtt.client as mqtt

def on_publish(client, userdata, mid):
    print(f"Message published mid: {mid}")

client = mqtt.Client()
client.on_publish = on_publish
client.connect("localhost", 1891, 60)

topic = "test/topic"
payload = "Hello MQTT with QoS 2"
qos = 2

client.publish(topic, payload, qos)
client.loop_forever()
  ```
  </TabItem>
  <TabItem value="sub" label="Subscriber">
  ```python showLineNumbers title="subscriber.py"
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("test/topic", 2)

def on_message(client, userdata, msg):
    print(f"Received message: {msg.topic} {msg.payload.decode()}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("localhost", 1891, 60)

client.loop_forever()
  ```
  </TabItem>
</Tabs>

Note que é possível configurar separadamente o QoS para publishers e
subscriber. Isso significa que é possível um publisher estar em QoS 1 e o
subcriber em QoS 2 e vice versa. O ideal é garantir que todos estejam
alinhados.

## 2. Persistência de sessão e fila de mensagens

<Admonition 
    type="info" 
    title="Autoestudo">

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/2ETj1fM7-ZA" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

Sessões persistentes em MQTT é a *feature* que permite que um cliente mantenha
suas subscrições e estado das mensagens mesmo após desconectar-se e conectar-se
novamente. Isso ocorre totalmente do lado do broker, que armazena as
informações relevantes para aquele cliente e gerencia sua sessão no quando ele
se reconecta.

Para que um cliente sinalize que quer uma sessão persistente, basta enviar um
flag **cleanSession=*False*** ao broker. Todos os brokers e clientes que
suportam MQTT >=3 devem ter suporte a essa flag.

O cliente consegue confirmar que trata-se de uma sessão persistente pelo
retorno de **acknowledgement** de conexão do broker (`CONNACK`). Essa mensagem
por padrão inclui um flag para dizer se a sessão é persistente ou não.

As mensagens retidas durante a perda de conexão em sessões persistentes são
armazenadas em fila, garantindo sua entrega na ordem correta. Essa fila é
implementada tanto no broker quanto nos clientes para mensagens QoS1 ou QoS2.

### 2.1. O que fica armazenado em sessões persistentes?

As informações que o broker guarda para o cliente que se descontectou e
disponibiliza novamente para ele quando a sessão é resumida podem ser vistas
abaixo:

1. **A existência da sessão em si** - a primeira coisa que o broker guarda é o
   fato de que aquele cliente estava conectado a ele previamente, mesmo que não
   haja mensagens ou mesmo subscrições relacionadas a esse cliente.
2. **Todas as subscrições do cliente**
3. **Todas as mensagens QoS1 ou QoS2 que o cliente ainda não confirmou que
   recebeu**
4. **Todas as mensagens QoS1 ou QoS2 que o cliente recebeu enquanto estava
   offline**
5. **Todas as mensagens QoS2 que ainda não passaram pelo processo completo de
   confirmação**

### 2.2. Quando usar sessões persistentes

De modo geral, deve-se usar sessões persistentes quando:

* **Precisa guardar informações de um tópico importante** - se há um tópico de
  extrema importância para o seu cliente, sessões persistentes garantem que
  nenhuma informação desse tópico é perdida.
* **Cliente com recursos limitados** - pode cair a qualquer momento, portanto
  guardar as informações da sessão é oportuno.
* **As mensagens enviadas pelo cliente são críticas** - se o cliente envia
  informações que são críticas para algum outro sistema, sessões persistentes
  garantem que essas mensagens fiquem enfileiradas mesmo quando o cliente está
  offline.
