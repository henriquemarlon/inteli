---
title: Introdução ao Eclipse Paho
sidebar_position: 3
sidebar_class_name: autoestudo
slug: /paho
---

# Introdução ao Eclipse Paho

O Eclipse Paho é um projeto de código aberto que fornece implementações de
clientes para as mensagens e protocolos de comunicação de Machine-to-Machine
(M2M) e Internet of Things (IoT). Uma das principais características do Paho é
a implementação de clientes para o protocolo MQTT (Message Queuing Telemetry
Transport), que é um protocolo de mensagens leve e eficiente, ideal para
ambientes de IoT devido ao seu baixo uso de largura de banda e eficiência
energética.

Além disso, o Eclipse Paho também oferece suporte a outros protocolos
relacionados à IoT, como o MQTT-SN, que é uma variação do MQTT para redes
sensíveis (como ZigBee). A ênfase do Paho está na criação de clientes que são
leves, eficientes e fáceis de usar, tornando-o uma escolha popular entre
desenvolvedores que trabalham em projetos de IoT.

O projeto Paho é parte da Eclipse IoT Working Group, uma comunidade
colaborativa de empresas e indivíduos que trabalham juntos para desenvolver
plataformas e frameworks de código aberto para IoT. O envolvimento da Eclipse
Foundation assegura que o Paho continue a ser desenvolvido e mantido
ativamente, seguindo padrões abertos e sendo acessível a todos os
desenvolvedores interessados em soluções de IoT.

## 1. Configurando um Broker MQTT Local com Mosquitto

Antes de utilizar as bibliotecas do Eclipse Paho em Python ou Go, é necessário
configurar um broker MQTT. O Mosquitto é uma opção popular devido à sua leveza
e facilidade de uso.

### 1.1. Instalação do Mosquitto

1. **Windows**: Baixe o instalador do [site oficial do
   Mosquitto](https://mosquitto.org/download/) e siga as instruções de
   instalação.
2. **Linux**: Use o gerenciador de pacotes da sua distribuição. Por exemplo, no
   Ubuntu, execute `sudo apt-get install mosquitto mosquitto-clients`.
3. **macOS**: Use o Homebrew, executando `brew install mosquitto`.

### 1.2. Configuração Básica

Após a instalação, o Mosquitto pode ser iniciado com a configuração padrão. No
entanto, é recomendável criar um arquivo de configuração básico.

1. Crie um arquivo chamado `mosquitto.conf` e adicione as seguintes linhas para
   uma configuração básica:

   ```
   listener 1891
   allow_anonymous true
   ```

2. Inicie o Mosquitto com este arquivo de configuração: 

```bash
mosquitto -c mosquitto.conf
```

## 2. Utilizando o Eclipse Paho em Python

Para usar o Paho em Python, primeiro instale a biblioteca e, em seguida,
escreva um script simples para publicar e assinar mensagens.

### 2.1. Instalação

Instale a biblioteca Paho MQTT para Python:

```bash
pip install paho-mqtt
```

### 2.2. Publisher

```python showLineNumbers title="publisher.py"
import paho.mqtt.client as mqtt
import time

# Configuração do cliente
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "python_publisher")

# Conecte ao broker
client.connect("localhost", 1891, 60)

# Loop para publicar mensagens continuamente
try:
    while True:
        message = "Hello MQTT " + time.strftime("%H:%M:%S")
        client.publish("test/topic", message)
        print(f"Publicado: {message}")
        time.sleep(2)
except KeyboardInterrupt:
    print("Publicação encerrada")

client.disconnect()
```

Aqui não há muito o que ressaltar do código acima, ele se assemelha muito ao
que já estamos acostumados a fazer com ROS. Primeiro, instanciamos um objeto do
tipo cliente com:

```python
mqtt.Client(client_id="", clean_session=True, userdata=None, protocol=MQTTv311, transport="tcp")
```

Após isso, nos conectamos a nosso broker local rodando em mosquitto com:
```python
client.connect(host, port=1883, keepalive=60, bind_address="")
```

A seguir, utilizamos dentro de um loop infinito:
```python
client.publish(topic, payload=None, qos=0, retain=False)
```
Para publicar uma mensagem em um tópico.

:::info

Todas as declarações de métodos e construtores, assim como uma explicação mais
detalhada de cada um dos seus parâmetros, pode ser visto na [documentação da
biblioteca paho-mqtt](https://pypi.org/project/paho-mqtt/).

:::

### 2.3. Subscriber

```python showLineNumbers title="subscriber.py"
import paho.mqtt.client as mqtt

# Callback quando uma mensagem é recebida do servidor.
def on_message(client, userdata, message):
    print(f"Recebido: {message.payload.decode()} no tópico {message.topic}")

# Callback para quando o cliente recebe uma resposta CONNACK do servidor.
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Conexão bem sucedida!")
        client.subscribe("test/topic")
    else:
        print(f"Conexão falhou! Código {reason_code}")
        exit(reason_code)

# Configuração do cliente
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "python_subscriber")
client.on_connect = on_connect
client.on_message = on_message

# Conecte ao broker
client.connect("localhost", 1891, 60)

# Loop para manter o cliente executando e escutando por mensagens
client.loop_forever()
```

Aqui nós temos um mecanismo muito semelhante aos **callbacks** de ROS, só que
temos dois tipos de callback:

1. on_message - gatilho para quando há uma nova mensagem em um tópico no qual
   estamos inscritos; e
2. on_connect - gatilho para quando a conexão com o broker é bem sucedida.

Note que a configuração da inscrição ao tópico `test/topic` se dá apenas quando
há uma conexão estabelecida com o broker. Esse tipo de confirmação jamais
poderia ser feita com o ROS, pois ele usa um protocolo que não fornece
confirmação de recebimento de mensagens/conexões (UDP).

## 3. Utilizando o Eclipse Paho em Go

Para usar o Paho em Go, instale a biblioteca e escreva um exemplo de código
para interagir com o MQTT.

### 3.1. Criando um novo módulo

Como já haviamos visto na seção de setup da linguagem Go, é mais fácil primeiro
criar um módulo, adicionar o import necessário em nosso código para, enfim,
instalar todas as dependências de forma automática. Sendo assim, vamos criar o
novo módulo:

```bash
go mod init paho-go
```

### 3.2. Publisher

Agora vamos criar o arquivo `publisher.go` e preenche-lo com o código para
publicar uma mensagem continuamente no tópico `test/topic`:

```go showLineNumbers title="publisher.go"
package main

import (
	"fmt"
	"time"

	MQTT "github.com/eclipse/paho.mqtt.golang"
)

func main() {
	opts := MQTT.NewClientOptions().AddBroker("tcp://localhost:1891")
	opts.SetClientID("go_publisher")

	client := MQTT.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		panic(token.Error())
	}

	for {
		text := "Hello MQTT " + time.Now().Format(time.RFC3339)
		token := client.Publish("test/topic", 0, false, text)
		token.Wait()
		fmt.Println("Publicado:", text)
		time.Sleep(2 * time.Second)
	}
}
```

### 3.3. Subscriber

```go showLineNumbers title="subscriber.go"
package main

import (
	"fmt"
	MQTT "github.com/eclipse/paho.mqtt.golang"
)

var messagePubHandler MQTT.MessageHandler = func(client MQTT.Client, msg MQTT.Message) {
	fmt.Printf("Recebido: %s do tópico: %s\n", msg.Payload(), msg.Topic())
}

func main() {
	opts := MQTT.NewClientOptions().AddBroker("tcp://localhost:1891")
	opts.SetClientID("go_subscriber")
	opts.SetDefaultPublishHandler(messagePubHandler)

	client := MQTT.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		panic(token.Error())
	}

	if token := client.Subscribe("test/topic", 1, nil); token.Wait() && token.Error() != nil {
		fmt.Println(token.Error())
		return
	}

	fmt.Println("Subscriber está rodando. Pressione CTRL+C para sair.")
	select {} // Bloqueia indefinidamente
}
```

### 3.4. Instalando as dependencias e rodando o código

Agora basta rodar:

```bash
go mod tidy
```

Para instalar as dependências e

```bash
go run publisher.go
```

```bash 
go run subscriber.go
```
