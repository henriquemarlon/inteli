---
title: MQTT - parte 3
sidebar_position: 1
sidebar_class_name: autoestudo
slug: /mqtt3
---

import Admonition from '@theme/Admonition';

# MQTT Parte 3

## 1. Mensagem retidas

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
        src="https://www.youtube.com/embed/Ct5s4gXefn4" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

Mensagens retidas nada mais são do que mensagens que ficam "pinadas" ao tópico.
Apenas uma mensagem pode ser retida por tópico e ela geralmente serve para
garantir que os inscritos consigam ver o último estado conhecido daquele
tópico.

## 2. Last will e Testament

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
        src="https://www.youtube.com/embed/dNy9GEXngoE" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

A mensagem de testamento (LWT) é um caso particular da mensagem retida, pois
ela é uma mensagem que o cliente envia com antecedência ao broker. O broker,
por sua vez, só dispara essa mensagem como uma mensagem retida no tópico nos
seguintes casos:

1. **Falha de I/O de Rede**
2. **Cliente não se comunica dentro do período de Keep Alive**
3. **Cliente fecha a conexão sem o pacote DISCONNECT**
4. **Conexão fechada por falha de protocolo**

Basicamente, essa é uma forma de garantir que o sistema como um todo seja
notificado em caso de falhas de comunicação de um cliente, o que pode ser útil
para ativar rotinas de mitigação/fail safe.

## 3. Keep alive e client takeover

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
        src="https://www.youtube.com/embed/2EsrWOFPmc4" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

Keep alive nada mais é do que um período de tempo negociado entre o broker e o
cliente (é um parâmetro do construtor de um cliente com Paho, por exemplo) para
que o cliente mande um pacote **PINGREQ** e o broker responda com um
**PINGRESP**.

Client takeover é uma ferramenta de monitoramento de portas TCP. Quando um
cliente se desconecta, teoricamente o protocolo TCP prevê que seja possível que
a outra parte envolvida na comunicação seja notificada. Na prática, é possível
que a conexão fique só 'meio aberta', o que faz com que a porta fique ocupada
em caso de tentativas de reconectar-se por alguma das partes. O Client takeover
é um mecanismo onde o broker fica resopnsável por monitorar a porta TCP e, caso
haja uma tentativa de reconexão por parte de um cliente, o broker fecha a
conexão antiga para garantir que o cliente consiga voltar.

## 4. Wildcards

Wildcards em MQTT são símbolos especiais usados em tópicos de subscrição que
permitem inscrever-se em múltiplos tópicos simultaneamente. Eles são
incrivelmente úteis para criar subscrições flexíveis e escaláveis. Existem dois
tipos principais de wildcards em MQTT:

1. **`+` (Wildcard de um único nível)**: Este wildcard substitui exatamente um
   nível de tópico. Por exemplo, `sensor/+/temperatura` pode corresponder a
   `sensor/casa/temperatura` e `sensor/escritorio/temperatura`, mas não a
   `sensor/casa/quarto/temperatura`.

2. **`#` (Wildcard de múltiplos níveis)**: Este wildcard pode substituir vários
   níveis de tópicos e sempre deve ser colocado no final do tópico de
   subscrição. Por exemplo, `sensor/#` pode corresponder a qualquer tópico que
   comece com `sensor/`, incluindo `sensor/casa/temperatura` ou
   `sensor/escritorio/luminosidade/quarto`.

### 4.1. Usando wildcards (Python)

O exemplo abaixo assume que você tem um broker local configurado para a porta
TCP `1891`.


```python showLineNumbers title="subscriber.py"
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    print("Conectado com o código de resultado: " + str(rc))
    client.subscribe("sensor/+/temperatura")  # Usando o wildcard +

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("localhost", 1891, 60)
client.loop_forever()
```

### 4.2. Usando wildcards (Go)

```go
package main

import (
	"fmt"
	"os"
	"time"
	"github.com/eclipse/paho.mqtt.golang"
)

var messagePubHandler mqtt.MessageHandler = func(client mqtt.Client, msg mqtt.Message) {
	fmt.Printf("Recebido: %s de %s\n", msg.Payload(), msg.Topic())
}

var connectHandler mqtt.OnConnectHandler = func(client mqtt.Client) {
	fmt.Println("Conectado")
	client.Subscribe("sensor/+/temperatura", 1, nil)  // Usando o wildcard +
}

func main() {
	opts := mqtt.NewClientOptions()
	opts.AddBroker("tcp://endereco_do_broker_mqtt:1891")
	opts.SetClientID("go_mqtt_client")
	opts.OnConnect = connectHandler
	opts.DefaultPublishHandler = messagePubHandler

	client := mqtt.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		fmt.Println(token.Error())
		os.Exit(1)
	}
	time.Sleep(60 * time.Second)
	client.Disconnect(250)
}
```
