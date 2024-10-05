---
title: Revisão MQTT
sidebar_position: 2
sidebar_class_name: opcional
slug: /mqtt
---

import Admonition from '@theme/Admonition';

# Revisão MQTT

## 1. O que é MQTT

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
        src="https://www.youtube.com/embed/jTeJxQFD8Ak" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 2. Principais características do MQTT

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
        src="https://www.youtube.com/embed/r89uHL2wj5Q" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 3. O modelo Pub/Sub

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
        src="https://www.youtube.com/embed/HCzQJMdHcy0" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 4. Setup de conexão MQTT

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
        src="https://www.youtube.com/embed/vVJk5rES5vY" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 5. Publish/Subscribe/Unsubscribe

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
        src="https://www.youtube.com/embed/t2b1CwQmDRY" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 6. Melhores práticas para tópicos

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
        src="https://www.youtube.com/embed/juq_l70Vg1w" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 8. MQTT vs HTTP

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
        src="https://www.youtube.com/embed/LKz1jYngpcU" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 7. MQTT vs ROS

### 7.1. TCP vs UDP

<img 
  src="https://engineertips.files.wordpress.com/2019/01/tcp-vs-udp.png"
  alt="Camadas do modelo OSI" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '70vh',
    marginRight: 'auto'
  }} 
/>
<br/>


Falar sobre as diferenças e semalhanças entre o MQTT e o ROS significa
basicamente falar sobre as diferenças e semelhanças entre TCP e UDP. O ROS
utiliza comunicação UDP e o MQTT é construido em cima do TCP.

<img 
  src="https://miro.medium.com/v2/resize:fit:720/format:webp/0*8iDIOIBEdSdxzKmY.gif"
  alt="Camadas do modelo OSI" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>
<img 
  src="https://miro.medium.com/v2/resize:fit:720/format:webp/0*juQ6FJ4XsznftWpe.gif"
  alt="Camadas do modelo OSI" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>

A principal diferença entre o TCP e o UDP é a **checagem de conexão**. TCP é um
protocolo que faz **checagem de erros**, garante a **conexão com o servidor** a
cada etapa da comunicação e preserva a **ordem dos pacotes**. UDP não faz nada
disso. Talvez a pergunta mais óbvia agora seja: por que diabos eu usaria UDP se
ele não faz nenhuma dessas checagens? **Velocidade**. UDP é mais rápido
justamente por não fazer todas essas verificações. Não é a toa que esse é o
protocolo utilizado para serviços como **Netflix**, **Zoom** ou qualquer outro
serviço em que seja aceitável perder alguns pacotes, mas é essencial manter um
**throughput alto**.

Tendo em vista essa diferença entre o TCP e o UDP, é possível voltar para a
comparação original, entre **ROS** e **MQTT**. Para fechar nossa comparação,
vamos considerar para que serve cada uma dessas tecnologias?

1. O ROS foi feito para realizar a comunicação de sistemas relacionados à
   robótica em uma rede local. Que tipos de informações são compartilhadas?
   Dados de sensores, atuadores, câmeras que são **essenciais** para a operação
   do robô e precisam ter **latência mínima**. Faz sentido UDP, né?
2. O MQTT foi feito para dispositivos IoT, que tipicamente contam com alguns
   sensores e o nome do jogo é **conservar energia**. Sendo assim, é muito
   comum uma **taxa de aquisiçõa** bem mais baixa. Se temos uma taxa baixa,
   torna-se mais importante garantir que as **poucas mensagens enviadas chegam
   ao seu destino**. Faz sentido o TCP, né?

### 7.2. Broker vs ROScore

Ainda há diversas outras diferenças entre ROS e MQTT, mas vou delinear apenas
mais uma: a diferença entre o broker e o Roscore. Vocês sequer devem saber o
que é Roscore, pois usaram o ROS2 que **esconde** essa complexidade. O
**Roscore** é o software responsável por garantir que o pattern **observer** é
implementado corretamente. Isso significa criar toda a infraestrutura para o
funcionamento dos tópicos. O **Broker** MQTT faz **exatamente a mesma função**.
A principal diferença é que não podemos nos dar ao luxo de ignorá-lo. A
**escolha/desenvolvimento do broker** é uma das principais etapas de
implementação de um projeto IoT. Nesse primeiro momento vamos apenas utilizar
uma implementação open source chamada **mosquitto**, mas trabalharemos mais com
o broker na **sprint 2**.
