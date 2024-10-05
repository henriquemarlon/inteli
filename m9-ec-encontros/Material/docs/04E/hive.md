---
title: HiveMQ
sidebar_position: 2
sidebar_class_name: autoestudo
slug: /hivemq
---

import Admonition from '@theme/Admonition';

# Primeiros passos com o HiveMQ

## 1. O que é o HiveMQ

HiveMQ é uma plataforma de broker MQTT desenvolvida para facilitar a
comunicação entre dispositivos em uma rede IoT, oferecendo alta escalabilidade
e confiabilidade para a transmissão de mensagens em tempo real. Diferentemente
de brokers MQTT locais como o Mosquitto, que são excelentes para aplicações de
menor escala e ambientes de teste, o HiveMQ é projetado para suportar
implantações de grande escala, oferecendo recursos avançados que são
particularmente úteis em ambientes corporativos e industriais.

As principais vantagens do HiveMQ em relação a um broker MQTT local como o
Mosquitto incluem:

1. **Escalabilidade**: HiveMQ pode lidar com milhões de conexões simultâneas de
   dispositivos, tornando-o ideal para grandes redes IoT. Isso é essencial para
   empresas que precisam garantir a comunicação eficiente entre um grande
   número de dispositivos distribuídos globalmente.
2. **Alta Disponibilidade e Tolerância a Falhas**: HiveMQ oferece suporte a
   clusters e balanceamento de carga, garantindo que o serviço esteja sempre
   disponível, mesmo em caso de falhas em um dos servidores. Essa
   característica é crucial para aplicações críticas onde a continuidade do
   serviço é imperativa.
3. **Segurança Reforçada**: Enquanto o Mosquitto suporta as funcionalidades
   básicas de segurança, o HiveMQ oferece recursos de segurança avançados, como
   autenticação e autorização sofisticadas, integração com sistemas de
   gerenciamento de identidade e suporte para TLS/SSL para a comunicação
   segura.
4. **Suporte Empresarial**: O HiveMQ vem com suporte profissional e
   consultoria, oferecendo às empresas a assistência necessária para a
   implantação, operação e manutenção de suas soluções IoT em larga escala.
   Isso é particularmente valioso para projetos complexos e missões críticas.
5. **Interoperabilidade e Integração**: HiveMQ foi projetado para integrar-se
   facilmente com sistemas e tecnologias existentes, oferecendo plugins e APIs
   que permitem uma personalização extensiva e a capacidade de interoperar com
   sistemas de mensagens empresariais, bases de dados e outras aplicações.
6. **Monitoramento e Administração**: A plataforma oferece ferramentas
   avançadas para monitoramento e administração, permitindo que as equipes de
   operações acompanhem o desempenho do broker em tempo real, identifiquem e
   resolvam problemas rapidamente.

## 2. Configurando o HiveMQ

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
        src="https://www.youtube.com/embed/y6PFe3YDr2g" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 3. Segurança com o HiveMQ

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
        src="https://www.youtube.com/embed/ug5UAylAqEY" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

Por padrão, os brokers MQTT costumam ter suporte a autenticação por segurança
na camada de transporte (TLS). O HiveMQ habilita essa verificação por padrão em
seu cluster, assim como permite a criação de novos usuários e criação de regras
de autorização complexas para esse usuário (e.g. restrição para
publicar/subscrever em determinados tópicos).

Para configurar o sistema de segurança do HiveMQ, basta acessar a página de
configuração do seu cluster através do botão **MANAGE CLUSTER**

<img 
  src="./img/hivemq-cluster.png"
  alt="Camadas do modelo OSI" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '60vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Nessa tela, você pode navegar até **ACCESS MANAGEMENT**. Nessa tela, você pode
cadastrar novos usuários e controlar o nível de autorização de cada um deles.

<img 
  src="./img/hivemq-access.png"
  alt="Camadas do modelo OSI" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '60vh',
    marginRight: 'auto'
  }} 
/>
<br/>

## 4. Testando o seu cluster

:::warning

Há uma diferença entre o **Windows** e o Ubuntu que faz com que o mosquitto
**não consiga se comunicar** com o broker do **HiveMQ** sem a instalação de um
certificado de segurança. Portanto, se estiver usando Windows (shame on
you...), mude a ferramenta de teste para o [MQTT
Cli](https://www.hivemq.com/blog/mqtt-cli/).

O tópico reportando o erro está nesse
[link](https://community.hivemq.com/t/failing-with-connection-return-codes-7/1673/2).

:::

Para testar o cluster criado, vamos tentar acessá-lo utilizando as credenciais
criadas no passo anterior e o **mosquitto** como cliente. Para isso, rode:

```bash title="Comando subscriber"
mosquitto_sub -h {ID-DO-SEU-CLUSTER}.s1.eu.hivemq.cloud -p 8883 -u {USUARIO} -P {SENHA} -t my/test/topic
```

E, para o publisher:

```bash title="Comando publisher"
mosquitto_pub -h {ID-DO-SEU-CLUSTER}.s1.eu.hivemq.cloud -p 8883 -u {USUARIO} -P {SENHA} -t 'my/test/topic' -m 'Hello'
```

Se tudo deu certo, a mensagem `Hello` deve ter aparecido na tela do seu
subscriber.
