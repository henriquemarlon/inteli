---
title: Introdução ao Metabase
sidebar_position: 2
sidebar_class_name: autoestudo
slug: /metabase
---

import Admonition from '@theme/Admonition';

# Primeiros passos com o Metabase

Metabase é uma ferramenta de inteligência de negócios (Business Intelligence -
BI) de código aberto que permite às empresas visualizar e compartilhar
informações sobre seus dados. Uma das principais vantagens do Metabase é sua
facilidade de uso, que permite até mesmo a usuários não técnicos realizar
consultas de dados complexas através de uma interface gráfica intuitiva. Isso
significa que, sem a necessidade de entender linguagens de consulta como SQL,
usuários podem facilmente extrair insights valiosos de seus dados, criar
dashboards personalizados e gerar relatórios detalhados.

Além da interface amigável, o Metabase se destaca pela sua flexibilidade e
capacidade de integração com uma vasta gama de fontes de dados. Seja
conectando-se a bancos de dados SQL tradicionais, como PostgreSQL e MySQL, ou a
fontes de dados mais modernas, como MongoDB e Google Analytics, o Metabase
facilita a centralização de informações de diferentes origens em um único
local. Isso permite que as equipes de análise de dados tenham uma visão
holística do desempenho da empresa, otimizando a tomada de decisões com base em
dados confiáveis e atualizados.

Outro ponto forte do Metabase é sua comunidade ativa e recursos de colaboração.
Como uma ferramenta de código aberto, ela é constantemente aprimorada por uma
comunidade global de desenvolvedores, que contribuem com novas funcionalidades
e melhorias. Além disso, o Metabase facilita a colaboração entre equipes ao
permitir que usuários compartilhem dashboards e relatórios com facilidade,
promovendo uma cultura de dados na organização. Essas características fazem do
Metabase uma escolha popular entre startups, PMEs e departamentos de grandes
empresas que buscam democratizar o acesso aos dados e fomentar uma abordagem
orientada por dados em suas operações.

## 1. Instalando o Metabase

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
        src="https://www.youtube.com/embed/VZrNqaHlIBs" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

O tutorial de instalação do Metabase pode ser encontrado [nesse
link](https://www.metabase.com/docs/latest/installation-and-operation/running-metabase-on-docker).
O jeito mais fácil de instalar a versão open source é utilizando `docker`.
Portanto, vamos começar dando `pull` na imagem do metabase.

```bash
docker pull metabase/metabase:latest
```

Agora, basta rodar o container a partir da imagem baixada.

```bash
docker run -d -p 3000:3000 --name metabase metabase/metabase
```

Pronto! Sua versão local do metabase já está rodando em (http://localhost:3000)

:::tip

Lembre-se que, para não precisar rodar o `docker` como root, basta adicionar o seu usuário ao grupo docker com o comando:

```bash
sudo groupadd docker
```

```bash
sudo usermod -aG docker $SEU_USUÁRIO
```

Note que é necessário reiniciar o sistema para que essa mudança seja efetuada.

:::

Se tudo deu certo, você vai ver a seguinte página inicial (tenha calma, o
processo pode alguns minutinhos):

<img 
  src="./img/metabase-login.png"
  alt="Tela inicial metabase" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '60vh',
    marginRight: 'auto'
  }} 
/>
<br/>

## 2. Explorando o Metabase

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
        src="https://www.youtube.com/embed/uiK_ESCaM9M" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 3. Um tour do Metabase para iniciantes

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
        src="https://www.youtube.com/embed/utMxX014na4" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>
:::tip

Os vídeos a partir daqui não servem como um tutorial holístico, mas sim uma
visão rápida das diversas ferramentas do Metabase. Sendo assim, não há uma
ordem obrigatória para assistí-los tampouco há a necessidade de assistir todos.
Sugiro assistir o vídeo acima e depois clicar nos vídeos das ferramentas que te
interessam.

:::

## 4. Modelos

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
        src="https://www.youtube.com/embed/8UnLNXETC4c" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 5. Mapas

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
        src="https://www.youtube.com/embed/MzXZfPuNN2E" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 6. Line Charts

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
        src="https://www.youtube.com/embed/AfhhP_cbLNM" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 7. Bar Charts

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
        src="https://www.youtube.com/embed/MTjubnVTTbU" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 8. Combo Charts

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
        src="https://www.youtube.com/embed/MDGsfrtgs78" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 9. Area Charts

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
        src="https://www.youtube.com/embed/6TO3CkHhTCI" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 10. Pie Charts

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
        src="https://www.youtube.com/embed/EigiZF_jOsY" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 11. Progress & Gauge Charts

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
        src="https://www.youtube.com/embed/pF7hN9KIkW4" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 12. Number & Trend Charts

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
        src="https://www.youtube.com/embed/NQA37gmpxGo" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 13. Funnel Charts

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
        src="https://www.youtube.com/embed/mBvSf34EtOc" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 14. Scatterplots

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
        src="https://www.youtube.com/embed/nsbQGdVXZ4M" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 15. Tables & Pivot Tables

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
        src="https://www.youtube.com/embed/RiTcE1oV31M" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 16. Waterfall Charts

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
        src="https://www.youtube.com/embed/DRWD1S-SN_Y" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>
