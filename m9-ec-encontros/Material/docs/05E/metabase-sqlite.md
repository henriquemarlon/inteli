---
title: Integração Metabase SQLite
sidebar_position: 4
sidebar_class_name: autoestudo
slug: /metabase-sqlite
---

# Integrando o Metabase ao SQLite

Para conseguir integrar um banco de dados local SQLite no seu Metabase, você
deve executar as seguintes etapas:

1. Configurar o mapeamento de volume de onde está seu arquivo `.db`
2. Adicionar o arquivo .db na tela de configurações de **admin** do metabase.

## 1. Configurando o mapeamento de volume

Por padrão, quando rodamos um container, nenhum volume (ou diretório) do nosso
sistema host é compartilhado com o container. Isso é ótimo para segurança, mas
muitas vezes queremos que nossos containers contem com algum tipo de
persistência. A melhor forma de fazer isso é mapeando volumes. Rode novamente o
comando de setup do metabase, agora com a tag `-v`:

```b̀ash
docker run -d -p 3000:3000 --name metabase -v /caminho/absoluto/volume/host:/ponto/de/mount/container
```

Para monitorar o progresso do container, use:

```bash
docker log -f metabase
```

:::warning

Essa seção apresenta a forma mais rápida de mapear um volume, mas certamente
não a mais indicada. Para maior conveniência e rebetibilidade, crie um
`docker-compose`. Veja a [documentação](https://docs.docker.com/compose/).

:::

## 2. Configuração metabase

Para conseguir configurar o Metabase para ler seu arquivo `.db`, primeiro
precisamos acessar o **menu de admin**. Para fazer isso, clique no ícone de
engrenagem no canto superior direito da sua página inicial e, quando aparecer
um menu *drop-down*, selecione **Admin Settings**.

<img 
  src="./img/metabase-admin.png"
  alt="Acessando o menu de admin" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '60vh',
    marginRight: 'auto'
  }} 
/>
<br/>

A seguir, na aba Setup, clique em **Add a Database**.
<img 
  src="./img/metabase-setup.png"
  alt="Aba setup" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '60vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Agora basta escolher o tipo de database como **SQLite** e preencher o nome do
seu banco de dados e o caminho até o arquivo `.db` (lembre-se que esse é o
caminho dentro do container, não no host).

<img 
  src="./img/metabase-db.png"
  alt="Adicionando um db sqlite" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '60vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Por fim, basta clicar em **Save** e pronto, seu banco de dados SQLite está
conectado ao Metabase.
