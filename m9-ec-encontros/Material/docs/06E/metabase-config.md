---
title: DB de aplicação Metabase
sidebar_position: 3
sidebar_class_name: autoestudo
slug: /metabase-persitence
---

# Configurando persistência de sessão no Metabase

O Metabase é uma ferramenta de análise de dados que utiliza um banco de dados
embutido H2 para armazenar suas configurações e usuários por padrão. No
entanto, quando executado em um contêiner Docker, é importante mapear um volume
para garantir que essas configurações sejam persistentes, mesmo que o contêiner
seja reiniciado ou removido.

## 1. Criando um diretório para armazenar os dados do Metabase

Primeiro, você precisa criar um diretório no seu sistema de arquivos para
armazenar os dados do Metabase. Isso garantirá que as informações sejam
mantidas fora do contêiner.

```bash
mkdir -p /path/to/metabase-data
```

Substitua `/path/to/metabase-data` pelo caminho onde você deseja armazenar os
dados.

## 2. Execute o container com mapeamento de volume

Agora, você pode executar o Metabase em um contêiner Docker, mapeando o
diretório criado para o local padrão onde o Metabase armazena seus dados dentro
do contêiner (`/metabase.db`).

```bash
docker run -d -p 3000:3000 \
    -v /path/to/metabase-data:/metabase.db \
    --name metabase metabase/metabase
```

Neste comando:

- `-d` executa o contêiner em segundo plano.
- `-p 3000:3000` mapeia a porta 3000 do contêiner para a porta 3000 do host,
  permitindo que você acesse a interface do Metabase.
- `-v /path/to/metabase-data:/metabase.db` mapeia o diretório criado no sistema
  de arquivos do host para o diretório `/metabase.db` dentro do contêiner.
- `--name metabase` dá um nome ao contêiner para facilitar sua identificação.

## 3. Acessar e configurar o Metabase

Após iniciar o contêiner, você pode acessar o Metabase em seu navegador, usando
o endereço `http://localhost:3000`. Siga o assistente de configuração para
configurar sua instância do Metabase.

## 4. Verificar persistência dos dados

Para testar se a configuração é persistente:

1. Faça algumas alterações na configuração do Metabase, como adicionar um novo
   usuário ou conectar a uma fonte de dados.
2. Remova o contêiner do Metabase:

    ```bash
    docker rm metabase -f
    ```

3. Reinicie o contêiner:

```bash
docker run -d -p 3000:3000 \
    -v /path/to/metabase-data:/metabase.db \
    --name metabase metabase/metabase
```

4. Acesse o Metabase novamente e verifique se as alterações feitas
   anteriormente foram mantidas.

**Nìce**


<img 
  src="https://pnghq.com/wp-content/uploads/pnghq.com-leorio-png-4651-download.png"
  alt="Tela inicial metabase" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '60vh',
    marginRight: 'auto'
  }} 
/>
<br/>

