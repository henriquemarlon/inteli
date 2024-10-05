# Atividade 2 - Módulo 8

### Pacote Intelligible:

O pacote de intelligible se encontra no diretório `ativ2-m8/src/intelligible`. Nessa nova implementação, temos um chatbot representado pelo executável daedalus que, por meio de expressões regulares, identifica uma intenção do usuário em relação ao lugar onde ele deseja ir e transforma aquela intenção em coordenadas a serem passadas para o robô.

Para executa-lo é preciso seguir as seguintes instruções:

- Na pasta raíz desse projeto rode o seguinte comando:

```
sudo chmod +x ./navigation_zsh.sh
```

> [!IMPORTANT]
> Verifique qual a versão do seu shell com o seguinte comando: ```echo $SHELL```, caso ela difira do tipo zsh rode o comando acima com o sufixo bash: ```sudo chmod +x ./navigation_bash.sh```.

- Em seguida, no mesmo diretório, rode o comando:

```
./navigation_zsh.sh
```

> [!IMPORTANT]
> Mais uma vez verifique qual a versão do seu shell com o seguinte comando: ```echo $SHELL```, caso ela difira do tipo zsh rode o comando acima com o sufixo bash: ```./navigation_bash.sh```.

- Em seguida, na janela do X-Term criada, indique a intenção entre as seguintes possobilidades:

+ sun
+ ariadne
+ minotaur
+ wings

Depois de obter sucesso com os comandos acima, você cumpriu a rotina de mapeamento. Agora, basta clicar CTRL+C para encerrar todos os processos criados pelo script `.sh`.

#### Video da navegação utilizando o pacote intelligible
Neste [link](https://youtu.be/akOfqSXRPs8) é possível ver o funcionamento do pacote atualizado com o chatbot.
