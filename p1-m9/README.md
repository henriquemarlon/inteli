# P1-M9

O código contido neste repositório representa o sistema de simulação. Este projeto foi construído conforme as [golang-standards](https://github.com/golang-standards/project-layout) [^1].

## Dependências:

Antes de continuar, é necessário instalar as dependências para a execução dos comandos abaixo. Acesse o [link](https://docs.docker.com/desktop/install/ubuntu/).

## Como rodar o sistema:

Abaixo estão as possíveis interações e as instruções de como realizá-las.

#### Rodar testes:

Aqui, todos os comandos necessários estão sendo abstraídos por um arquivo Makefile. Se você tiver curiosidade para saber o que o comando abaixo faz, basta conferir [aqui](https://github.com/Inteli-College/2024-T0002-EC09-G04/blob/main/backend/Makefile#L7).

###### Comando:

```shell
make test
```

###### Output:

```shell
[+] Running 1/1
 ✔ Container p1-m9-broker-1  Started                                                                                                                                       0.0s 
Running the tests
?       github.com/henriquemarlon/p1-m9/cmd/freezer     [no test files]
?       github.com/henriquemarlon/p1-m9/cmd/refrigerator        [no test files]
?       github.com/henriquemarlon/p1-m9/cmd/subscriber  [no test files]
=== RUN   TestGenerateFreezerPayload
--- PASS: TestGenerateFreezerPayload (0.00s)
=== RUN   TestConnectFreezerMQTT
--- PASS: TestConnectFreezerMQTT (0.00s)
=== RUN   TestFreezerMessageTransmissionAndQOS
    freezer_test.go:68: New message on topic /sectors: {"id":"ST-1","type":"freezer","temperature":-18,"timestamp":"2024-03-08 15:58:06.464641995 -0300 -03 m=+0.009711507"}
--- PASS: TestFreezerMessageTransmissionAndQOS (2.00s)
=== RUN   TestGenerateRefrigeratorPayload
--- PASS: TestGenerateRefrigeratorPayload (0.00s)
=== RUN   TestConnectRefrigeratorMQTT
--- PASS: TestConnectRefrigeratorMQTT (0.00s)
=== RUN   TestRefrigeratorMessageTransmissionAndQOS
    refrigerator_test.go:68: New message on topic /sectors: {"id":"ST-1","type":"refrigerator","temperature":2,"timestamp":"2024-03-08 15:58:08.4738584 -0300 -03 m=+2.018927982"}
--- PASS: TestRefrigeratorMessageTransmissionAndQOS (2.00s)
PASS
coverage: 87.5% of statements
ok      github.com/henriquemarlon/p1-m9/internal/domain/entity  4.023s  coverage: 87.5% of statements
[+] Running 2/2
 ✔ Container p1-m9-broker-1  Removed                                                                                                                                       0.2s 
 ✔ Network p1-m9_default     Removed  
```

> [!NOTE]
> - No meio do processo, é necessário subir um broker local para realizar os testes de transmissão de mensagens entre os tópicos.

#### Rodar a simulação:

Mais uma vez, todos os comandos necessários estão sendo abstraídos por um arquivo Makefile. Se você tiver curiosidade para saber o que o comando abaixo faz, basta conferir [aqui](https://github.com/Inteli-College/2024-T0002-EC09-G04/blob/main/backend/Makefile#L15C2-L15C7).

###### Comando:

```bash
make run
```

###### Output:

```shell
subscriber-1    | ST-0 - Temperature: 4 ºC [OK Refrigerator] 
subscriber-1    | ST-0 - Temperature: 3 ºC [OK Refrigerator] 
subscriber-1    | ST-1 - Temperature: -19 ºC [OK Freezer] 
subscriber-1    | ST-2 - Temperature: -20 ºC [OK Freezer] 
subscriber-1    | ST-4 - Temperature: -14 ºC [ALERT High Temperature - Freezer] 
subscriber-1    | ST-3 - Temperature: -20 ºC [OK Freezer] 
subscriber-1    | ST-0 - Temperature: 9 ºC [OK Refrigerator] 
subscriber-1    | ST-0 - Temperature: 3 ºC [OK Refrigerator] 
subscriber-1    | ST-0 - Temperature: 8 ºC [OK Refrigerator] 
subscriber-1    | ST-2 - Temperature: -18 ºC [OK Freezer] 
subscriber-1    | ST-3 - Temperature: -23 ºC [OK Freezer] 
subscriber-1    | ST-4 - Temperature: -21 ºC [OK Freezer] 
subscriber-1    | ST-0 - Temperature: -16 ºC [OK Freezer] 
subscriber-1    | ST-1 - Temperature: -26 ºC [ALERT Low Temperature - Freezer] 
subscriber-1    | ST-0 - Temperature: 7 ºC [OK Refrigerator] 
subscriber-1    | ST-1 - Temperature: -19 ºC [OK Freezer] 
subscriber-1    | ST-3 - Temperature: -16 ºC [OK Freezer] 
subscriber-1    | ST-2 - Temperature: -21 ºC [OK Freezer] 
subscriber-1    | ST-4 - Temperature: -24 ºC [OK Freezer] 
subscriber-1    | ST-0 - Temperature: 9 ºC [OK Refrigerator] 
subscriber-1    | ST-0 - Temperature: 3 ºC [OK Refrigerator] 
subscriber-1    | ST-4 - Temperature: -10 ºC [ALERT High Temperature - Freezer] 
subscriber-1    | ST-1 - Temperature: -10 ºC [ALERT High Temperature - Freezer] 
subscriber-1    | ST-2 - Temperature: -28 ºC [ALERT Low Temperature - Freezer] 
subscriber-1    | ST-3 - Temperature: -22 ºC [OK Freezer] 
subscriber-1    | ST-0 - Temperature: -22 ºC [OK Freezer] 
subscriber-1    | ST-3 - Temperature: -24 ºC [OK Freezer] 
subscriber-1    | ST-2 - Temperature: -29 ºC [ALERT Low Temperature - Freezer] 
subscriber-1    | ST-1 - Temperature: -26 ºC [ALERT Low Temperature - Freezer] 
subscriber-1    | ST-0 - Temperature: -27 ºC [ALERT Low Temperature - Freezer] 
subscriber-1    | ST-0 - Temperature: 0 ºC [ALERT Low Temperature - Refrigerator] 
subscriber-1    | ST-0 - Temperature: 12 ºC [ALERT High Temperature - Refrigerator] 
subscriber-1    | ST-0 - Temperature: 11 ºC [ALERT High Temperature - Refrigerator] 
subscriber-1    | ST-1 - Temperature: -12 ºC [ALERT High Temperature - Freezer] 
subscriber-1    | ST-3 - Temperature: -19 ºC [OK Freezer] 
subscriber-1    | ST-0 - Temperature: -19 ºC [OK Freezer] 
subscriber-1    | ST-2 - Temperature: -29 ºC [ALERT Low Temperature - Freezer] 
subscriber-1    | ST-4 - Temperature: -18 ºC [OK Freezer] 
subscriber-1    | ST-0 - Temperature: 9 ºC [OK Refrigerator] 
subscriber-1    | ST-0 - Temperature: 7 ºC [OK Refrigerator] 
```

> [!NOTE]
>  - Este comando está subindo todos os serviços presentes no arquivo compose.yml. São eles, o broker local, a simulação e a api-test que está sendo usada, por hora apenas para mostrar o log do que está sendo transmitido pela simulação.

#### Rodar a visualização da cobertura de testes:

Novamente, todos os comandos necessários estão sendo abstraídos por um arquivo Makefile. Se você tiver curiosidade para saber o que o comando abaixo faz, basta conferir [aqui](https://github.com/Inteli-College/2024-T0002-EC09-G04/blob/main/backend/Makefile#L21).

###### Comando:

```bash
make coverage 
```

###### Output:
![output_coverage](https://github.com/henriquemarlon/p1-m9/assets/89201795/4128b513-10bd-4200-8e06-285da5701830)

> [!NOTE]
>  - Este comando está criando, a partir do arquivo `coverage_sheet.md`, uma visualização da cobertura de testes nos principais arquivos Go.

[^1]: A estrutura de pastas escolhida para este projeto está de acordo com as convenções e padrões utilizados pela comunidade de desenvolvedores Golang.
