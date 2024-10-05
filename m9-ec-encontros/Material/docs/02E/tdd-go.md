---
title: Testes em Go
sidebar_position: 4
sidebar_class_name: autoestudo
slug: /tdd-go
---

# Implementação de testes em Go

Para implementar os conceitos de TDD em Go, vamos repetir o exemplo feito com
Python de uma calculadora. Antes de mais nada, vamos criar nosso módulo em Go:

```bash
go mod calculadora
```

Como manda a cartilha TDD, vamos começar escrevendo o teste. Crie o arquivo
`calculadora_test.go`.

```go showLineNumbers title="calculadora/calculadora_test.go"
package main

import (
    "testing"
)

func TestAdicao(t *testing.T) {
    resultado := Adicao(2, 3)
    esperado := 5
    if resultado != esperado {
        t.Errorf("Adicao(2, 3) = %d; esperado %d", resultado, esperado)
    }
}
```

Agora que temos nosso teste, vamos rodá-lo:

```bash
go test
```

Sério mesmo, é só isso. Não precisa instalar nada, já está tudo incluso com Go.
Você deve ter visto a seguinte mensagem ao executar o comando `go test`:

```bash
# Tdd-Go [Tdd-Go.test]
./calculadora_test.go:8:15: undefined: Adicao
FAIL	Tdd-Go [build failed]
```

Esse erro é normal, esperado e **desejável**. Sim, TDD **pede** que você comece
falhando testes. O passo que vem a seguir é implementar apenas a definição
daquilo que está sendo testado (só definição **mesmo**, sem implementação):

```go showLineNumbers title="calculadora/calculadora.go"
package main

func Adicao(a int, b int) int {
    return 0;
}
```

Agora, ao rodar o `go test` o output deve ser:

```bash
--- FAIL: TestAdicao (0.00s)
    calculadora_test.go:11: Adicao(2, 3) = 0; esperado 5
FAIL
exit status 1
FAIL	Tdd-Go	0.001s
```

Eu sempre acho que vale a pena ver a versão verbosa do comando, então por
padrão eu rodo `go test -v`. O output para essa etapa deve ser:

```bash
=== RUN   TestAdicao
    calculadora_test.go:11: Adicao(2, 3) = 0; esperado 5
--- FAIL: TestAdicao (0.00s)
FAIL
exit status 1
FAIL	Tdd-Go	0.001s
```

Parece não ter mudança alguma, mas agora o output inclui um detalhe de cada
função testada.

Legal! Agora ao menos o teste reconheceu a existência da função. No entanto,
ela não está implementada de forma correta. Vamos arrumar isso.

```go showLineNumbers title="calculadora/calculadora.go"
package main

func Adicao(a int, b int) int {
    return a + b;
}
```

Agora, nosso comando `go test -v` retorna:

```bash
=== RUN   TestAdicao
--- PASS: TestAdicao (0.00s)
PASS
ok  	Tdd-Go	0.001s
```

Boa! Agora nossa feature foi implementada com sucesso. E agora? Agora
implementamos outra feature **ou** podemos **refatorar** o código com
tranquilidade no coração. Por que? Porque **sabemos** que o nosso teste vai
pegar se a função implementada passar a ser não funcional. Nada paga essa paz
mental!

## 1. Extra: test coverage

Caso queira ver uma porcentagem de código do projeto que está contemplada pelos
seus testes unitários, rode:

```bash
go test -v -cover
```

Output:

```bash
=== RUN   TestAdicao
--- PASS: TestAdicao (0.00s)
PASS
coverage: 100.0% of statements
ok  	Tdd-Go	0.001s
```

## 2. Extra 2: Python vs Go

Você pode ter notado o **quão mais fácil** é usar Go para TDD do que Python. O
motivo pelo qual esse autoestudo é **obrigatório** é para que vocês percebam
isso. Todas as seções de Go no futuro serão **opcionais**. 

Mas mas mas e o `flake8`, `mypy`, `tox` e os outros 43215234 pacotes que
instalei para conseguir desenvolver testes para o Python. Sério mesmo que o `go
test` substitui **TUDO**?

Mais ou menos. Vamos por partes:

* `flake8` é uma ferramenta para garantir estilo de código aderente ao `PEP8`.
  Go tem uma ferramenta que **formata** o seu código para aderir à cartilha de
  estilo da linguage. Essa ferramenta é o `go fmt`. Sugiro **fortemente** que
  configurem o seu editor de texto para rodar `go fmt` quando o arquivo for
  salvo.
* `mypy` é para verificar uso de **type hints**. Go não precisa disso pois se
  não tem o tipo bem definido, o código não compila.
* `tox` é uma ferramenta para testar o código em várias versões de Python
  diferentes. Go **compila** o código com um **target** de arquitetura. O que
  isso significa? Que tanto faz a versão do Go que gerou o executável. Se
  compilou pra sua arquitetura, vai rodar.

  Python é mais fácil que Go para muitas coisas, mas para TDD Go é **MUITO**
  mais fácil.

## 3. Integrando os testes com o Github Actions

Para integrar os testes feitos em Go com o Github Actions você não precisa
fazer nada além de criar um novo arquivo de workflow. Vamos fazer um muito
similar ao que fizemos para Python:

```yaml showLineNumbers title=".github/workflows/test-go-tdd.yaml"
name: Test Go TDD example

on:
  push:
    branches:
      - 'dev'
    paths:
      - 'Exemplos/E02/goTdd/**'

jobs:
  test:
    runs-on: ubuntu-latest

    defaults:
      run:
        shell: bash
        working-directory: './Exemplos/E02/goTdd'

    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-go@v5
        with:
          go-version: 'stable'
      - name: Get dependencies
        run: go mod tidy   
      - name: Test
        run: go test -v --cover .
```

Abaixo um screenshot da minha execução:

<img 
  src="img/workflow-tdd-2.png"
  alt="Workflow de teste com Go" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '70vh',
    marginRight: 'auto'
  }} 
/>
<br/>
