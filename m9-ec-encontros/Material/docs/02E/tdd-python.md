---
title: Testes em Python
sidebar_position: 3
sidebar_class_name: autoestudo
slug: /tdd-python
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Implementação de testes em Python

Para seguir este tutorial, sugere-se **fortemente** o uso de **virtual
environment**. Para isso, rode:

```bash
python3 virtualenv venv
```

Para ativar o **venv**, use:

```bash
source venv/bin/activate
```

:::tip

Se o seu **venv** estiver dentro do seu repositório git, garanta que vai
atualizar o .gitignore para não tentar fazer upload de pastas gigantescas no
github.

:::


## 1. Criando um pacote Python

<img 
  src="https://dotmethod.me/posts/poetry-python-package-manager/cover.png"
  alt="Camadas do modelo OSI" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '40vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Antes de trabalharmos com testes unitários em Python, vamos precisar criar um
pacote em Python com o nosso código. O problema? Python é uma **bagunça** no
que diz respeito ao processo de criar um pacote. Por isso eu não vou explicar
tanto os passos envolvidos no processo (honestamente, eu fiz por tentativa e
erro e google e chatGPT pois não tenho disposição para desvendar os
anti-patterns que inventaram para o sistema de empacotamento de Python)

Vamos começar pela estrutura de diretório do seu pacote:

```bash
.
├── __init__.py
├── pyproject.toml
├── pyTdd
│   ├── Calculator
│   │   ├── __init__.py
│   │   └── Operations.py
│   └── __init__.py
├── setup.cfg
├── setup.py
├── tests
│   └── test_calculator.py
└── tox.ini
```

Todo o nosso **código fonte** do pacote fica dentro do diretório `pyTdd`, que é o
nome do pacote. Nossos **testes** ficam no diretório `tests`. Todos os
diretórios dentro de `pyTdd` são **submódulos** dentro de nosso pacote. No
caso, temos um submódulo chamado **Calculator**. Os arquivos `pyproject.toml`,
`setup.cfg` e `setup.py` são os arquivos que configuram o processo de **build**
do pacote. Sim, são **TRÊS** arquivos diferentes. Se você ainda não tinha
encontrado o ponto fraco do Python; está aí. Não vou entrar em detalhes do
motivo do que está em `pyproject.toml` (não sei quem é tom, mas sei que ele não
deveria ter criado uma linguagem) e nem do `setup.py`. O único arquivo que
realmente importa para a gente é o `setup.cfg`. Então segue para você só copiar
e colar os dois primeiros:

<Tabs>
  <TabItem value="setup" label="setup.py" default>
  ```python showLineNumbers title="setup.py"
from setuptools import setup


if __name__ == '__main__':
    setup()
  ```
  </TabItem>
  <TabItem value="pyproject" label="pyproject.toml">
  ```python showLineNumbers title="pyproject.toml"
[build-system]
requires = ["setuptools"]
build-backend = "setuptools.build_meta"
  ```
  </TabItem>
</Tabs>

Agora vamos para o `setup.cfg`:

```python showLineNumbers title="setup.cfg"
[metadata]
name = pyTdd
version = 1.0
author = Rodrigo Nicola
author_email = nicola@prof.inteli.edu.br
description = Teste TDD
long_description = file: README.md
long_description_content_type = text/markdown
url = https://inteli.edu
classifiers =
    Programming Language :: Python :: 3
    Operating System :: OS Independent
license_files = LICENSE.txt


[options]
zip_safe = False
include_package_data = True
packages = find:
python_requires = >=3.8
install_requires =
    requests
    importlib-metadata
    pydantic

[options.packages.find]
exclude = 
    tests

[options.extras_require]
test =
    pytest
    pytest-cov
    mypy
    flake8
    tox
```

A seção **[metadata]** é um tanto autoexplicativa, então vamos pulá-la. A seção
**options** e suas subseções aqui são as que importam para a gente. Dentro
delas, se destacam: 
* **python_requires** - define a versão mínima de Python compatível com o
  seu pacote
* **install_requires** - define os pacotes dos quais o seu pacote depende
  (serve o mesmo propósito do `requirements.txt`, só que aqui ele garante que
  os pacotes serão instalados quando dermos `pip install pyTdd`)
* **exclude** - define quais diretórios **não** serão buscados para encontrar
  código em Python pertencente ao pacote.
* **extra_requires** - define dependências extra para outras tarefas
  relacionadas ao seu projeto. No caso, estamos definindo uma série de pacotes
  necessários para poder fazer testes unitários, de cobertura, de estilo, de
  tipagem e de compatibilidade em Python. Sim, são **CINCO** ferramentas
  diferentes.

 Vamos agora criar um arquivo que implementa um classe de **adição** dentro do
 submodulo **Calculator**. Para isso, vamos criar o arquivo `Operations.py`: 

 ```python showLineNumbers title="pyTdd/Calculator/Operations.py"
from pydantic import BaseModel


class Adder(BaseModel):
    """
    Classe de teste para criar a operação de adição
    """

    def Add(self, *args: int) -> float:
        return sum(args)
```

Legal, agora vamos instalar nosso pacote. Para isso, vamos usar o `pip`:

```bash
pip install -e .
```

O `-e` serve para criar uma instalação com **link simbólico**, o que significa
que não é necessário **reinstalar** o pacote para testar cada uma das
alterações.

Agora, para testar se o pacote foi instalado e faz o que deveria fazer, vamos
rodar o seguinte código em Python:

```python showLineNumbers title="teste.py"
from pyTdd.Calculator.Operations import Adder

ad = Adder()

print(ad.Add(1, 2, 3))
```

Mas, pera, não iamos criar testes automatizados? Pois é, então deletem esse
arquivo `teste.py` pra poder seguir do **jeito certo**.

## 2. Testes com Python

Eu comentei ali em cima que vamos usar não um, nem dois, nem três, mas
**CINCO** ferramentas para testar nosso código. São elas:

1. Pytest - ferramentas para criação de testes unitários em Python;
2. Pytest-cov - plugin para verificar **cobertura** de testes unitários;
3. flake8 - ferramenta que verifica a aderência do seu projeto ao **PEP8**;
4. mypy - caso você use **type hints** (deveria), o mypy serve para verificar
   se não faltou nenhum type hint;
5. tox - com essa ferramenta, é possível automatizar o uso de todas as outras,
   inclusive utilizando diversas versões de python;

Sim, é muita coisa só para adicionar testes no seu software. Python é patético
nesse aspecto. Mas chega de reclamar, vamos configurar nossos testes. A boa
notícia é que na seção anterior eu já coloquei todas as dependências de teste
no `setup.cfg`. O que isso significa? Que podemos instalar todas essas
dependências usando:

```bash
pip install -e ".[test]"
```

Para o `mypy` e o `flake8` não há a necessidade de configurações extra, basta
rodar:

```bash
mypy pyTdd
```

e 

```
flake8 pyTdd tests
```

Para o `pytest` precisamos de testes unitários definidos. Para isso, vamos
criar um arquivo chamado `test_calculator.py` no diretório `tests`.

:::warning

Todos os arquivos que contem testes unitários para serem executados pelo
`pytest` devem ter `test_` como prefixo em seu nome. É usando esse padrão que o
`pytest` encontra os arquivos que deve executar.

:::

```python showLineNumbers title="tests/test_calculator.py"
from pyTdd.Calculator import Operations


def test_adder():
    adder = Operations.Adder()
    assert adder.Add(1, 2, 3) == 6
```

Agora que temos um teste pronto, podemos rodar o `pytest`.

```bash
pytest
```

E agora que temos o `pytest`, o `mypy` e o `flake8` configurados, podemos
configurar o `tox` usando o arquivo `tox.ini`:

```ini showLineNumbers title="tox.ini"
[tox]
envlist = 
    py310
    style
    type

[testenv]
deps = pytest
commands =
    pytest

[testenv:style]
deps = flake8
commands = flake8 pyTdd tests

[testenv:type]
deps = mypy
commands = mypy pyTdd
```

Notem que o `tox` cria **ambientes virtuais** para executar cada um dos testes.
No caso, estamos usando três ambientes diferentes:
1. py310 - ambiente que se configura automaticamente pelo `tox` com o Python
   3.10. Aqui é onde usaremos o `pytest`.
2. style - ambiente criado e configurado para instalar e rodar o `flake8`.
2. type - ambiente criado e configurado para instalar e rodar o `mypy`.

Para rodar o tox, basta executá-lo na raíz do projeto:

```bash
tox
```

Como tratam-se de 3 ambientes diferentes, podemos pedir para o `tox` rodá-los
em **paralelo**:

```bash
tox run-parallel
```

### Testando com múltiplas versões de Python

Embora o `tox` te permita especificar quais versões de Python vai usar para
fazer seus testes, ele assume que você já instalou mais de uma versão de Python
em seu sistema. Sendo assim, precisaremos instalar versões antigas de Python.

No **Ubuntu**, só podemos instalar se adicionarmos um repositório novo ao `apt`
chamado **dead snakes**. 

```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
```

Agora, podemos instalar o **Python 3.9** para testar a funcionalidade do `tox`:

```bash
sudo apt install python3.9 python3.9-distutils
```

Agora, precisamos atualizar nosso arquivo `tox.ini`:

```ini showLineNumbers title="tox.ini"
[tox]
envlist = 
    py39
    py310
    style
    type

[testenv]
deps = pytest
commands =
    pytest

[testenv:style]
deps = flake8
commands = flake8 pyTdd tests

[testenv:type]
deps = mypy
commands = mypy pyTdd
```

Agora, ao rodarmos o `tox` vamos testar, também, a versão 3.9 do Python. Repita
esse procedimento para todas as versões que estão em seu **target** de release.
Depois de todo esse trabalho, saiba que você não precisa mais nem ativar sua
**venv**. Basta rodar o `tox` na raíz do seu projeto que ele testa tudo.

<img 
  src="https://media1.tenor.com/m/HhR0mx8HrXsAAAAd/morel-hunter-x-hunter.gif"
  alt="Camadas do modelo OSI" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '40vh',
    marginRight: 'auto'
  }} 
/>
<br/>


## 3. Integrando os testes com o Github Actions

Agora que já fizemos a parte mais difícil, podemos criar um workflow no Github
para rodar o `tox` automaticamente e testar nosso código de acordo com algum
trigger que escolhermos. Para isso, primeiro precisamos adicionar ao `tox.ini`
a configuração de **mais uma** ferramenta: o
[tox-gh-actions](https://github.com/ymyzk/tox-gh-actions).

```ini showLineNumbers title="tox.ini"
[tox]
envlist = 
    py39
    py310
    style
    type

[gh-actions]
python =
    3.9: py39
    3.10: py310, mypy, flake8

[testenv]
deps = pytest
commands =
    pytest

[testenv:style]
deps = flake8
commands = flake8 pyTdd tests

[testenv:type]
deps = mypy
commands = mypy pyTdd
```

A seguir, vamos criar um arquivo de **workflow** do Github Actions em
`.github/workflows`.

```yaml showLineNumbers title=".github/workflows/test-python-tdd.yaml"
name: Test Python TDD example

on:
  push:
    branches:
      - 'dev'
    paths:
      - 'Exemplos/E02/pyTdd/**'

jobs:
  test:
    runs-on: ubuntu-latest
    strategy: 
      matrix:
        python-version: ['3.9', '3.10']

    defaults:
      run:
        shell: bash
        working-directory: './Exemplos/E02/pyTdd'

    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - name: Set up Python ${{ matrix.python-version }}
        uses: actions/setup-python@v4
        with:
          python-version: ${{ matrix.python-version }}
      - name: Install dependencies
        run: |
          python -m pip install --upgrade pip
          python -m pip install tox tox-gh-actions
      - name: Test with tox
        run: tox
```

Vamos observar as novidades desse arquivo com relação ao último tutorial:

* chave **paths** - com essa chave, podemos restringir o gatilho desse workflow
  para apenas quando arquivos dentro do diretório `Exemplos/E02/pyTdd` são
  modificados. Essa chave pode ser utilizada em conjunto com a chave
  **branches**. Isso significa que, mesmo que haja mudanças no diretório
  especificado, ainda precisa ser um `push` com target para a branch `dev`.
* estratégia **matrix** - aqui podemos especificar diversas versões do Python
  para rodar com os passos do workflow.

:::note

É possível usar a estratégia **matrix** também com sistemas operacionais. Fica
de lição de casa caso queira testar o seu pacote não só em várias versões de
Python, como também para vários sistemas operacionais (esteira CI/CD ficando
pro, hein?)

:::

E pronto! Agora, basta fazer o `push` e observar os testes sendo executados
automaticamente. Abaixo um screenshot da minha execução após escrever esse
tutorial:

<img 
  src="img/workflow-tdd-1.png"
  alt="Workflow de teste com Python" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '70vh',
    marginRight: 'auto'
  }} 
/>
<br/>
