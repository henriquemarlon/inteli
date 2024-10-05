# robot-simulation

### Estrutura do projeto:

```
├── godot
│   ├── alvo2.png
│   ├── alvo2.png.import
│   ├── alvo.png
│   ├── alvo.png.import
│   ├── braço.png
│   ├── braço.png.import
│   ├── corpo.png
│   ├── corpo.png.import
│   ├── default_env.tres
│   ├── icon.png
│   ├── icon.png.import
│   ├── KinematicBody2D.gd
│   ├── project.godot
│   └── simulation.tscn
├── LICENSE
├── README.md
├── requirements.txt
└── src
    ├── app.db
    ├── app.py
    ├── config
    │   ├── db.py
    │   ├── __init__.py
    │   └── __pycache__
    │       ├── db.cpython-311.pyc
    │       └── __init__.cpython-311.pyc
    ├── controllers
    │   ├── coordinates_controller.py
    │   ├── __init__.py
    │   └── __pycache__
    │       ├── coordinates_controller.cpython-311.pyc
    │       └── __init__.cpython-311.pyc
    ├── main.py
    ├── models
    │   ├── base.py
    │   ├── coordinates.py
    │   ├── __init__.py
    │   └── __pycache__
    │       ├── base.cpython-311.pyc
    │       ├── coordinates.cpython-311.pyc
    │       └── __init__.cpython-311.pyc
    ├── __pycache__
    │   └── app.cpython-311.pyc
    └── templates
        ├── index.html
        └── __init__.py
 ```

### Descrição do projeto:

- O projeto consiste em uma simulação de um braço robô na engine dobot utilizando um backend em flask e sqlalchemy.

### Para rodar esse projeto você precisa rodar os seguintes códigos:

1º Entre na pasta em que está o backend do projeto ```cd src```

2º Intale todas as bibliotecas necessárias ```pip install -r requirements.txt```

3º Dê "start" no projeto ```flask run```

4º Abra a engine GODOT

5º Dentro do dashboard do GODOT entre na pasta godot do projeto e de start

6º O seu navegador e entre na seguinte rota: ```http://127.0.0.1:5000/coordinates```

7º Interaja com o Robô submetendo posições x, y, z e r no formulário.
