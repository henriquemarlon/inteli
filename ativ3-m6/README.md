# Processamento de imagens e detecção de objetos

Para realizar essa atividade, foram empregadas duas fases distintas: o treinamento do modelo utilizando a versão 8 do YOLO (You Only Look Once) e a implementação em tempo real do modelo, fazendo uso da câmera do notebook.

No primeiro estágio, foi utilizado o ambiente do Google Colaboratory para executar o treinamento do modelo. Nesse processo, empregou-se um conjunto de dados específico para treinar o modelo de detecção de rachaduras, fornecido pelo Roboflow. Após a importação e preparação desse conjunto de dados, o treinamento do modelo foi conduzido ao longo de 10 épocas. Ao término desse treinamento, é possível obter um arquivo com a extensão ".pt", que representa o modelo treinado. Tal arquivo armazena os parâmetros adquiridos durante o processo de treinamento, incluindo pesos e configurações específicas do modelo.

No segundo estágio, o arquivo ".pt" foi utilizado para aplicar o modelo treinado e efetuar a detecção em tempo real de rachaduras, utilizando a câmera integrada ao notebook. Com esse propósito, um script em Python foi desenvolvido, o qual permite acessar a câmera do notebook e submeter os quadros capturados à detecção do modelo. Ao processar cada frame obtido pela câmera, o modelo treinado é capaz de identificar a presença de rachaduras e, caso detecte alguma, demarca a região correspondente desenhando um retângulo ao redor da mesma.

Existe aqui uma integração com a arquitetura ROS2 por meio da qual o backend contido na pasta "backend" é subscriber de um tópico em que, no caso de uma implementação futura, um robô publica imagens. Antes de rederizar o output das imagens, todos os arquivos passam pela análise por parte do modelo pré treinado e assim nós conseguimos mostrar na tela a os interações com as rachaduras.

## Intruções para rodar o projeto:

- Abra dois terminais
- Em um deles entre na pasta:
```cd backend```
- Rode o controller do subscriber do tópico /camera:
```python backend_controller.py```


- Em outro entre na pasta:
```cd embedded```
- Rode o controller do publisher do tópico /camera:
```python embedded_controller.py```

 Video: https://drive.google.com/file/d/1Azm9uZ-otGJPdJZaab5iYAsOXD-JIfz0/view
