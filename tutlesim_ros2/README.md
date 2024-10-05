# Turtlesim: simulando um ambiente robótico integrado no ROS

## Descrição:
Para implementar o controle de movimento da tartaruga, foi necessário criar um nó que controla seu movimento. Para isso, o ROS2 foi instalado e configurado no sistema operacional POP OS 22.04 e as bibliotecas rclpy e geometry_msgs foram utilizadas para desenvolver um nó chamado "turtle_controller". Esse nó publica mensagens no tópico "turtle1/cmd_vel" e é uma subclasse da classe "Node" da biblioteca rclpy. A função "diamond" controla a sequência e a duração dos movimentos, chamando as funções de movimento correspondentes em intervalos específicos.

A função "draw" inicializa o nó do ROS e o encerra após a conclusão do movimento. Quando o código é executado, as mensagens de movimento são publicadas no tópico "turtle1/cmd_vel" para controlar a tartaruga e a mensagem "The turtle is moving" é exibida na tela para indicar que a tartaruga está em movimento.
Para garantir o sucesso da execução da atividade, é necessário abrir dois terminais no POP OS. O primeiro terminal é responsável por executar o nó "turtlesim". No segundo terminal, o código é executado e, por meio do ROS2, é possível mover a tartaruga no "turtlesim" publicando informações no tópico "turtle1/cmd_vel". É importante destacar que essa é uma comunicação do tipo publisher e subscriber (pub-sub).

Por fim, a tartaruga foi programada para desenhar um losango usando quatro funções que são executadas em um intervalo de tempo determinado.

## Link de demo:
https://www.youtube.com/watch?v=kLFpDWysuiQ

## Estrutura do Projeto:

```
.
├── LICENSE
├── README.md
├── requirements.txt
└── turtlesim_ros2.py

```

PS:. POP OS é a distro do meu notebook.
