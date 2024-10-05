[![Deploy to GitHub Pages](https://github.com/rmnicola/m9-ec-encontros/actions/workflows/deploy.yaml/badge.svg)](https://github.com/rmnicola/m9-ec-encontros/actions/workflows/deploy.yaml)
[![Test Python TDD example](https://github.com/rmnicola/m9-ec-encontros/actions/workflows/test-python-tdd.yaml/badge.svg)](https://github.com/rmnicola/m9-ec-encontros/actions/workflows/test-python-tdd.yaml)
[![Test Go TDD example](https://github.com/rmnicola/m9-ec-encontros/actions/workflows/test-go-tdd.yaml/badge.svg)](https://github.com/rmnicola/m9-ec-encontros/actions/workflows/test-go-tdd.yaml)

# Módulo de infraestrutura de cidades inteligentes

Este é o repositório que abriga o material da área de computação do módulo 9 do
curso de Engenharia de Computação do Inteli, cujo tema é Hiperconectividade de
cidades inteligentes. O objetivo do módulo é a introdução do conceito de
sistemas de alta escalabilidade no contexto de IoT, utilizando o protocolo MQTT
e ferramentas para processamento de eventos complexos (como o Kafka) para que
seja possível criar um sistema capaz de lidar com as demandas de inúmeros
dipositivos de borda. Esse módulo não aborda a perspectiva do dispositivo IoT
e, portanto, tem pouca exposição à hardware e tecnologias de baixo nível.

Este material foi feito utilizando o [docusaurus](https://docusaurus.io/), com
a página gerada hospedada no Github pages. Para acessar o material renderizado,
acesse [esse link](https://rmnicola.github.io/m9-ec-encontros/).

## Estrutura de pastas

Este repositório é relativamente simples e conta apenas com dois diretórios:
`Exemplos` e `Material`. O diretório `Material` é a pasta raíz do docusaurus,
enquanto o diretório `Exemplos` conta com o código fonte dos exemplos
desenvolvidos no decorrer do curso para demonstrar as tecnologias e conceitos
aplicados no módulo.

## Instalando e rodando a documentação localmente

> [!IMPORTANT]  
> As instruções a seguir assumem que você possuí `npm >= 16` instalado e
> devidamente configurado em seu sistema. A princípio, as instruções são
> agnósticas a sistema operacional.

Para instalar e rodar localmente uma documentação feita com o docusaurus, basta
seguir as instruções delineadas pela [documentação do
docusaurus](https://docusaurus.io/docs/installation#running-the-development-server),
o que consiste em:

1. Para rodar o projeto localmente utilizando o `npm`, navegue até a pasta raíz
   do docusaurus e rode:

```bash
npm run start
```

Esta forma de execução é primariamente utilizada para desenvolvimento de
conteúdo, pois ela conta com atualizações instantâneas à pagina local sempre
que os arquivos de conteúdo forem alterados.

2. Para criar artefatos de distribuição do site estático, rode:

```bash
npm run build
```

Esta forma deve ser utilizada somente para o deploy da página estática
(atualmente é utilizada no workflow do github pages)

## Considerações de uso

Ver [descrição da licença](./LICENSE).

## Como contribuir

Quer ajudar a construir e/ou melhorar o material? Fico feliz s2! Para que não
haja atrito na adição de suas contribuições, considere as seguintes instruções:

1. Antes de mais nada, abra um issue. Notou um erro de português, algum
   tutorial não funciona, alguma abstração está difícil de entender? Expresse o
   problema de forma clara em um issue nesse repositório. Essa é a forma mais
   rápida e de menor atrito para iniciar uma discussão sobre o problema em
   questão.

2. Issue aberto, mas você quer resolver a parada sozinho? Deus te abençõe. Para
   ter sua contribuição oficialmente como parte desse projeto, você deve:

   * Criar um fork deste repositório
   * Crie um branch para suas edições
   * Referencie issues em seus commits
   * Abra um PR para este repositório que fecha a issue em questão
   * Após passar em todos os testes e passar por uma revisão, o seu PR será
     integrado ao material. Muito obrigado! =)




