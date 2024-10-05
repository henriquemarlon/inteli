# Desafio dos 1 Bilhão de Linhas em Zig

O Desafio dos 1 Bilhão de Linhas (1BRC) está disponível em: https://github.com/gunnarmorling/1brc/.

> [!NOTE]
> O Desafio dos 1 Bilhão de Linhas (1BRC) é uma exploração divertida de até onde o Java moderno pode ser levado para agregar um bilhão de linhas de um arquivo de texto. Reúna todos os seus (virtuais) threads, explore o SIMD, otimize seu GC ou use qualquer outro truque e crie a implementação mais rápida para resolver essa tarefa!

## Como Executar

- Baixe o seu executável do Zig [aqui!](https://ziglang.org/download/).
     - Estou usando a versão `0.11.0`.

- Dentro do repositório, execute `zig build -Doptimize=ReleaseFast` para compilar.

- Se ainda não tiver feito, utilize o executável `run-create-sample` para criar um arquivo TXT com um bilhão de linhas **(~12GB)**: `./zig-out/bin/run-create-sample 1000000000` *(Isso deve levar de alguns minutos)*.

- Feito isso! Agora execute `time ./zig-out/bin/1brc-zig measurements.txt` e veja o tempo!

## Resultados de Desempenho

Acer Nitro AN515-44, AMD® Ryzen 7 4800h with radeon graphics × 16 , NVIDIA Corporation TU117M / NVIDIA GeForce GTX 1650/PCIe/SSE2 (PopOS): **1.71 seconds**
```
49.60s user 1.71s system 1244% cpu 4.124 total
```