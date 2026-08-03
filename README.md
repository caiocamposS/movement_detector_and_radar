# movement_detector_and_radar

# 🎥 Detecção de Movimento em Vídeos Raw (YUV420p)

Este projeto consiste em um **player de vídeo desenvolvido em C** utilizando a biblioteca **SDL2**, capaz de reproduzir arquivos **Raw YUV420p** e realizar **detecção de movimento em tempo real**.

A detecção é baseada na comparação da **luminosidade (Plano Y)** entre quadros consecutivos. Quando a diferença de brilho ultrapassa um determinado **limiar (threshold)**, o pixel é considerado em movimento e destacado em branco, enquanto o restante da imagem é escurecido para evidenciar a movimentação.

---

## 🚀 Funcionalidades

* 📹 Reprodução de vídeos no formato **Raw YUV420p**;
* ⚡ Processamento em tempo real de vídeos de alta resolução (incluindo **4K**);
* 🎯 Detecção de movimento baseada na diferença absoluta de brilho (Plano Y);
* 🌑 Escurecimento automático do fundo (pixels estáticos);
* ⏯️ Controles de reprodução (pausar e retomar).

---

## 🛠️ Pré-requisitos

Para compilar e executar o projeto, é necessário:

* Um compilador C (ex.: **GCC**);
* Biblioteca de desenvolvimento **SDL2**.

### Ubuntu / Debian

```bash
sudo apt update
sudo apt install build-essential libsdl2-dev
```

---

## ⚙️ Compilação

Compile o projeto utilizando o **GCC**.

> **Importante:** a flag `-lSDL2` deve ser incluída ao final do comando para realizar corretamente a etapa de linkagem da biblioteca.

```bash
gcc yuv420p_player_sdl_3.c -o player -lSDL2
```

Após a compilação, será criado um executável chamado:

```text
player
```

---

## ▶️ Como Executar

A sintaxe do programa é:

```bash
./player <arquivo.yuv> <largura> <altura> [fps]
```

### Parâmetros

| Parâmetro          | Descrição                                        |
| ------------------ | ------------------------------------------------ |
| `arquivo.yuv`      | Caminho para o vídeo no formato Raw YUV420p      |
| `largura`          | Largura do vídeo em pixels                       |
| `altura`           | Altura do vídeo em pixels                        |
| `fps` *(opcional)* | Taxa de quadros por segundo (padrão: **30 FPS**) |

> **Observação:** arquivos **YUV Raw** não possuem cabeçalho (header), portanto o programa não consegue identificar automaticamente a resolução nem a taxa de quadros. Essas informações devem ser fornecidas pelo usuário.

---

## 🌟 Exemplo de Execução

Para reproduzir um vídeo chamado `video_4k.yuv`, gravado em resolução **3840×2160 (4K UHD)** a **30 FPS**:

```bash
./player video_4k.yuv 3840 2160 30
```

Nesse comando:

* `./player` → executa o programa;
* `video_4k.yuv` → arquivo de vídeo;
* `3840` → largura;
* `2160` → altura;
* `30` → taxa de quadros por segundo (opcional).

---

## 🎮 Controles

Durante a reprodução, as seguintes teclas estão disponíveis:

| Tecla            | Ação                         |
| ---------------- | ---------------------------- |
| **Espaço**       | Pausa ou retoma a reprodução |
| **Q** ou **ESC** | Encerra o player             |

---

## 🧠 Funcionamento do Algoritmo

O formato **YUV420p** separa a imagem em três componentes:

* **Y (Luma):** brilho;
* **U (Chroma):** componente de cor;
* **V (Chroma):** componente de cor.

O algoritmo utiliza apenas o **Plano Y (luminosidade)** para detectar movimento.

O processamento ocorre da seguinte forma:

1. O quadro atual é carregado na memória;
2. Cada pixel do Plano **Y** é comparado com o pixel correspondente do quadro anterior;
3. É calculada a diferença absoluta entre os valores de brilho:

```c
abs(atual - anterior)
```

4. Se essa diferença for maior ou igual ao valor de **THRESHOLD** (definido como `30`), o pixel é considerado em movimento e recebe valor máximo de brilho (`255`), aparecendo branco na tela;
5. Caso contrário, o pixel é tratado como parte do fundo e seu brilho é reduzido pela metade, tornando-o mais escuro;
6. Ao final do processamento, o quadro atual é armazenado como referência para a próxima comparação e a imagem processada é exibida.

---

## 📌 Resumo do Fluxo

```text
Leitura do quadro atual
          │
          ▼
Comparação com o quadro anterior
          │
          ▼
Diferença de brilho (Plano Y)
          │
     ┌────┴────┐
     │         │
Diferença   Diferença
≥ Threshold < Threshold
     │         │
     ▼         ▼
Pixel branco  Fundo escurecido
     │
     ▼
Exibição do quadro processado
```
