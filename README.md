# 🎥 Detecção de Movimento e Velocidade em Vídeos Raw (YUV420p)

Este projeto consiste em um **player de vídeo desenvolvido em C** utilizando a biblioteca **SDL2**, capaz de reproduzir arquivos **Raw YUV420p** e realizar **detecção de movimento em tempo real, bem como a identificação da velocidade desses movimentos**.

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
