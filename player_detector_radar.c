#include <SDL2/SDL.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define THRESHOLD 30

static void usage(const char *program)
{
    fprintf(stderr,
            "Uso: %s <arquivo.yuv> <largura> <altura> [fps]\n"
            "Exemplo: %s video_4k.yuv 3840 2160 30\n",
            program, program);
}

static int parse_positive_int(const char *text, const char *name)
{
    char *end = NULL;
    errno = 0;
    long value = strtol(text, &end, 10);

    if (errno != 0 || end == text || *end != '\0' || value <= 0 || value > INT32_MAX) {
        fprintf(stderr, "%s invalido: %s\n", name, text);
        return -1;
    }

    return (int)value;
}

int main(int argc, char **argv)
{
    if (argc < 4 || argc > 5) {
        usage(argv[0]);
        return 1;
    }

    const char *path = argv[1];
    int width = parse_positive_int(argv[2], "largura");
    int height = parse_positive_int(argv[3], "altura");
    int fps = argc == 5 ? parse_positive_int(argv[4], "fps") : 30;

    if (width <= 0 || height <= 0 || fps <= 0) {
        return 1;
    }

    if ((width % 2) != 0 || (height % 2) != 0) {
        fprintf(stderr, "yuv420p exige largura e altura pares.\n");
        return 1;
    }

    size_t y_size = (size_t)width * (size_t)height;
    size_t uv_size = y_size / 4;
    size_t frame_size = y_size + (2 * uv_size);
    
	//printf("frame size: %zu\n", frame_size);

    uint8_t *frame = malloc(frame_size);
    if (!frame) {
        fprintf(stderr, "Falha ao alocar %zu bytes para o frame.\n", frame_size);
        return 1;
    }

    // ALOCA PREV_Y_PLANE 
    uint8_t *prev_y_plane = calloc(1, y_size); // UTILIZA CALLOC POIS INICIA COM ZERO
    if (!prev_y_plane) {
        fprintf(stderr, "Falha ao alocar %zu bytes para o plano y anterior.\n", y_size);
        return 1;
    }

    // ALOCA MASCARA PARA FILTRAGEM
    uint8_t *mask = calloc(1, y_size); 
    if (!mask) {
        fprintf(stderr, "Falha ao alocar %zu bytes para a mascara.\n", y_size);
        return 1;
    }

    FILE *file = fopen(path, "rb");
    if (!file) {
        fprintf(stderr, "Falha ao abrir %s: %s\n", path, strerror(errno));
        free(frame);
        free(prev_y_plane);
        free(mask);
        return 1;
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        fprintf(stderr, "SDL_Init falhou: %s\n", SDL_GetError());
        fclose(file);
        free(frame);
        free(prev_y_plane);
        free(mask);
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow("YUV420P 4K Player",
                                          SDL_WINDOWPOS_CENTERED,
                                          SDL_WINDOWPOS_CENTERED,
                                          width,
                                          height,
                                          SDL_WINDOW_RESIZABLE);
    if (!window) {
        fprintf(stderr, "SDL_CreateWindow falhou: %s\n", SDL_GetError());
        SDL_Quit();
        fclose(file);
        free(frame);
        free(prev_y_plane);
        free(mask);
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        fprintf(stderr, "SDL_CreateRenderer falhou: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        fclose(file);
        free(frame);
        free(prev_y_plane);
        free(mask);
        return 1;
    }

    SDL_Texture *texture = SDL_CreateTexture(renderer,
                                             SDL_PIXELFORMAT_IYUV,
                                             SDL_TEXTUREACCESS_STREAMING,
                                             width,
                                             height);
    if (!texture) {
        fprintf(stderr, "SDL_CreateTexture falhou: %s\n", SDL_GetError());
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        fclose(file);
        free(frame);
        free(prev_y_plane);
        free(mask);
        return 1;
    }

    const uint32_t frame_delay_ms = 1000u / (uint32_t)fps;
    int running = 1;
    int paused = 0;

    int delta[4] = {-1, 1, -width, width};

    while (running) {
        uint32_t started_ms = SDL_GetTicks();

        SDL_Event event;
        // Leitura de eventos (quit, pause)
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = 0;
            } else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE || event.key.keysym.sym == SDLK_q) {
                    running = 0;
                } else if (event.key.keysym.sym == SDLK_SPACE) {
                    paused = !paused;
                }
            }
        }

        if (!running) {
            break;
        }

        if (!paused) {
        	//aqui é feita a leitura de um frame do arquivo binário
            size_t read_bytes = fread(frame, 1, frame_size, file);
            if (read_bytes != frame_size) {
                // Rewind para retornar ao primeiro frame caso o arquivo acabe
                if (feof(file)) {
                    rewind(file);
                    read_bytes = fread(frame, 1, frame_size, file);
                }

                if (read_bytes != frame_size) {
                    fprintf(stderr, "Frame incompleto ou erro de leitura.\n");
                    break;
                }
            }

            uint8_t *y_plane = frame; // Começa na posicao 0
            const uint8_t *u_plane = frame + y_size; // Vem logo depois
            const uint8_t *v_plane = frame + y_size + uv_size; // Vem depois dos planos y e u
            
            if (prev_y_plane) {
                for (int i = 0; i < y_size; i++) {
                    uint8_t original_pix = y_plane[i];

                    // DIFERENÇA ENTRE OS PIXEIS
                    int diff = abs((int)y_plane[i] - (int)prev_y_plane[i]);

                    prev_y_plane[i] = original_pix;

                    if (diff >= THRESHOLD) {
                        mask[i] = 1;
                    } else {
                        mask[i] = 0;
                    }
                }
            }

            // VERIFICA MASK SEM OUT OF BOUNDS
            for (int i = width; i < y_size - width; i++) {
                if (mask[i]) {
                    int n_mov = 0; // QUANTIDADE DE VIZINHOS EM MOVIMENTO

                    for (int k = 0; k < 4; k++) {
                        if (mask[i + delta[k]]) {
                            n_mov++;
                        }
                    }

                    if (n_mov < 2) {
                        mask[i] = 0;
                    }
                }
            }

            for (int i = 0; i < y_size; i++) {
                if (mask[i]) {
                    y_plane[i] = 255;
                } else {
                    y_plane[i] /= 2;
                }
            }
			
            //aqui será feita a exibição do frame na textura SDL
            if (SDL_UpdateYUVTexture(texture,
                                     NULL,
                                     y_plane,
                                     width,
                                     u_plane,
                                     width / 2,
                                     v_plane,
                                     width / 2) != 0) {
                fprintf(stderr, "SDL_UpdateYUVTexture falhou: %s\n", SDL_GetError());
                break;
            }
        }

        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);

        uint32_t elapsed_ms = SDL_GetTicks() - started_ms;
        if (elapsed_ms < frame_delay_ms) {
            SDL_Delay(frame_delay_ms - elapsed_ms);
        }
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    fclose(file);
    free(frame);
    free(prev_y_plane);
    free(mask);

    return 0;
}
