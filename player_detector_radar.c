#include <SDL2/SDL.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define THRESHOLD 25  // Limiar de diferença de pixel para detectar movimento

typedef struct { double x; double y;} Point2D;  // Estrutura para ponto 2D com coordenadas reais

const Point2D real_points[4] = {{0.0, 0.0}, {100.0, 0.0}, {100.0, 10.0}, {0.0, 10.0 }};  // Pontos reais de calibração (100cm x 10cm)

double H[9];  // Matriz de homografia 3x3
int homography_ready = 0;  // Flag indicando se a homografia foi calculada

// Resolve sistema linear 8x8 por eliminação de Gauss
static int solve_8x8(double A[8][8], double b[8], double x[8]){
    int i, j, k, piv;
    double maxv, tmp, factor;

    for (i = 0; i < 8; i++){  // Loop principal da eliminação
        piv = i;
        maxv = fabs(A[i][i]);
        for (k = i + 1; k < 8; k++){  // Busca o maior pivô
            if (fabs(A[k][i]) > maxv) {
                maxv = fabs(A[k][i]);
                piv = k;
            }
        }

        if (maxv < 1e-12)  // Matriz singular
            return -1;

        if (piv != i) {  // Troca de linhas
            for (j = 0; j < 8; j++){
                tmp = A[i][j];
                A[i][j] = A[piv][j];
                A[piv][j] = tmp;
            }

            tmp = b[i];
            b[i] = b[piv];
            b[piv] = tmp;
        }

        for (k = i + 1; k < 8; k++){  // Eliminação gaussiana
            factor = A[k][i] / A[i][i];

            for (j = i; j < 8; j++){
                A[k][j] -= factor * A[i][j];
            }

            b[k] -= factor * b[i];
        }
    }

    for (i = 7; i >= 0; i--){  // Substituição regressiva
        tmp = b[i];

        for (j = i + 1; j < 8; j++){
            tmp -= A[i][j] * x[j];
        }

        x[i] = tmp / A[i][i];
    }

    return 0;
}

// Calcula a matriz de homografia a partir de 4 correspondências
static int compute_homography(const SDL_Point src[4], const Point2D  dst[4], double H_out[9]){
    double A[8][8];
    double b[8];
    double h[8];
    int i;

    memset(A, 0, sizeof(A));  // Zera a matriz A

    for (i = 0; i < 4; i++){  // Monta as 8 equações do DLT
        double xi = (double)src[i].x;
        double yi = (double)src[i].y;
        double ui = dst[i].x;
        double vi = dst[i].y;

        A[2*i][0] = xi;
        A[2*i][1] = yi;
        A[2*i][2] = 1.0;
        A[2*i][3] = 0.0;
        A[2*i][4] = 0.0;
        A[2*i][5] = 0.0;
        A[2*i][6] = -ui * xi;
        A[2*i][7] = -ui * yi;
        b[2*i] = ui;

        A[2*i+1][0] = 0.0;
        A[2*i+1][1] = 0.0;
        A[2*i+1][2] = 0.0;
        A[2*i+1][3] = xi;
        A[2*i+1][4] = yi;
        A[2*i+1][5] = 1.0;
        A[2*i+1][6] = -vi * xi;
        A[2*i+1][7] = -vi * yi;
        b[2*i+1] = vi;
    }

    if (solve_8x8(A, b, h) != 0)  // Resolve o sistema
        return -1;

    H_out[0] = h[0]; H_out[1] = h[1]; H_out[2] = h[2];  // Preenche a matriz H
    H_out[3] = h[3]; H_out[4] = h[4]; H_out[5] = h[5];
    H_out[6] = h[6]; H_out[7] = h[7]; H_out[8] = 1.0;

    return 0;
}

// Converte coordenada de pixel para coordenada real (cm)
static void transform_point(const double H_mat[9], double px, double py, double *rx, double *ry){
    double X = H_mat[0]*px + H_mat[1]*py + H_mat[2];
    double Y = H_mat[3]*px + H_mat[4]*py + H_mat[5];
    double W = H_mat[6]*px + H_mat[7]*py + H_mat[8];

    if (fabs(W) < 1e-12) {  // Evita divisão por zero
        *rx = 0.0;
        *ry = 0.0;
        return;
    }

    *rx = X / W;
    *ry = Y / W;
}

// Exibe mensagem de uso do programa
static void usage(const char *program){
    fprintf(stderr, "Uso: %s <arquivo.yuv> <largura> <altura> [fps]\n" "Exemplo: %s video_4k.yuv 3840 2160 30\n", program, program);
}

// Converte string para inteiro positivo válido
static int parse_positive_int(const char *text, const char *name){
    char *end = NULL;
    errno = 0;

    long value = strtol(text, &end, 10);

    if (errno != 0 || end == text || *end != '\0' || value <= 0 || value > INT32_MAX){
        fprintf(stderr, "%s invalido: %s\n", name, text);
        return -1;
    }

    return (int)value;
}

// Calcula de que lado da linha um ponto está
static double line_side( SDL_Point line_start, SDL_Point line_end, double x, double y){
        return (double) (line_end.x - line_start.x) * (y - line_start.y) - (double)(line_end.y - line_start.y) * (x - line_start.x);
}

// Verifica se o ponto cruzou a linha entre dois frames
static int crossed_line(SDL_Point line_start, SDL_Point line_end, double previous_x, double previous_y, double current_x, double current_y){
    double previous_side = line_side(line_start, line_end, previous_x, previous_y);

    double current_side = line_side(line_start, line_end, current_x, current_y);

    if ((previous_side < 0.0 && current_side >= 0.0) || (previous_side > 0.0 && current_side <= 0.0)){
        return 1;  // Houve cruzamento
    }

    return 0;
}

int main(int argc, char **argv){
    if (argc < 4 || argc > 5){  // Valida quantidade de argumentos
        usage(argv[0]);
        return 1;
    }

    const char *path = argv[1];  // Caminho do arquivo YUV

    int width  = parse_positive_int(argv[2], "largura");
    int height = parse_positive_int(argv[3], "altura");
    int fps    = argc == 5 ? parse_positive_int(argv[4], "fps") : 30;  // FPS padrão 30

    if (width <= 0 || height <= 0 || fps <= 0){
        return 1;
    }

    if ((width % 2) != 0 || (height % 2) != 0){  // YUV420 exige dimensões pares
        fprintf(stderr, "yuv420p exige largura e altura pares.\n");
        return 1;
    }

    size_t y_size = (size_t)width * (size_t)height;  // Tamanho do plano Y
    size_t uv_size = y_size / 4;  // Tamanho dos planos U e V
    size_t frame_size = y_size + (2 * uv_size);  // Tamanho total do frame

    uint8_t *frame = malloc(frame_size);  // Buffer do frame atual

    if (!frame){
        fprintf(stderr, "Falha ao alocar %zu bytes para o frame.\n", frame_size);
        return 1;
    }

    uint8_t *prev_y_plane = calloc(1, y_size);  // Plano Y do frame anterior
    if (!prev_y_plane){
        fprintf(stderr, "Falha ao alocar %zu bytes para o plano y anterior.\n", y_size);
        free(frame);
        return 1;
    }

    uint8_t *mask = calloc(1, y_size);  // Máscara de movimento
    if (!mask){
        fprintf(stderr, "Falha ao alocar %zu bytes para a mascara.\n", y_size);
        free(frame);
        free(prev_y_plane);
        return 1;
    }

    FILE *file = fopen(path, "rb");  // Abre o arquivo de vídeo
    if (!file){
        fprintf(stderr, "Falha ao abrir %s: %s\n", path, strerror(errno));
        free(frame);
        free(prev_y_plane);
        free(mask);
        return 1;
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0){  // Inicializa SDL
        fprintf(stderr, "SDL_Init falhou: %s\n", SDL_GetError());
        fclose(file);
        free(frame);
        free(prev_y_plane);
        free(mask);
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow("YUV420P 4K Player + Homografia", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_RESIZABLE);  // Cria a janela

    if (!window){
        fprintf(stderr, "SDL_CreateWindow falhou: %s\n", SDL_GetError());
        SDL_Quit();
        fclose(file);
        free(frame);
        free(prev_y_plane);
        free(mask);
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);  // Cria o renderer

    if (!renderer){
        fprintf(stderr, "SDL_CreateRenderer falhou: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        fclose(file);
        free(frame);
        free(prev_y_plane);
        free(mask);
        return 1;
    }

    SDL_Texture *texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING, width, height);  // Cria a textura YUV

    if (!texture){
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

    const uint32_t frame_delay_ms = 1000u / (uint32_t)fps;  // Delay entre frames

    int running = 1;  // Controle do loop principal
    int paused  = 0;  // Estado de pausa

    int delta[4] = {-1, 1, -width, width};  // Vizinhos para limpeza de ruído

    SDL_Point control_points[4];  // Pontos de controle em pixels
    int control_points_count = 0;

    SDL_Point lines[2][2];  // Duas linhas de medição
    int line_points_count = 0;

    int selection_stage = 0;  // 0=pontos, 1=linha1, 2=linha2, 3=monitoramento

    int car_detected = 0;  // Flag de carro detectado

    double previous_car_x = 0.0;  // Posição anterior do carro
    double previous_car_y = 0.0;
    double current_car_x = 0.0;  // Posição atual do carro
    double current_car_y = 0.0;

    double current_real_x = 0.0;  // Posição real atual (cm)
    double current_real_y = 0.0;
    double line1_real_x = 0.0;  // Posição real na primeira linha
    double line1_real_y = 0.0;

    int crossed_first_line = 0;  // Flag de cruzamento da primeira linha
    uint32_t line1_time = 0;  // Timestamp da primeira linha
    uint32_t line2_time = 0;  // Timestamp da segunda linha
    double elapsed_seconds = 0.0;  // Tempo entre as linhas

    printf("Clique no primeiro ponto de controle (P1)\n");
    fflush(stdout);

    while (running){  // Loop principal
        uint32_t started_ms = SDL_GetTicks();  // Tempo de início do frame
        SDL_Event event;

        while (SDL_PollEvent(&event)){  // Processa eventos

            if (event.type == SDL_QUIT){  // Fechar janela
                running = 0;
            }
            else if (event.type == SDL_KEYDOWN){  // Teclas
                if (event.key.keysym.sym == SDLK_ESCAPE || event.key.keysym.sym == SDLK_q){
                    running = 0;
                }
                else if (event.key.keysym.sym == SDLK_SPACE){  // Pausa/continua
                    paused = !paused;
                }
            }
            else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT){  // Clique esquerdo
                int mouse_x = event.button.x;
                int mouse_y = event.button.y;

                int window_width, window_height;
                SDL_GetWindowSize(window, &window_width, &window_height);

                int video_x = (mouse_x * width) / window_width;  // Converte para coordenada do vídeo
                int video_y = (mouse_y * height) / window_height;

                if (selection_stage == 0){  // Seleção dos 4 pontos de controle
                    if (control_points_count < 4) {
                        control_points[control_points_count].x = video_x;
                        control_points[control_points_count].y = video_y;

                        printf("Ponto %d: (%d, %d)\n", control_points_count + 1, video_x, video_y);

                        control_points_count++;

                        if (control_points_count == 1){
                            printf("Clique no segundo ponto de controle (P2)\n");
                        }
                        else if (control_points_count == 2){
                            printf("Clique no terceiro ponto de controle (P3)\n");
                        }
                        else if (control_points_count == 3){
                            printf("Clique no quarto ponto de controle (P4)\n");
                        }
                        else if (control_points_count == 4){  // Todos os pontos selecionados
                            printf("\nQuatro pontos de controle selecionados!\n");
                            for (int i = 0; i < 4; i++) {
                                printf("P%d = (%d, %d)  →  real (%.1f, %.1f) cm\n", i + 1, control_points[i].x, control_points[i].y, real_points[i].x, real_points[i].y);
                            }

                            if (compute_homography(control_points, real_points, H) == 0){  // Calcula homografia
                                homography_ready = 1;
                                printf("\nHomografia calculada com sucesso.\n");
                                printf("Agora qualquer ponto (x,y) em pixels" "pode ser convertido para cm.\n");
                            }
                            else{
                                fprintf(stderr, "Erro: nao foi possivel calcular a " "homografia (pontos colineares?).\n");
                                homography_ready = 0;
                            }

                            selection_stage = 1;  // Avança para seleção da primeira linha
                            line_points_count = 0;

                            printf("\nAgora trace a primeira linha.\n");
                            printf("Clique no ponto inicial da primeira linha.\n");
                        }

                        fflush(stdout);
                    }
                }

                else if (selection_stage == 1){  // Seleção da primeira linha
                    lines[0][line_points_count].x = video_x;
                    lines[0][line_points_count].y = video_y;
                    line_points_count++;

                    if (line_points_count == 1){
                        printf("Primeiro ponto da linha 1: (%d, %d)\n", video_x, video_y);
                        printf("Clique no ponto final da primeira linha.\n");
                    }
                    else if (line_points_count == 2){
                        printf("Segundo ponto da linha 1: (%d, %d)\n", video_x, video_y);
                        printf("\nPrimeira linha definida!\n");
                        printf("Agora trace a segunda linha.\n");
                        printf("Clique no ponto inicial da segunda linha.\n");

                        selection_stage = 2;  // Avança para segunda linha
                        line_points_count = 0;
                    }
                    fflush(stdout);
                }
                else if (selection_stage == 2){  // Seleção da segunda linha
                    lines[1][line_points_count].x = video_x;
                    lines[1][line_points_count].y = video_y;
                    line_points_count++;

                    if (line_points_count == 1){
                        printf("Primeiro ponto da linha 2: (%d, %d)\n", video_x, video_y);
                        printf("Clique no ponto final da segunda linha.\n");
                    }
                    else if (line_points_count == 2){
                        printf("Segundo ponto da linha 2: (%d, %d)\n", video_x, video_y);
                        printf("\nDuas linhas definidas com sucesso!\n");

                        selection_stage = 3;  // Inicia monitoramento

                        printf("\nMonitoramento iniciado.\n");
                        printf("Aguardando um carro cruzar a primeira linha...\n");
                    }
                    fflush(stdout);
                }
            }
        }

        if (!running){
            break;
        }

        if (!paused){  // Processa frame apenas se não estiver pausado
            size_t read_bytes = fread(frame, 1, frame_size, file);  // Lê o próximo frame

            if (read_bytes != frame_size){
                if (feof(file)){  // Reinicia o vídeo se chegou ao fim
                    rewind(file);
                    read_bytes = fread(frame, 1, frame_size, file);
                }
                if (read_bytes != frame_size){
                    fprintf(stderr, "Frame incompleto ou erro de leitura.\n");
                    break;
                }
            }

            uint8_t *y_plane = frame;  // Ponteiro para o plano Y
            const uint8_t *u_plane = frame + y_size;  // Plano U
            const uint8_t *v_plane = frame + y_size + uv_size;  // Plano V

            if (prev_y_plane){  // Detecção de movimento
                for (size_t i = 0; i < y_size; i++){
                    uint8_t original_pix = y_plane[i];
                    int diff = abs((int)y_plane[i] - (int)prev_y_plane[i]);  // Diferença de luminância
                    prev_y_plane[i] = original_pix;  // Atualiza frame anterior
                    mask[i] = (diff >= THRESHOLD) ? 1 : 0;  // Marca pixel em movimento
                }
            }

            for (int i = width; i < (int)y_size - width; i++){  // Remove pontos isolados (ruído)
                if (mask[i]){
                    int n_mov = 0;

                    for (int k = 0; k < 4; k++){
                        if (mask[i + delta[k]])
                            n_mov++;
                    }

                    if (n_mov < 2)
                        mask[i] = 0;
                }
            }

            if (selection_stage >= 3){  // Cálculo do centroide apenas após definir as linhas
                int roi_margin = 180;  // Margem da região de interesse

                int min_x = lines[0][0].x;  // Calcula bounding-box das linhas
                int max_x = lines[0][0].x;
                int min_y = lines[0][0].y;
                int max_y = lines[0][0].y;

                if (lines[0][1].x < min_x) min_x = lines[0][1].x;
                if (lines[0][1].x > max_x) max_x = lines[0][1].x;
                if (lines[0][1].y < min_y) min_y = lines[0][1].y;
                if (lines[0][1].y > max_y) max_y = lines[0][1].y;

                if (lines[1][0].x < min_x) min_x = lines[1][0].x;
                if (lines[1][0].x > max_x) max_x = lines[1][0].x;
                if (lines[1][0].y < min_y) min_y = lines[1][0].y;
                if (lines[1][0].y > max_y) max_y = lines[1][0].y;

                if (lines[1][1].x < min_x) min_x = lines[1][1].x;
                if (lines[1][1].x > max_x) max_x = lines[1][1].x;
                if (lines[1][1].y < min_y) min_y = lines[1][1].y;
                if (lines[1][1].y > max_y) max_y = lines[1][1].y;

                min_x = (min_x - roi_margin < 0) ? 0 : min_x - roi_margin;  // Expande a ROI
                min_y = (min_y - roi_margin < 0) ? 0 : min_y - roi_margin;
                max_x = (max_x + roi_margin >= width)  ? width  - 1 : max_x + roi_margin;
                max_y = (max_y + roi_margin >= height) ? height - 1 : max_y + roi_margin;

                uint64_t sum_x = 0;
                uint64_t sum_y = 0;
                uint64_t movement_pixels = 0;

                for (int y = min_y; y <= max_y; y++){  // Soma pixels de movimento dentro da ROI
                    size_t row_start = (size_t)y * (size_t)width;

                    for (int x = min_x; x <= max_x; x++){
                        size_t index = row_start + (size_t)x;

                        if (mask[index]){
                            sum_x += (uint64_t)x;
                            sum_y += (uint64_t)y;
                            movement_pixels++;
                        }
                    }
                }

                if (movement_pixels > 80) {  // Centroide válido
                    current_car_x = (double)sum_x / (double)movement_pixels;  // Calcula centroide
                    current_car_y = (double)sum_y / (double)movement_pixels;

                    if (homography_ready){  // Converte para coordenadas reais
                        transform_point(H, current_car_x, current_car_y, &current_real_x, &current_real_y);
                    }

                    if (!car_detected){  // Primeiro frame com carro
                        car_detected = 1;
                        previous_car_x = current_car_x;
                        previous_car_y = current_car_y;

                        printf("\nCarro detectado na ROI das linhas:\n");
                        printf("  Pixels : (%.1f, %.1f)\n", current_car_x, current_car_y);
                        if (homography_ready){
                            printf("  Reais  : (%.2f, %.2f) cm\n", current_real_x, current_real_y);
                        }
                    }
                    else{
                        if (!crossed_first_line){  // Verifica cruzamento da primeira linha
                            if (crossed_line(lines[0][0], lines[0][1], previous_car_x, previous_car_y, current_car_x, current_car_y)){
                                crossed_first_line = 1;
                                line1_time = SDL_GetTicks();  // Inicia cronômetro

                                if (homography_ready){
                                    line1_real_x = current_real_x;
                                    line1_real_y = current_real_y;
                                }

                                printf("\n================================\n");
                                printf("CARRO ENTROU NA PRIMEIRA LINHA!\n");
                                printf("  Pixels : (%.1f, %.1f)\n", current_car_x, current_car_y);
                                if (homography_ready){
                                    printf("  Reais  : (%.2f, %.2f) cm\n", current_real_x, current_real_y);
                                }
                                printf("Cronometro iniciado.\n");
                                printf("Aguardando o carro sair pela segunda linha...\n");
                                printf("================================\n");
                            }
                        }
                        else{  // Verifica cruzamento da segunda linha
                            if (crossed_line(lines[1][0], lines[1][1], previous_car_x, previous_car_y, current_car_x, current_car_y)){
                                line2_time = SDL_GetTicks();
                                elapsed_seconds = (double)(line2_time - line1_time) / 1000.0;  // Calcula tempo

                                printf("\n================================\n");
                                printf("CARRO SAIU PELA SEGUNDA LINHA!\n");
                                printf("  Pixels : (%.1f, %.1f)\n", current_car_x, current_car_y);

                                if (homography_ready){
                                    printf("  Reais  : (%.2f, %.2f) cm\n", current_real_x, current_real_y);

                                    double dx = current_real_x - line1_real_x;  // Deslocamento
                                    double dy = current_real_y - line1_real_y;
                                    double displacement_cm = hypot(dx, dy);  // Distância euclidiana
                                    double velocity_cm_s = displacement_cm / elapsed_seconds;  // Velocidade
                                    double velocity_m_s = velocity_cm_s / 100.0;

                                    printf("\n--- RESULTADO MÉTRICO ---\n");
                                    printf("Deslocamento : %.2f cm  (%.3f m)\n", displacement_cm, displacement_cm / 100.0);
                                    printf("Tempo        : %.3f s\n", elapsed_seconds);
                                    printf("Velocidade   : %.2f cm/s  (%.3f m/s)\n", velocity_cm_s, velocity_m_s);
                                    printf("-------------------------\n");
                                } else{
                                    printf("Tempo entre as linhas: %.3f segundos\n", elapsed_seconds);
                                }

                                printf("================================\n");

                                crossed_first_line = 0;  // Reinicia para o próximo carro
                                car_detected = 0;

                                printf("\nAguardando o proximo carro...\n");
                            }
                        }

                        previous_car_x = current_car_x;  // Atualiza posição anterior
                        previous_car_y = current_car_y;
                    }
                }
                else{
                    car_detected = 0;  // Sem movimento suficiente
                }
            }

            for (size_t i = 0; i < y_size; i++){  // Realce visual do movimento
                if (mask[i]){
                    y_plane[i] = 255;  // Pixels em movimento ficam brancos
                }
                else{
                    y_plane[i] /= 2;  // Escurece o fundo
                }
            }

            if (SDL_UpdateYUVTexture(texture, NULL, y_plane, width, u_plane, width / 2, v_plane, width / 2) != 0){  // Atualiza textura
                fprintf(stderr, "SDL_UpdateYUVTexture falhou: %s\n", SDL_GetError());
                break;
            }
        }

        SDL_RenderClear(renderer);  // Limpa a tela
        SDL_RenderCopy(renderer, texture, NULL, NULL);  // Desenha o vídeo

        if (selection_stage >= 1){  // Desenha as linhas
            int window_width, window_height;
            SDL_GetWindowSize(window, &window_width, &window_height);

            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);  // Cor vermelha

            if (selection_stage >= 2){  // Desenha primeira linha
                int x1 = (lines[0][0].x * window_width) / width;
                int y1 = (lines[0][0].y * window_height) / height;
                int x2 = (lines[0][1].x * window_width) / width;
                int y2 = (lines[0][1].y * window_height) / height;
                SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
            }

            if (selection_stage >= 3){  // Desenha segunda linha
                int x1 = (lines[1][0].x * window_width) / width;
                int y1 = (lines[1][0].y * window_height) / height;
                int x2 = (lines[1][1].x * window_width) / width;
                int y2 = (lines[1][1].y * window_height) / height;
                SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
            }

            if (selection_stage == 1 && line_points_count == 1){  // Marcador do primeiro ponto da linha 1
                int x = (lines[0][0].x * window_width) / width;
                int y = (lines[0][0].y * window_height) / height;
                SDL_RenderDrawLine(renderer, x - 4, y, x + 4, y);
                SDL_RenderDrawLine(renderer, x, y - 4, x, y + 4);
            }

            if (selection_stage == 2 && line_points_count == 1){  // Marcador do primeiro ponto da linha 2
                int x = (lines[1][0].x * window_width) / width;
                int y = (lines[1][0].y * window_height) / height;
                SDL_RenderDrawLine(renderer, x - 4, y, x + 4, y);
                SDL_RenderDrawLine(renderer, x, y - 4, x, y + 4);
            }
        }

        if (selection_stage >= 3 && car_detected){  // Desenha o centroide do carro
            int window_width, window_height;
            SDL_GetWindowSize(window, &window_width, &window_height);

            int car_screen_x = (int)(current_car_x * window_width / width);
            int car_screen_y = (int)(current_car_y * window_height / height);

            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);  // Cor azul
            SDL_RenderDrawLine(renderer, car_screen_x - 6, car_screen_y, car_screen_x + 6, car_screen_y);
            SDL_RenderDrawLine(renderer, car_screen_x, car_screen_y - 6, car_screen_x, car_screen_y + 6);
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderPresent(renderer);  // Apresenta o frame

        uint32_t elapsed_ms = SDL_GetTicks() - started_ms;
        if (elapsed_ms < frame_delay_ms){  // Mantém o FPS
            SDL_Delay(frame_delay_ms - elapsed_ms);
        }
    }

    printf("\n");

    if (control_points_count > 0){  // Relatório final
        printf("Pontos de controle selecionados:\n");

        for (int i = 0; i < control_points_count; i++){
            printf("P%d = (%d, %d)\n", i + 1, control_points[i].x, control_points[i].y);
        } 

        if (homography_ready){
            printf("Homografia: OK\n");
        }
    }

    if (selection_stage >= 3){
        printf("\nLinhas selecionadas:\n");
        printf("Linha 1: (%d, %d) -> (%d, %d)\n", lines[0][0].x, lines[0][0].y, lines[0][1].x, lines[0][1].y);
        printf("Linha 2: (%d, %d) -> (%d, %d)\n", lines[1][0].x, lines[1][0].y, lines[1][1].x, lines[1][1].y);
    }

    SDL_DestroyTexture(texture);  // Libera recursos SDL
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    fclose(file);  // Fecha o arquivo
    free(frame);  // Libera buffers
    free(prev_y_plane);
    free(mask);

    return 0;
}