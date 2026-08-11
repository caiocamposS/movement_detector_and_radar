#include <SDL2/SDL.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define THRESHOLD 25

typedef struct {
    double x;
    double y;
} Point2D;

/* ================================================================
 * PONTOS REAIS CORRESPONDENTES AOS 4 PONTOS DE CONTROLE
 * (retângulo físico de 100 cm × 10 cm)
 *
 *   P1 ---------------- P2
 *   |                    |
 *   |     região         |  10 cm
 *   |     calibrada      |
 *   |                    |
 *   P4 ---------------- P3
 *          100 cm
 * ================================================================ */
static const Point2D real_points[4] = {
    {  0.0,  0.0 },   /* P1 */
    {100.0,  0.0 },   /* P2 */
    {100.0, 10.0 },   /* P3 */
    {  0.0, 10.0 }    /* P4 */
};

/* Matriz de homografia 3×3 (row-major) + flag de prontidão */
static double H[9];
static int   homography_ready = 0;

/* ================================================================
 * SOLUCIONADOR LINEAR 8×8 (eliminação de Gauss com pivotamento)
 * Resolve A * x = b  (A é modificado in-place)
 * ================================================================ */
static int solve_8x8(double A[8][8], double b[8], double x[8])
{
    int i, j, k, piv;
    double maxv, tmp, factor;

    for (i = 0; i < 8; i++) {

        /* Pivotamento parcial */
        piv = i;
        maxv = fabs(A[i][i]);
        for (k = i + 1; k < 8; k++) {
            if (fabs(A[k][i]) > maxv) {
                maxv = fabs(A[k][i]);
                piv = k;
            }
        }

        if (maxv < 1e-12)
            return -1; /* matriz singular */

        /* Troca de linhas */
        if (piv != i) {
            for (j = 0; j < 8; j++) {
                tmp = A[i][j];
                A[i][j] = A[piv][j];
                A[piv][j] = tmp;
            }
            tmp = b[i];
            b[i] = b[piv];
            b[piv] = tmp;
        }

        /* Eliminação */
        for (k = i + 1; k < 8; k++) {
            factor = A[k][i] / A[i][i];
            for (j = i; j < 8; j++)
                A[k][j] -= factor * A[i][j];
            b[k] -= factor * b[i];
        }
    }

    /* Substituição regressiva */
    for (i = 7; i >= 0; i--) {
        tmp = b[i];
        for (j = i + 1; j < 8; j++)
            tmp -= A[i][j] * x[j];
        x[i] = tmp / A[i][i];
    }

    return 0;
}

/* ================================================================
 * CALCULA A HOMOGRAFIA A PARTIR DE 4 CORRESPONDÊNCIAS
 * (Direct Linear Transformation com h33 = 1)
 * ================================================================ */
static int compute_homography(const SDL_Point src[4],
                              const Point2D  dst[4],
                              double         H_out[9])
{
    double A[8][8];
    double b[8];
    double h[8];
    int i;

    memset(A, 0, sizeof(A));

    for (i = 0; i < 4; i++) {
        double xi = (double)src[i].x;
        double yi = (double)src[i].y;
        double ui = dst[i].x;
        double vi = dst[i].y;

        /* Equação para u */
        A[2*i][0] = xi;
        A[2*i][1] = yi;
        A[2*i][2] = 1.0;
        A[2*i][3] = 0.0;
        A[2*i][4] = 0.0;
        A[2*i][5] = 0.0;
        A[2*i][6] = -ui * xi;
        A[2*i][7] = -ui * yi;
        b[2*i]    = ui;

        /* Equação para v */
        A[2*i+1][0] = 0.0;
        A[2*i+1][1] = 0.0;
        A[2*i+1][2] = 0.0;
        A[2*i+1][3] = xi;
        A[2*i+1][4] = yi;
        A[2*i+1][5] = 1.0;
        A[2*i+1][6] = -vi * xi;
        A[2*i+1][7] = -vi * yi;
        b[2*i+1]    = vi;
    }

    if (solve_8x8(A, b, h) != 0)
        return -1;

    H_out[0] = h[0]; H_out[1] = h[1]; H_out[2] = h[2];
    H_out[3] = h[3]; H_out[4] = h[4]; H_out[5] = h[5];
    H_out[6] = h[6]; H_out[7] = h[7]; H_out[8] = 1.0;

    return 0;
}

/* ================================================================
 * TRANSFORMA UM PONTO DE PIXELS → COORDENADAS REAIS (cm)
 * ================================================================ */
static void transform_point(const double H_mat[9],
                            double px, double py,
                            double *rx, double *ry)
{
    double X = H_mat[0]*px + H_mat[1]*py + H_mat[2];
    double Y = H_mat[3]*px + H_mat[4]*py + H_mat[5];
    double W = H_mat[6]*px + H_mat[7]*py + H_mat[8];

    if (fabs(W) < 1e-12) {
        *rx = 0.0;
        *ry = 0.0;
        return;
    }

    *rx = X / W;
    *ry = Y / W;
}

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

    if (errno != 0 || end == text || *end != '\0' ||
        value <= 0 || value > INT32_MAX) {

        fprintf(stderr, "%s invalido: %s\n", name, text);
        return -1;
    }

    return (int)value;
}

/*
 * ================================================================
 * CALCULA DE QUE LADO DA LINHA O PONTO ESTÁ
 * ================================================================
 */
static double line_side(
    SDL_Point line_start,
    SDL_Point line_end,
    double x,
    double y)
{
    return
        (double)(line_end.x - line_start.x) *
        (y - line_start.y)
        -
        (double)(line_end.y - line_start.y) *
        (x - line_start.x);
}

/*
 * ================================================================
 * VERIFICA SE O OBJETO ATRAVESSOU A LINHA
 * ================================================================
 */
static int crossed_line(
    SDL_Point line_start,
    SDL_Point line_end,
    double previous_x,
    double previous_y,
    double current_x,
    double current_y)
{
    double previous_side =
        line_side(line_start, line_end, previous_x, previous_y);

    double current_side =
        line_side(line_start, line_end, current_x, current_y);

    if ((previous_side < 0.0 && current_side >= 0.0) ||
        (previous_side > 0.0 && current_side <= 0.0)) {
        return 1;
    }

    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 4 || argc > 5) {
        usage(argv[0]);
        return 1;
    }

    const char *path = argv[1];

    int width  = parse_positive_int(argv[2], "largura");
    int height = parse_positive_int(argv[3], "altura");
    int fps    = argc == 5 ? parse_positive_int(argv[4], "fps") : 30;

    if (width <= 0 || height <= 0 || fps <= 0)
        return 1;

    if ((width % 2) != 0 || (height % 2) != 0) {
        fprintf(stderr, "yuv420p exige largura e altura pares.\n");
        return 1;
    }

    size_t y_size     = (size_t)width * (size_t)height;
    size_t uv_size    = y_size / 4;
    size_t frame_size = y_size + (2 * uv_size);

    uint8_t *frame = malloc(frame_size);
    if (!frame) {
        fprintf(stderr, "Falha ao alocar %zu bytes para o frame.\n", frame_size);
        return 1;
    }

    uint8_t *prev_y_plane = calloc(1, y_size);
    if (!prev_y_plane) {
        fprintf(stderr, "Falha ao alocar %zu bytes para o plano y anterior.\n", y_size);
        free(frame);
        return 1;
    }

    uint8_t *mask = calloc(1, y_size);
    if (!mask) {
        fprintf(stderr, "Falha ao alocar %zu bytes para a mascara.\n", y_size);
        free(frame);
        free(prev_y_plane);
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

    SDL_Window *window = SDL_CreateWindow(
        "YUV420P 4K Player + Homografia",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        width,
        height,
        SDL_WINDOW_RESIZABLE
    );

    if (!window) {
        fprintf(stderr, "SDL_CreateWindow falhou: %s\n", SDL_GetError());
        SDL_Quit();
        fclose(file);
        free(frame);
        free(prev_y_plane);
        free(mask);
        return 1;
    }

    SDL_Renderer *renderer =
        SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

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

    SDL_Texture *texture =
        SDL_CreateTexture(renderer,
                          SDL_PIXELFORMAT_IYUV,
                          SDL_TEXTUREACCESS_STREAMING,
                          width, height);

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
    int paused  = 0;

    int delta[4] = { -1, 1, -width, width };

    /* ================================================================
     * PONTOS DE CONTROLE (pixels)
     * ================================================================ */
    SDL_Point control_points[4];
    int control_points_count = 0;

    /* ================================================================
     * LINHAS
     * ================================================================ */
    SDL_Point lines[2][2];
    int line_points_count = 0;

    /*
     * 0 = pontos de controle
     * 1 = primeira linha
     * 2 = segunda linha
     * 3 = monitoramento
     */
    int selection_stage = 0;

    /* ================================================================
     * RASTREAMENTO DO CARRO
     * ================================================================ */
    int car_detected = 0;

    double previous_car_x = 0.0;
    double previous_car_y = 0.0;
    double current_car_x  = 0.0;
    double current_car_y  = 0.0;

    /* Coordenadas reais (cm) */
    double current_real_x = 0.0;
    double current_real_y = 0.0;
    double line1_real_x   = 0.0;
    double line1_real_y   = 0.0;

    int crossed_first_line = 0;
    uint32_t line1_time    = 0;
    uint32_t line2_time    = 0;
    double elapsed_seconds = 0.0;

    printf("Clique no primeiro ponto de controle (P1)\n");
    fflush(stdout);

    while (running) {

        uint32_t started_ms = SDL_GetTicks();
        SDL_Event event;

        while (SDL_PollEvent(&event)) {

            if (event.type == SDL_QUIT) {
                running = 0;
            }
            else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE ||
                    event.key.keysym.sym == SDLK_q) {
                    running = 0;
                }
                else if (event.key.keysym.sym == SDLK_SPACE) {
                    paused = !paused;
                }
            }
            else if (event.type == SDL_MOUSEBUTTONDOWN &&
                     event.button.button == SDL_BUTTON_LEFT) {

                int mouse_x = event.button.x;
                int mouse_y = event.button.y;

                int window_width, window_height;
                SDL_GetWindowSize(window, &window_width, &window_height);

                int video_x = (mouse_x * width)  / window_width;
                int video_y = (mouse_y * height) / window_height;

                /* ====================================================
                 * ETAPA 0 – QUATRO PONTOS DE CONTROLE
                 * ==================================================== */
                if (selection_stage == 0) {

                    if (control_points_count < 4) {

                        control_points[control_points_count].x = video_x;
                        control_points[control_points_count].y = video_y;

                        printf("Ponto %d: (%d, %d)\n",
                               control_points_count + 1, video_x, video_y);

                        control_points_count++;

                        if (control_points_count == 1)
                            printf("Clique no segundo ponto de controle (P2)\n");
                        else if (control_points_count == 2)
                            printf("Clique no terceiro ponto de controle (P3)\n");
                        else if (control_points_count == 3)
                            printf("Clique no quarto ponto de controle (P4)\n");
                        else if (control_points_count == 4) {

                            printf("\nQuatro pontos de controle selecionados!\n");
                            for (int i = 0; i < 4; i++) {
                                printf("P%d = (%d, %d)  →  real (%.1f, %.1f) cm\n",
                                       i + 1,
                                       control_points[i].x,
                                       control_points[i].y,
                                       real_points[i].x,
                                       real_points[i].y);
                            }

                            /* Calcula a homografia */
                            if (compute_homography(control_points,
                                                   real_points, H) == 0) {
                                homography_ready = 1;
                                printf("\nHomografia calculada com sucesso.\n");
                                printf("Agora qualquer ponto (x,y) em pixels "
                                       "pode ser convertido para cm.\n");
                            } else {
                                fprintf(stderr,
                                    "Erro: nao foi possivel calcular a "
                                    "homografia (pontos colineares?).\n");
                                homography_ready = 0;
                            }

                            selection_stage = 1;
                            line_points_count = 0;

                            printf("\nAgora trace a primeira linha.\n");
                            printf("Clique no ponto inicial da primeira linha.\n");
                        }
                        fflush(stdout);
                    }
                }

                /* ====================================================
                 * ETAPA 1 – PRIMEIRA LINHA
                 * ==================================================== */
                else if (selection_stage == 1) {

                    lines[0][line_points_count].x = video_x;
                    lines[0][line_points_count].y = video_y;
                    line_points_count++;

                    if (line_points_count == 1) {
                        printf("Primeiro ponto da linha 1: (%d, %d)\n",
                               video_x, video_y);
                        printf("Clique no ponto final da primeira linha.\n");
                    }
                    else if (line_points_count == 2) {
                        printf("Segundo ponto da linha 1: (%d, %d)\n",
                               video_x, video_y);
                        printf("\nPrimeira linha definida!\n");
                        printf("Agora trace a segunda linha.\n");
                        printf("Clique no ponto inicial da segunda linha.\n");

                        selection_stage = 2;
                        line_points_count = 0;
                    }
                    fflush(stdout);
                }

                /* ====================================================
                 * ETAPA 2 – SEGUNDA LINHA
                 * ==================================================== */
                else if (selection_stage == 2) {

                    lines[1][line_points_count].x = video_x;
                    lines[1][line_points_count].y = video_y;
                    line_points_count++;

                    if (line_points_count == 1) {
                        printf("Primeiro ponto da linha 2: (%d, %d)\n",
                               video_x, video_y);
                        printf("Clique no ponto final da segunda linha.\n");
                    }
                    else if (line_points_count == 2) {
                        printf("Segundo ponto da linha 2: (%d, %d)\n",
                               video_x, video_y);
                        printf("\nDuas linhas definidas com sucesso!\n");

                        selection_stage = 3;

                        printf("\nMonitoramento iniciado.\n");
                        printf("Aguardando um carro cruzar a primeira linha...\n");
                    }
                    fflush(stdout);
                }
            }
        }

        if (!running)
            break;

        /* ============================================================
         * LEITURA DO FRAME
         * ============================================================ */
        if (!paused) {

            size_t read_bytes = fread(frame, 1, frame_size, file);

            if (read_bytes != frame_size) {
                if (feof(file)) {
                    rewind(file);
                    read_bytes = fread(frame, 1, frame_size, file);
                }
                if (read_bytes != frame_size) {
                    fprintf(stderr, "Frame incompleto ou erro de leitura.\n");
                    break;
                }
            }

            uint8_t *y_plane = frame;
            const uint8_t *u_plane = frame + y_size;
            const uint8_t *v_plane = frame + y_size + uv_size;

            /* ========================================================
             * DETECTOR DE MOVIMENTO
             * ======================================================== */
            if (prev_y_plane) {
                for (size_t i = 0; i < y_size; i++) {
                    uint8_t original_pix = y_plane[i];
                    int diff = abs((int)y_plane[i] - (int)prev_y_plane[i]);
                    prev_y_plane[i] = original_pix;
                    mask[i] = (diff >= THRESHOLD) ? 1 : 0;
                }
            }

            /* Remove pontos isolados */
            for (int i = width; i < (int)y_size - width; i++) {
                if (mask[i]) {
                    int n_mov = 0;
                    for (int k = 0; k < 4; k++) {
                        if (mask[i + delta[k]])
                            n_mov++;
                    }
                    if (n_mov < 2)
                        mask[i] = 0;
                }
            }

            /* ========================================================
             * CENTROIDE + TRANSFORMAÇÃO PARA COORDENADAS REAIS
             * Agora restrito à ROI das linhas de medição
             * ======================================================== */
            if (selection_stage >= 3) {

                /* ------------------------------------------------
                 * Calcula a bounding-box das duas linhas + margem
                 * ------------------------------------------------ */
                int roi_margin = 180;   /* ajuste se precisar */

                int min_x = lines[0][0].x;
                int max_x = lines[0][0].x;
                int min_y = lines[0][0].y;
                int max_y = lines[0][0].y;

                /* Linha 1 */
                if (lines[0][1].x < min_x) min_x = lines[0][1].x;
                if (lines[0][1].x > max_x) max_x = lines[0][1].x;
                if (lines[0][1].y < min_y) min_y = lines[0][1].y;
                if (lines[0][1].y > max_y) max_y = lines[0][1].y;

                /* Linha 2 */
                if (lines[1][0].x < min_x) min_x = lines[1][0].x;
                if (lines[1][0].x > max_x) max_x = lines[1][0].x;
                if (lines[1][0].y < min_y) min_y = lines[1][0].y;
                if (lines[1][0].y > max_y) max_y = lines[1][0].y;

                if (lines[1][1].x < min_x) min_x = lines[1][1].x;
                if (lines[1][1].x > max_x) max_x = lines[1][1].x;
                if (lines[1][1].y < min_y) min_y = lines[1][1].y;
                if (lines[1][1].y > max_y) max_y = lines[1][1].y;

                /* Expande com margem e limita aos limites da imagem */
                min_x = (min_x - roi_margin < 0) ? 0 : min_x - roi_margin;
                min_y = (min_y - roi_margin < 0) ? 0 : min_y - roi_margin;
                max_x = (max_x + roi_margin >= width)  ? width  - 1 : max_x + roi_margin;
                max_y = (max_y + roi_margin >= height) ? height - 1 : max_y + roi_margin;

                /* ------------------------------------------------
                 * Soma apenas os pixels de movimento DENTRO da ROI
                 * ------------------------------------------------ */
                uint64_t sum_x = 0;
                uint64_t sum_y = 0;
                uint64_t movement_pixels = 0;

                for (int y = min_y; y <= max_y; y++) {
                    size_t row_start = (size_t)y * (size_t)width;
                    for (int x = min_x; x <= max_x; x++) {
                        size_t index = row_start + (size_t)x;
                        if (mask[index]) {
                            sum_x += (uint64_t)x;
                            sum_y += (uint64_t)y;
                            movement_pixels++;
                        }
                    }
                }

                if (movement_pixels > 80) {   /* limiar um pouco menor porque a ROI é menor */

                    current_car_x = (double)sum_x / (double)movement_pixels;
                    current_car_y = (double)sum_y / (double)movement_pixels;

                    /* Converte para coordenadas reais (cm) */
                    if (homography_ready) {
                        transform_point(H,
                                        current_car_x, current_car_y,
                                        &current_real_x, &current_real_y);
                    }

                    if (!car_detected) {
                        car_detected = 1;
                        previous_car_x = current_car_x;
                        previous_car_y = current_car_y;

                        printf("\nCarro detectado na ROI das linhas:\n");
                        printf("  Pixels : (%.1f, %.1f)\n",
                               current_car_x, current_car_y);
                        if (homography_ready) {
                            printf("  Reais  : (%.2f, %.2f) cm\n",
                                   current_real_x, current_real_y);
                        }
                    }
                    else {
                        /* =================================================
                         * VERIFICA PRIMEIRA LINHA
                         * ================================================= */
                        if (!crossed_first_line) {

                            if (crossed_line(lines[0][0], lines[0][1],
                                             previous_car_x, previous_car_y,
                                             current_car_x, current_car_y)) {

                                crossed_first_line = 1;
                                line1_time = SDL_GetTicks();

                                if (homography_ready) {
                                    line1_real_x = current_real_x;
                                    line1_real_y = current_real_y;
                                }

                                printf("\n================================\n");
                                printf("CARRO ENTROU NA PRIMEIRA LINHA!\n");
                                printf("  Pixels : (%.1f, %.1f)\n",
                                       current_car_x, current_car_y);
                                if (homography_ready) {
                                    printf("  Reais  : (%.2f, %.2f) cm\n",
                                           current_real_x, current_real_y);
                                }
                                printf("Cronometro iniciado.\n");
                                printf("Aguardando o carro sair pela segunda linha...\n");
                                printf("================================\n");
                            }
                        }
                        /* =================================================
                         * VERIFICA SEGUNDA LINHA
                         * ================================================= */
                        else {

                            if (crossed_line(lines[1][0], lines[1][1],
                                             previous_car_x, previous_car_y,
                                             current_car_x, current_car_y)) {

                                line2_time = SDL_GetTicks();
                                elapsed_seconds =
                                    (double)(line2_time - line1_time) / 1000.0;

                                printf("\n================================\n");
                                printf("CARRO SAIU PELA SEGUNDA LINHA!\n");
                                printf("  Pixels : (%.1f, %.1f)\n",
                                       current_car_x, current_car_y);

                                if (homography_ready) {
                                    printf("  Reais  : (%.2f, %.2f) cm\n",
                                           current_real_x, current_real_y);

                                    double dx = current_real_x - line1_real_x;
                                    double dy = current_real_y - line1_real_y;
                                    double displacement_cm = hypot(dx, dy);
                                    double velocity_cm_s  = displacement_cm / elapsed_seconds;
                                    double velocity_m_s   = velocity_cm_s / 100.0;

                                    printf("\n--- RESULTADO MÉTRICO ---\n");
                                    printf("Deslocamento : %.2f cm  (%.3f m)\n",
                                           displacement_cm, displacement_cm / 100.0);
                                    printf("Tempo        : %.3f s\n", elapsed_seconds);
                                    printf("Velocidade   : %.2f cm/s  (%.3f m/s)\n",
                                           velocity_cm_s, velocity_m_s);
                                    printf("-------------------------\n");
                                } else {
                                    printf("Tempo entre as linhas: %.3f segundos\n",
                                           elapsed_seconds);
                                }

                                printf("================================\n");

                                crossed_first_line = 0;
                                car_detected = 0;

                                printf("\nAguardando o proximo carro...\n");
                            }
                        }

                        previous_car_x = current_car_x;
                        previous_car_y = current_car_y;
                    }
                }
                else {
                    car_detected = 0;
                }
            }

            /* ========================================================
             * REALCE VISUAL DO MOVIMENTO
             * ======================================================== */
            for (size_t i = 0; i < y_size; i++) {
                if (mask[i])
                    y_plane[i] = 255;
                else
                    y_plane[i] /= 2;
            }

            if (SDL_UpdateYUVTexture(texture, NULL,
                                     y_plane, width,
                                     u_plane, width / 2,
                                     v_plane, width / 2) != 0) {
                fprintf(stderr, "SDL_UpdateYUVTexture falhou: %s\n",
                        SDL_GetError());
                break;
            }
        }

        /* ============================================================
         * RENDERIZAÇÃO
         * ============================================================ */
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);

        if (selection_stage >= 1) {

            int window_width, window_height;
            SDL_GetWindowSize(window, &window_width, &window_height);

            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

            /* Primeira linha */
            if (selection_stage >= 2) {
                int x1 = (lines[0][0].x * window_width)  / width;
                int y1 = (lines[0][0].y * window_height) / height;
                int x2 = (lines[0][1].x * window_width)  / width;
                int y2 = (lines[0][1].y * window_height) / height;
                SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
            }

            /* Segunda linha */
            if (selection_stage >= 3) {
                int x1 = (lines[1][0].x * window_width)  / width;
                int y1 = (lines[1][0].y * window_height) / height;
                int x2 = (lines[1][1].x * window_width)  / width;
                int y2 = (lines[1][1].y * window_height) / height;
                SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
            }

            /* Marcadores de ponto inicial */
            if (selection_stage == 1 && line_points_count == 1) {
                int x = (lines[0][0].x * window_width)  / width;
                int y = (lines[0][0].y * window_height) / height;
                SDL_RenderDrawLine(renderer, x - 4, y, x + 4, y);
                SDL_RenderDrawLine(renderer, x, y - 4, x, y + 4);
            }

            if (selection_stage == 2 && line_points_count == 1) {
                int x = (lines[1][0].x * window_width)  / width;
                int y = (lines[1][0].y * window_height) / height;
                SDL_RenderDrawLine(renderer, x - 4, y, x + 4, y);
                SDL_RenderDrawLine(renderer, x, y - 4, x, y + 4);
            }
        }

        /* Centroide do carro (azul) */
        if (selection_stage >= 3 && car_detected) {
            int window_width, window_height;
            SDL_GetWindowSize(window, &window_width, &window_height);

            int car_screen_x = (int)(current_car_x * window_width  / width);
            int car_screen_y = (int)(current_car_y * window_height / height);

            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
            SDL_RenderDrawLine(renderer,
                               car_screen_x - 6, car_screen_y,
                               car_screen_x + 6, car_screen_y);
            SDL_RenderDrawLine(renderer,
                               car_screen_x, car_screen_y - 6,
                               car_screen_x, car_screen_y + 6);
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderPresent(renderer);

        uint32_t elapsed_ms = SDL_GetTicks() - started_ms;
        if (elapsed_ms < frame_delay_ms)
            SDL_Delay(frame_delay_ms - elapsed_ms);
    }

    /* ============================================================
     * RESULTADO FINAL
     * ============================================================ */
    printf("\n");

    if (control_points_count > 0) {
        printf("Pontos de controle selecionados:\n");
        for (int i = 0; i < control_points_count; i++) {
            printf("P%d = (%d, %d)\n",
                   i + 1,
                   control_points[i].x,
                   control_points[i].y);
        }
        if (homography_ready)
            printf("Homografia: OK\n");
    }

    if (selection_stage >= 3) {
        printf("\nLinhas selecionadas:\n");
        printf("Linha 1: (%d, %d) -> (%d, %d)\n",
               lines[0][0].x, lines[0][0].y,
               lines[0][1].x, lines[0][1].y);
        printf("Linha 2: (%d, %d) -> (%d, %d)\n",
               lines[1][0].x, lines[1][0].y,
               lines[1][1].x, lines[1][1].y);
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