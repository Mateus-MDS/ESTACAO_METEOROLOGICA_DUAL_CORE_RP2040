// =========================
// ESTA��O METEOROL�GICA - DUAL CORE RP2040
// Projeto: Monitoramento com AHT20 + BMP280
// Core 0: Aquisi��o de dados dos sensores
// Core 1: Interface de usu�rio (Display + LEDs)
// =========================

// =========================
// BIBLIOTECAS
// =========================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "lib/aht20.h"
#include "lib/bmp280.h"
#include "lib/ssd1306.h"
#include "lib/font.h"

// =========================
// CONFIGURA��ES DO SISTEMA
// =========================
// I2C dos Sensores (Core 0)
#define I2C_SENSORS       i2c0
#define PIN_SDA_SENSORS   0
#define PIN_SCL_SENSORS   1

// I2C do Display (Core 1)
#define I2C_DISPLAY       i2c1
#define PIN_SDA_DISPLAY   14
#define PIN_SCL_DISPLAY   15
#define SSD1306_ADDR      0x3C

// Perif�ricos
#define PIN_BUTTON_A      5
#define PIN_BUZZER        21
#define PIN_LED_R         13
#define PIN_LED_G         11
#define PIN_LED_B         12

// Temporiza��o
#define SAMPLE_MS          1000u    // Leitura dos sensores a cada 1s
#define DISPLAY_REFRESH_MS 500u     // Atualiza��o do display a cada 500ms
#define DEBOUNCE_MS        200u     // Debounce do bot�o
#define LED_BLINK_MS       500u     // LED piscando

// =========================
// TIPOS DE DADOS
// =========================
typedef struct {
    float temp;          // Temperatura do AHT20 (�C)
    float humid;         // Umidade do AHT20 (%)
    float temp_bmp;      // Temperatura do BMP280 (�C)
    float pressure;      // Press�o do BMP280 (kPa)
    uint32_t ts;         // Timestamp da leitura (ms)
} packet_t;

// =========================
// VARI�VEIS GLOBAIS
// =========================
static ssd1306_t DISP;                         // Display OLED
static volatile bool btnA_pressed = false;     // Flag de bot�o pressionado
static volatile uint32_t btn_last_time_ms = 0; // �ltimo tempo do bot�o (debounce)
static packet_t LATEST_PACKET = {0};           // �ltimo pacote recebido

// =========================
// PROT�TIPOS DE FUN��ES
// =========================
// Core 0 - Sensores
static void core0_main_loop(void);
static bool sensors_init(i2c_inst_t *i2c);
static bool sensors_read_once(i2c_inst_t *i2c, packet_t *out);

// Core 1 - Interface
static void core1_main_loop(void);
static void ui_init_peripherals(void);
static void ui_init_display(void);
static void ui_refresh_display(void);
static void ui_handle_button(void);
static void gpio_button_callback(uint gpio, uint32_t events);

// Comunica��o FIFO
static void fifo_send_packet(const packet_t *p);
static bool fifo_receive_packet(packet_t *out);

// Utilit�rios
static void led_rgb_set(uint8_t r, uint8_t g, uint8_t b);
static void buzzer_tone(int ms);
static void i2c_scan_print(i2c_inst_t *i2c);

// =========================
// FUN��ES AUXILIARES
// =========================

/**
 * @brief Escaneia o barramento I2C e imprime os endere�os encontrados
 * @param i2c Inst�ncia I2C a ser escaneada
 */
static void i2c_scan_print(i2c_inst_t *i2c) {
    printf("[I2C SCAN] Procurando dispositivos...\n");
    bool found = false;
    
    for (int addr = 1; addr < 128; ++addr) {
        uint8_t tmp;
        if (i2c_read_blocking(i2c, addr, &tmp, 1, false) >= 0) {
            printf(" - Dispositivo encontrado: 0x%02X\n", addr);
            found = true;
        }
    }
    
    if (!found) {
        printf(" - Nenhum dispositivo encontrado!\n");
    }
}

/**
 * @brief Inicializa o barramento I2C e os sensores AHT20 + BMP280
 * @param i2c Inst�ncia I2C (i2c0)
 * @return true se inicializa��o bem-sucedida, false caso contr�rio
 */
static bool sensors_init(i2c_inst_t *i2c) {
    printf("[CORE0] Inicializando sensores...\n");
    
    // Configura I2C a 400kHz
    i2c_init(i2c, 400 * 1000);
    gpio_set_function(PIN_SDA_SENSORS, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL_SENSORS, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA_SENSORS);
    gpio_pull_up(PIN_SCL_SENSORS);
    sleep_ms(50);

    // Escaneia dispositivos I2C
    i2c_scan_print(i2c);

    // Inicializa AHT20
    printf("[CORE0] Inicializando AHT20...\n");
    aht20_reset(i2c);
    sleep_ms(20);
    if (!aht20_init(i2c)) {
        printf("[CORE0] ERRO: Falha ao inicializar AHT20!\n");
        return false;
    }
    printf("[CORE0] AHT20 inicializado com sucesso!\n");

    // Inicializa BMP280
    printf("[CORE0] Inicializando BMP280...\n");
    bmp280_reset(i2c);
    sleep_ms(20);
    if (!bmp280_init(i2c)) {
        printf("[CORE0] ERRO: Falha ao inicializar BMP280!\n");
        return false;
    }
    printf("[CORE0] BMP280 inicializado com sucesso!\n");

    return true;
}

/**
 * @brief Realiza leitura �nica dos sensores AHT20 e BMP280
 * @param i2c Inst�ncia I2C
 * @param out Ponteiro para estrutura packet_t onde os dados ser�o armazenados
 * @return true se leitura bem-sucedida, false caso contr�rio
 */
static bool sensors_read_once(i2c_inst_t *i2c, packet_t *out) {
    AHT20_Data aht;
    int32_t raw_temp_bmp;
    int32_t raw_pressure;
    struct bmp280_calib_param params;
    
    // Obt�m par�metros de calibra��o do BMP280
    bmp280_get_calib_params(i2c, &params);

    // ===== LEITURA DO AHT20 =====
    if (!aht20_read(i2c, &aht)) {
        printf("[CORE0] ERRO: Falha na leitura do AHT20\n");
        return false;
    }
    out->temp = aht.temperature;
    out->humid = aht.humidity;
    
    // ===== LEITURA DO BMP280 =====
    if (!bmp280_read_raw(i2c, &raw_temp_bmp, &raw_pressure)) {
        printf("[CORE0] ERRO: Falha na leitura do BMP280\n");
        return false;
    }
    
    // Converte valores brutos para unidades f�sicas
    // Temperatura em �C (dividido por 100 para converter de cent�simos)
    out->temp_bmp = bmp280_convert_temp(raw_temp_bmp, &params) / 100.0f;
    
    // Press�o em kPa (dividido por 1000 para converter de Pa para kPa)
    out->pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params) / 1000.0f;
    
    // Timestamp da leitura
    out->ts = to_ms_since_boot(get_absolute_time());
    
    return true;
}

// =========================
// COMUNICA��O FIFO ENTRE CORES
// =========================

/**
 * @brief Envia pacote de dados do Core0 para Core1 via FIFO
 * @param p Ponteiro para o pacote a ser enviado
 * 
 * Formato do envio:
 * 1. Marcador 0xAAAAAAAA (sincroniza��o)
 * 2. Dados do pacote em palavras de 32 bits
 */
static void fifo_send_packet(const packet_t *p) {
    // Calcula quantas palavras de 32 bits s�o necess�rias
    uint32_t words = (sizeof(packet_t) + 3) / 4;
    uint32_t buf[words];
    
    // Limpa buffer e copia dados
    memset(buf, 0, sizeof(buf));
    memcpy(buf, p, sizeof(packet_t));

    // Envia marcador de sincroniza��o
    multicore_fifo_push_blocking(0xAAAAAAAA);
    
    // Envia todas as palavras
    for (uint32_t i = 0; i < words; i++) {
        multicore_fifo_push_blocking(buf[i]);
    }
}

/**
 * @brief Recebe pacote de dados do Core0 no Core1 via FIFO
 * @param out Ponteiro onde o pacote ser� armazenado
 * @return true se pacote recebido com sucesso, false se FIFO vazio ou erro
 * 
 * Verifica o marcador de sincroniza��o antes de ler os dados
 */
static bool fifo_receive_packet(packet_t *out) {
    // Verifica se h� dados dispon�veis
    if (!multicore_fifo_rvalid()) return false;
    
    // L� marcador de sincroniza��o
    uint32_t marker = multicore_fifo_pop_blocking();
    if (marker != 0xAAAAAAAA) {
        // Marcador inv�lido - limpa FIFO e retorna erro
        printf("[CORE1] ERRO: Marcador FIFO inv�lido (0x%08X)\n", marker);
        while (multicore_fifo_rvalid()) {
            multicore_fifo_pop_blocking();
        }
        return false;
    }
    
    // Recebe dados
    uint32_t words = (sizeof(packet_t) + 3) / 4;
    uint32_t buf[words];
    
    for (uint32_t i = 0; i < words; i++) {
        buf[i] = multicore_fifo_pop_blocking();
    }
    
    // Copia para estrutura de sa�da
    memset(out, 0, sizeof(packet_t));
    memcpy(out, buf, sizeof(packet_t));
    
    return true;
}

// =========================
// FUN��ES DE INTERFACE (CORE 1)
// =========================

/**
 * @brief Inicializa perif�ricos de interface (bot�o, buzzer, LEDs)
 */
static void ui_init_peripherals(void) {
    printf("[CORE1] Inicializando perif�ricos...\n");
    
    // Configura bot�o A com pull-up e interrup��o
    gpio_init(PIN_BUTTON_A);
    gpio_set_dir(PIN_BUTTON_A, GPIO_IN);
    gpio_pull_up(PIN_BUTTON_A);
    gpio_set_irq_enabled_with_callback(PIN_BUTTON_A, GPIO_IRQ_EDGE_FALL, 
                                       true, gpio_button_callback);

    // Configura buzzer
    gpio_init(PIN_BUZZER);
    gpio_set_dir(PIN_BUZZER, GPIO_OUT);
    gpio_put(PIN_BUZZER, 0);
    
    // Configura LEDs RGB
    gpio_init(PIN_LED_R);
    gpio_set_dir(PIN_LED_R, GPIO_OUT);
    gpio_put(PIN_LED_R, 0);
    
    gpio_init(PIN_LED_G);
    gpio_set_dir(PIN_LED_G, GPIO_OUT);
    gpio_put(PIN_LED_G, 0);
    
    gpio_init(PIN_LED_B);
    gpio_set_dir(PIN_LED_B, GPIO_OUT);
    gpio_put(PIN_LED_B, 0);

    // Sinaliza inicializa��o: LED azul + beep
    led_rgb_set(0, 0, 1);
    buzzer_tone(300);
    
    printf("[CORE1] Perif�ricos inicializados!\n");
}

/**
 * @brief Inicializa display OLED SSD1306
 */
static void ui_init_display(void) {
    printf("[CORE1] Inicializando display OLED...\n");
    
    // Configura I2C a 400kHz
    i2c_init(I2C_DISPLAY, 400 * 1000);
    gpio_set_function(PIN_SDA_DISPLAY, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL_DISPLAY, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA_DISPLAY);
    gpio_pull_up(PIN_SCL_DISPLAY);

    // Inicializa display
    ssd1306_init(&DISP, WIDTH, HEIGHT, false, SSD1306_ADDR, I2C_DISPLAY);
    ssd1306_config(&DISP);
    
    printf("[CORE1] Display OLED inicializado!\n");
}

/**
 * @brief Atualiza o display com os dados mais recentes dos sensores
 */
static void ui_refresh_display(void) {
    char str_temp_aht[16];
    char str_temp_bmp[16];
    char str_umid[16];
    char str_press[16];
    bool cor = true;  // true = branco/aceso, false = preto/apagado
    
    // Formata strings com valores dos sensores
    snprintf(str_temp_aht, sizeof(str_temp_aht), "%.1fC", LATEST_PACKET.temp);
    snprintf(str_temp_bmp, sizeof(str_temp_bmp), "%.1fC", LATEST_PACKET.temp_bmp);
    snprintf(str_umid, sizeof(str_umid), "%.1f%%", LATEST_PACKET.humid);
    snprintf(str_press, sizeof(str_press), "%.1fkPa", LATEST_PACKET.pressure);
    
    // Limpa display
    ssd1306_fill(&DISP, !cor);
    
    // Desenha moldura externa
    ssd1306_rect(&DISP, 1, 1, 122, 60, cor, !cor);
    
    // Desenha linhas divis�rias horizontais
    ssd1306_line(&DISP, 1, 13, 123, 13, cor);  // Linha ap�s cabe�alho
    ssd1306_line(&DISP, 1, 25, 123, 25, cor);  // Divis�ria 1
    ssd1306_line(&DISP, 1, 37, 123, 37, cor);  // Divis�ria 2
    ssd1306_line(&DISP, 1, 49, 123, 49, cor);  // Divis�ria 3
    ssd1306_line(&DISP, 1, 61, 123, 61, cor);  // Linha inferior

    // ===== CABE�ALHO =====
    ssd1306_draw_string(&DISP, "SENSORES", 35, 4);
    
    // ===== SE��O 1: TEMPERATURA AHT20 =====
    ssd1306_draw_string(&DISP, "TEMP AHT:", 3, 17);
    ssd1306_draw_string(&DISP, str_temp_aht, 75, 17);
    
    // ===== SE��O 2: UMIDADE AHT20 =====
    ssd1306_draw_string(&DISP, "UMI AHT:", 3, 29);
    ssd1306_draw_string(&DISP, str_umid, 73, 29);
    
    // ===== SE��O 3: TEMPERATURA BMP280 =====
    ssd1306_draw_string(&DISP, "TEMP BMP:", 3, 41);
    ssd1306_draw_string(&DISP, str_temp_bmp, 75, 41);
    
    // ===== SE��O 4: PRESS�O BMP280 =====
    ssd1306_draw_string(&DISP, "PRE BMP:", 3, 53);
    ssd1306_draw_string(&DISP, str_press, 66, 53);
    
    // Envia buffer para o display
    ssd1306_send_data(&DISP);
}

/**
 * @brief Callback de interrup��o do bot�o A
 * @param gpio N�mero do GPIO que gerou a interrup��o
 * @param events Tipo de evento (borda de subida/descida)
 */
static void gpio_button_callback(uint gpio, uint32_t events) {
    (void)events;  // Par�metro n�o utilizado
    
    // Implementa debounce por software
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - btn_last_time_ms < DEBOUNCE_MS) {
        return;  // Ignora se dentro do per�odo de debounce
    }
    
    btn_last_time_ms = now;
    
    // Define flag de bot�o pressionado
    if (gpio == PIN_BUTTON_A) {
        btnA_pressed = true;
    }
}

/**
 * @brief Trata evento de bot�o pressionado
 */
static void ui_handle_button(void) {
    if (!btnA_pressed) return;
    
    btnA_pressed = false;
    
    // Feedback sonoro
    buzzer_tone(40);
    
    // Voc� pode adicionar a��es aqui (ex: alternar modo de exibi��o)
    printf("[CORE1] Bot�o A pressionado!\n");
}

// =========================
// FUN��ES DE HARDWARE
// =========================

/**
 * @brief Define cor do LED RGB
 * @param r Estado do LED vermelho (0 ou 1)
 * @param g Estado do LED verde (0 ou 1)
 * @param b Estado do LED azul (0 ou 1)
 */
static void led_rgb_set(uint8_t r, uint8_t g, uint8_t b) {
    gpio_put(PIN_LED_R, r);
    gpio_put(PIN_LED_G, g);
    gpio_put(PIN_LED_B, b);
}

/**
 * @brief Emite um tom no buzzer
 * @param ms Dura��o do tom em milissegundos (1-999ms)
 */
static void buzzer_tone(int ms) {
    if (ms <= 0 || ms >= 1000) return;
    
    gpio_put(PIN_BUZZER, 1);
    sleep_ms(ms);
    gpio_put(PIN_BUZZER, 0);
}

// =========================
// CORE 1: LOOP PRINCIPAL - INTERFACE
// =========================

/**
 * @brief Loop principal do Core 1 - Interface de usu�rio
 * 
 * Responsabilidades:
 * - Receber dados do Core 0 via FIFO
 * - Atualizar display OLED
 * - Controlar LEDs (indicador de opera��o)
 * - Tratar eventos de bot�o
 */
static void core1_main_loop(void) {
    printf("[CORE1] Iniciando...\n");
    
    // Inicializa perif�ricos e display
    ui_init_peripherals();
    ui_init_display();

    // Tela de inicializa��o
    ssd1306_fill(&DISP, false);
    ssd1306_draw_string(&DISP, "INICIANDO", 30, 20);
    ssd1306_draw_string(&DISP, "SISTEMA", 32, 35);
    ssd1306_send_data(&DISP);

    // Vari�veis de controle de temporiza��o
    uint32_t last_display_ms = 0;
    uint32_t last_led_ms = 0;
    bool led_state = false;
    bool have_data = false;

    printf("[CORE1] Loop principal iniciado!\n");

    while (true) {
        // ===== RECEP��O DE DADOS DO CORE 0 =====
        packet_t pkt;
        if (fifo_receive_packet(&pkt)) {
            LATEST_PACKET = pkt;
            have_data = true;
            
            // Log dos dados recebidos
            printf("[CORE1] Dados recebidos: T_AHT=%.1f�C | U=%.1f%% | T_BMP=%.1f�C | P=%.1fkPa\n",
                   pkt.temp, pkt.humid, pkt.temp_bmp, pkt.pressure);
        }

        // ===== TRATAMENTO DE BOT�O =====
        ui_handle_button();

        uint32_t now = to_ms_since_boot(get_absolute_time());

        // ===== ATUALIZA��O DO DISPLAY =====
        if (have_data && (now - last_display_ms >= DISPLAY_REFRESH_MS)) {
            last_display_ms = now;
            ui_refresh_display();
        }

        // ===== LED VERDE PISCANDO (HEARTBEAT) =====
        if (now - last_led_ms >= LED_BLINK_MS) {
            last_led_ms = now;
            led_state = !led_state;
            led_rgb_set(0, led_state ? 1 : 0, 0);  // Verde piscando
        }

        // Pequeno delay para n�o sobrecarregar o loop
        sleep_ms(8);
    }
}

// =========================
// CORE 0: LOOP PRINCIPAL - SENSORES
// =========================

/**
 * @brief Loop principal do Core 0 - Aquisi��o de dados
 * 
 * Responsabilidades:
 * - Ler sensores AHT20 e BMP280 periodicamente
 * - Processar dados brutos
 * - Enviar pacotes para Core 1 via FIFO
 */
static void core0_main_loop(void) {
    printf("[CORE0] Iniciando...\n");
    
    // Inicializa sensores
    if (!sensors_init(I2C_SENSORS)) {
        printf("[CORE0] ERRO CR�TICO: Falha na inicializa��o dos sensores!\n");
        printf("[CORE0] Sistema interrompido.\n");
        while (true) {
            sleep_ms(1000);  // Loop infinito em caso de erro
        }
    }

    printf("[CORE0] Loop principal iniciado!\n");
    
    uint32_t last_sample_ms = 0;
    uint32_t sample_count = 0;
    
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // ===== LEITURA PERI�DICA DOS SENSORES =====
        if (now - last_sample_ms >= SAMPLE_MS) {
            last_sample_ms = now;
            sample_count++;

            packet_t pkt;
            if (!sensors_read_once(I2C_SENSORS, &pkt)) {
                printf("[CORE0] ERRO: Falha na leitura dos sensores (tentativa #%lu)\n", 
                       sample_count);
                continue;  // Tenta novamente no pr�ximo ciclo
            }

            // Log dos dados lidos
            printf("[CORE0] Leitura #%lu: T_AHT=%.1f�C | U=%.1f%% | T_BMP=%.1f�C | P=%.1fkPa\n",
                   sample_count, pkt.temp, pkt.humid, pkt.temp_bmp, pkt.pressure);

            // Envia dados para Core 1
            fifo_send_packet(&pkt);
        }

        // Pequeno delay para n�o sobrecarregar o loop
        sleep_ms(10);
    }
}

// =========================
// ENTRADA PRINCIPAL
// =========================

/**
 * @brief Fun��o principal - Inicializa��o do sistema
 * 
 * Sequ�ncia de inicializa��o:
 * 1. Inicializa comunica��o serial (USB)
 * 2. Lan�a Core 1 (Interface)
 * 3. Executa Core 0 (Sensores)
 */
int main(void) {
    // Inicializa comunica��o serial via USB
    stdio_init_all();
    sleep_ms(1000);  // Aguarda estabiliza��o
    
    printf("\n");
    printf("=====================================\n");
    printf("  ESTACAO METEOROLOGICA - DUAL CORE  \n");
    printf("  AHT20 + BMP280 | RP2040 | BitDogLab\n");
    printf("=====================================\n");
    printf("\n");

    // Lan�a Core 1 (Interface de usu�rio)
    printf("[MAIN] Iniciando Core 1 (Interface)...\n");
    multicore_launch_core1(core1_main_loop);
    sleep_ms(200);  // Aguarda inicializa��o do Core 1

    // Executa Core 0 (Sensores)
    printf("[MAIN] Iniciando Core 0 (Sensores)...\n");
    core0_main_loop();
    
    // Nunca deve chegar aqui
    return 0;
}