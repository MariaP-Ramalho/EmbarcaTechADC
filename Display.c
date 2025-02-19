#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "inc/ssd1306.h"

// Definições de pinos e constantes
#define I2C_PORT i2c1
#define I2C_SDA_PIN 14
#define I2C_SCL_PIN 15
#define DISPLAY_ADDR 0x3C

#define JOYSTICK_X 27
#define JOYSTICK_Y 26
#define JOYSTICK_PB_PIN 22

#define BUTTON_A 5
#define LED_RED 13
#define LED_GREEN 11
#define LED_BLUE 12

#define PWM_FREQ 50
#define PWM_WRAP 4095
#define DEADZONE 200

#define DEBOUNCE 200

//ENUM para os tipos de borda
typedef enum {
  BORDER_SOLIDA = 0,
  BORDER_PONTILHADA = 1,
  BORDER_DUPLA = 2,
  BORDER_TRACEJADA = 3,
  BORDER_MAX // Usado para evitar valores inválidos
} BorderType;

// Estrutura para armazenar o estado do sistema
typedef struct {
    bool led_active;
    bool led_green_status;
    BorderType  border_type; 
    int border_size;
    uint32_t last_joystick_press_time;
    uint32_t last_button_a_press_time;
} SystemState;


// Inicialização das funções
void init_i2c();
void init_adc();
void init_gpio();
void init_pwm(uint pin);
void change_led(uint pin, uint16_t value);
void change_border(ssd1306_t *ssd);
void update_display(ssd1306_t *ssd, uint8_t pos_x, uint8_t pos_y);
void update_leds(uint16_t adc_x, uint16_t adc_y, SystemState *state);
void gpio_irq_handler(uint gpio, uint32_t events);

// Função para mapear valores de um intervalo para outro
uint16_t map(uint16_t value, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Variáveis globais
ssd1306_t display;
SystemState system_state = {
    .led_active = true,
    .led_green_status = false,
    .border_type = true,
    .border_size = 2,
    .last_joystick_press_time = 0,
    .last_button_a_press_time = 0
};

// Seta o tipo de borda
void set_border_type(uint8_t new_type) {
  if (new_type < BORDER_MAX) {
      system_state.border_type = (BorderType)new_type;
  } else {
      system_state.border_type = BORDER_SOLIDA; // Fallback seguro
  }
}


// Função principal
int main() {
    stdio_init_all();

    // Inicializa hardware
    init_i2c();
    init_adc();
    init_gpio();

    // Inicializa o display SSD1306
    ssd1306_init(&display, WIDTH, HEIGHT, false, DISPLAY_ADDR, I2C_PORT);
    ssd1306_config(&display);
    ssd1306_fill(&display, false);
    ssd1306_send_data(&display);

    // Loop principal
    while (true) {
        // Lê os valores do joystick
        adc_select_input(1);  // Canal para o eixo X
        uint16_t adc_x = adc_read();
        
        adc_select_input(0);  // Canal para o eixo Y
        uint16_t adc_y = adc_read();
        
        // Atualiza os LEDs com base nos valores do joystick
        update_leds(adc_x, adc_y, &system_state);
        
        // Calcula a posição do quadrado no display
        uint8_t pos_x = (adc_x * (WIDTH - 8)) / 4095;
        uint8_t pos_y = ((4095 - adc_y) * (HEIGHT - 8)) / 4095;
        
        // Atualiza o display
        update_display(&display, pos_x, pos_y);
        
        // Pequena pausa para não sobrecarregar o processador
        sleep_ms(50);
    }
}

// Configuração do I2C
void init_i2c() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

// Configuração do ADC
void init_adc() {
    adc_init();
    adc_gpio_init(JOYSTICK_X);
    adc_gpio_init(JOYSTICK_Y);
}

// Configuração dos GPIOs
void init_gpio() {
    // Configura o botão do joystick com interrupção
    gpio_init(JOYSTICK_PB_PIN);
    gpio_set_dir(JOYSTICK_PB_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_PB_PIN);
    gpio_set_irq_enabled_with_callback(JOYSTICK_PB_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    // Configura o botão A com interrupção
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);
    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    // Configura os LEDs (RED e BLUE com PWM, GREEN como GPIO)
    init_pwm(LED_RED);
    init_pwm(LED_BLUE);
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
}

// Configuração do PWM
void init_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin); 
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_clkdiv(slice, 125.0); 
    pwm_set_enabled(slice, true);
}

// Define o brilho do LED
void change_led(uint pin, uint16_t value) {
    uint slice = pwm_gpio_to_slice_num(pin);
    uint channel = pwm_gpio_to_channel(pin);
    pwm_set_chan_level(slice, channel, value);
}

// Desenha a borda no display
void change_border(ssd1306_t *ssd) {
  switch (system_state.border_type) {
      case BORDER_SOLIDA:
          for (int i = 0; i < system_state.border_size * 2; i++) {
              ssd1306_rect(ssd, i, i, WIDTH - (2 * i), HEIGHT - (2 * i), true, false);
          }
          break;

      case BORDER_PONTILHADA:
          for (int i = 0; i < HEIGHT; i += 5) {
              ssd1306_pixel(ssd, 0, i, true);
              ssd1306_pixel(ssd, WIDTH - 1, i, true);
          }
          for (int i = 0; i < WIDTH; i += 5) {
              ssd1306_pixel(ssd, i, 0, true);
              ssd1306_pixel(ssd, i, HEIGHT - 1, true);
          }
          break;

      case BORDER_DUPLA:
          for (int i = 0; i < system_state.border_size; i += 2) {
              ssd1306_rect(ssd, i, i, WIDTH - (2 * i), HEIGHT - (2 * i), true, false);
          }
          for (int i = system_state.border_size + 4; i < system_state.border_size + 8; i += 2) {
              ssd1306_rect(ssd, i, i, WIDTH - (2 * i), HEIGHT - (2 * i), true, false);
          }
          break;

      case BORDER_TRACEJADA:
          for (int i = 0; i < HEIGHT; i += 8) {
              for (int j = 0; j < 4; j++) { // Segmento de 4 pixels
                  ssd1306_pixel(ssd, 0, i + j, true);
                  ssd1306_pixel(ssd, WIDTH - 1, i + j, true);
              }
          }
          for (int i = 0; i < WIDTH; i += 8) {
              for (int j = 0; j < 4; j++) { // Segmento de 4 pixels
                  ssd1306_pixel(ssd, i + j, 0, true);
                  ssd1306_pixel(ssd, i + j, HEIGHT - 1, true);
              }
          }
          break;

      default:
          break;
  }
}


// Atualiza o display com a posição do joystick e a borda
void update_display(ssd1306_t *ssd, uint8_t pos_x, uint8_t pos_y) {
    ssd1306_fill(ssd, false);
    ssd1306_rect(ssd, pos_y, pos_x, 8, 8, true, true);
    change_border(ssd);
    ssd1306_send_data(ssd);
}

// Atualiza os LEDs com base nos valores do joystick
void update_leds(uint16_t adc_x, uint16_t adc_y, SystemState *state) {
    uint16_t pwm_x = 0;
    uint16_t pwm_y = 0;
    
    if (abs(adc_x - 2048) > DEADZONE) {
        pwm_x = state->led_active ? map(abs(adc_x - 2048), DEADZONE, 2048, 0, PWM_WRAP) : 0;
        change_led(LED_RED, pwm_x);
    } else {
        change_led(LED_RED, 0);
    }
    
    if (abs(adc_y - 2048) > DEADZONE) {
        pwm_y = state->led_active ? map(abs(adc_y - 2048), DEADZONE, 2048, 0, PWM_WRAP) : 0;
        change_led(LED_BLUE, pwm_y);
    } else {
        change_led(LED_BLUE, 0);
    }
}

// Manipulador de interrupção GPIO
void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    if (gpio == JOYSTICK_PB_PIN && (current_time - system_state.last_joystick_press_time > DEBOUNCE)) {
      system_state.last_joystick_press_time = current_time;
  
      // Alterna estado do LED verde
      system_state.led_green_status = !system_state.led_green_status; 
      gpio_put(LED_GREEN, system_state.led_green_status);
      
      // Alterna entre os estilos de borda
      system_state.border_type = (system_state.border_type + 1) % BORDER_MAX;
  }
   
    else if (gpio == BUTTON_A && (current_time - system_state.last_button_a_press_time > DEBOUNCE)) {
        system_state.last_button_a_press_time = current_time;
        system_state.led_active = !system_state.led_active;
    }
}