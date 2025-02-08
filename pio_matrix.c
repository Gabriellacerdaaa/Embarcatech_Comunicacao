#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"         // Biblioteca padrão para Raspberry Pi Pico
#include "hardware/pio.h"        // Funções para a interface PIO (Programmable I/O)
#include "hardware/clocks.h"     // Funções para manipulação dos clocks do sistema
#include "hardware/adc.h"        // Acesso ao ADC (Conversor Analógico-Digital)
#include "pico/bootrom.h"        // Funções para operações relacionadas à ROM de boot
#include "hardware/i2c.h"        // Funções para comunicação via I2C
#include "font.h"                // Biblioteca de fontes para o display (possivelmente customizada)
#include "ssd1306.h"             // Biblioteca para controle do display OLED SSD1306

// Definições de constantes e pinos utilizados
#define I2C_PORT i2c1          // Seleciona a interface I2C1
#define I2C_SDA 14             // Pino para SDA da interface I2C
#define I2C_SCL 15             // Pino para SCL da interface I2C
#define endereco 0x3C          // Endereço I2C do display SSD1306
#define FRAMES 10              // Número de frames (padrões) para a matriz de LEDs
#define NUM_PIXELS 25          // Número total de pixels (para matriz 5x5)
#define LEDG 11                // Pino do LED verde (LED RGB)
#define LEDB 12                // Pino do LED azul (LED RGB)
#define BOTAOA 5               // Pino do Botão A
#define BOTAOB 6               // Pino do Botão B
#define OUT_PIN 7              // Pino de saída para a matriz de LEDs (WS2812)

// Variáveis globais
PIO pio = pio0;                // Utiliza a instância pio0 para o controle dos LEDs
bool ok;                       // Flag para verificar se a configuração do clock foi bem-sucedida
uint16_t i;                    // Variável auxiliar (pode ser utilizada em loops)
uint32_t valor_led;            // Armazena o valor formatado de cor (32 bits) para cada LED
uint sm;                       // Identificador da state machine utilizada pelo PIO
uint offset;                   // Offset do programa PIO carregado na memória
double r = 0.0, b = 0.0, g = 0.0;  // Variáveis para controle de intensidade das cores (Red, Blue, Green)

// Instância da estrutura do display SSD1306
ssd1306_t ssd;               

#include "pio_matrix.pio.h"    // Inclusão do código PIO para controle da matriz de LEDs

// Variável para debouncing: registra o tempo (em microssegundos) do último evento de botão
static volatile uint32_t lastEventButton = 0;

// Define os padrões para os dígitos, utilizando uma matriz de 10 frames com 25 pixels cada.
// Cada valor (0.0 ou 1.0) indica se o pixel estará apagado ou aceso, respectivamente.
double numeros[FRAMES][NUM_PIXELS] =
{ 
  {
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0
  },

  {
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0
  },

  {
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0
  },

  {
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0
  },

  {
    0.0, 1.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0
  },

  {
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0
  },

  {
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0
  },

  {
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0
  },

  {
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0
  },

  {
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 1.0, 1.0, 1.0, 0.0
  }
};

// Função para definir a intensidade de cada canal de cor e converter para um valor de 32 bits
// Cada canal (R, G, B) é multiplicado por 255 para converter o valor double (0.0 a 1.0) para um valor inteiro (0 a 255)
// A função empacota os valores de cor em um único inteiro de 32 bits com a ordem: G (mais significativo), R e B
uint32_t matrix_rgb(double b, double r, double g)
{
    unsigned char R, G, B;
    R = r * 255;
    G = g * 255;
    B = b * 255;
    return (G << 24) | (R << 16) | (B << 8);
}

// Função que desenha um número na matriz de LEDs utilizando o PIO
// Parâmetros:
//   desenho: ponteiro para o array que contém o padrão (frame) a ser exibido
//   valor_led: variável que armazenará o valor da cor formatado em 32 bits
//   pio: instância do PIO utilizada
//   sm: identificador da state machine utilizada
//   r, g, b: intensidades das cores (neste caso, os valores são redefinidos para 0.0 na chamada da função matrix_rgb)
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b)
{
    // Array que define a ordem física dos LEDs na matriz (para espelhamento, pois a imagem aparece invertida)
    int ordem_fisica[NUM_PIXELS] = 
    {
        24, 23, 22, 21, 20, 
        15, 16, 17, 18, 19, 
        14, 13, 12, 11, 10,
        5, 6, 7, 8, 9,     
        4, 3, 2, 1, 0       
    };

    // Loop que percorre todos os pixels da matriz
    for (int16_t i = 0; i < NUM_PIXELS; i++)
    {
        int indice_fisico = ordem_fisica[i];  // Obtém o índice físico do LED a partir da ordem definida
        // Converte o valor do pixel (do array 'desenho') para uma cor 32 bits.
        // Aqui, r e g são fixados como 0.0, definindo uma cor baseada somente no valor de 'b' passado pelo desenho.
        valor_led = matrix_rgb(desenho[indice_fisico], r = 0.0, g = 0.0);
        // Envia o valor da cor para a state machine do PIO de forma bloqueante
        pio_sm_put_blocking(pio, sm, valor_led);
    }
}

// Função de interrupção para tratamento dos botões (com debouncing)
// Essa função é chamada quando ocorre um evento de borda (queda) nos pinos dos botões
void gpio_irq_handler_BOTAO(uint gpio, uint32_t events)
{
    // Obtém o tempo atual em microssegundos
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    // Verifica se passaram pelo menos 200 ms desde o último evento (debouncing)
    if (current_time - lastEventButton > 200000)
    {
        lastEventButton = current_time;  // Atualiza o tempo do último evento
        
        // Se o evento ocorreu no pino do Botão A
        if(gpio == BOTAOA)
        {   
              // Alterna o estado do LED verde
              gpio_put(LEDG, !(gpio_get(LEDG)));

              // Se o LED verde está ligado, atualiza o display para indicar "LED VERDE ON"
              if(gpio_get(LEDG))
              {
                ssd1306_fill(&ssd, false);
                ssd1306_draw_string(&ssd, "LED VERDE ON", 18, 30);
                ssd1306_rect(&ssd, 3, 3, 122, 58, 1, 0);
                ssd1306_send_data(&ssd);
                printf("Led verde ligou\n");
              }
              // Caso contrário, atualiza o display para indicar "LED VERDE OFF"
              else
              {
                ssd1306_fill(&ssd, false);
                ssd1306_draw_string(&ssd, "LED VERDE OFF", 18, 30);
                ssd1306_rect(&ssd, 3, 3, 122, 58, 1, 0);
                ssd1306_send_data(&ssd);
                printf("Led verde desligou\n");
              }
        }

        // Se o evento ocorreu no pino do Botão B
        if (gpio == BOTAOB)
        {
              // Alterna o estado do LED azul
              gpio_put(LEDB, !(gpio_get(LEDB)));

              // Se o LED azul está ligado, atualiza o display para indicar "LED AZUL ON"
              if(gpio_get(LEDB))
              {
                ssd1306_fill(&ssd, false);
                ssd1306_draw_string(&ssd, "LED AZUL ON", 18, 30);
                ssd1306_rect(&ssd, 3, 3, 122, 58, 1, 0);
                ssd1306_send_data(&ssd);
                printf("Led azul ligou\n");
              }
              // Caso contrário, atualiza o display para indicar "LED AZUL OFF"
              else
              {
                ssd1306_fill(&ssd, false);
                ssd1306_draw_string(&ssd, "LED AZUL OFF", 28, 30);
                ssd1306_rect(&ssd, 3, 3, 122, 58, 1, 0);
                ssd1306_send_data(&ssd);
                printf("Led azul desligou\n");
              }
        }                                      
    }
}

// Função de inicialização de todos os periféricos e configurações do sistema
void inicializa()
{
  // Exibe mensagem de início da transmissão PIO e, se ok, mostra a frequência do clock
  printf("iniciando a transmissão PIO");
  if (ok) printf("clock set to %ld\n", clock_get_hz(clk_sys));

  // Configuração do PIO:
  // 1. Carrega o programa PIO (definido em pio_matrix_program, presente no cabeçalho pio_matrix.pio.h)
  // 2. Obtém uma state machine disponível
  // 3. Inicializa o programa PIO na state machine com o pino de saída definido (OUT_PIN)
  offset = pio_add_program(pio, &pio_matrix_program);
  sm = pio_claim_unused_sm(pio, true);
  pio_matrix_program_init(pio, sm, offset, OUT_PIN);

  // Inicializa o LED verde: configura o pino, define como saída e garante que inicia desligado
  gpio_init(LEDG);
  gpio_set_dir(LEDG, GPIO_OUT);
  gpio_put(LEDG, false);

  // Inicializa o LED azul: configura o pino, define como saída e garante que inicia desligado
  gpio_init(LEDB);
  gpio_set_dir(LEDB, GPIO_OUT);
  gpio_put(LEDB, false);

  // Configuração do Botão A: inicializa, define como entrada e habilita resistor de pull-up interno
  gpio_init(BOTAOA);
  gpio_set_dir(BOTAOA, GPIO_IN);
  gpio_pull_up(BOTAOA);

  // Configuração do Botão B: inicializa, define como entrada e habilita resistor de pull-up interno
  gpio_init(BOTAOB);
  gpio_set_dir(BOTAOB, GPIO_IN);
  gpio_pull_up(BOTAOB);

  // Inicializa a interface I2C para comunicação com o display SSD1306 (configurado para 400 kHz)
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);

  // Inicializa e configura o display OLED SSD1306
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
  ssd1306_config(&ssd);
  ssd1306_send_data(&ssd);

  // Limpa o display, garantindo que todos os pixels estejam apagados
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  // Configura as interrupções para os botões A e B utilizando a função de callback gpio_irq_handler_BOTAO
  gpio_set_irq_enabled_with_callback(BOTAOA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler_BOTAO);
  gpio_set_irq_enabled_with_callback(BOTAOB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler_BOTAO);
}

// Função principal
int main()
{
  // Configura a frequência do clock do sistema para 128 MHz
  ok = set_sys_clock_khz(128000, false);

  // Inicializa a saída padrão (UART, etc.) para debug e comunicação via USB
  stdio_init_all();

  // Chama a função de inicialização para configurar os periféricos e interfaces
  inicializa();
 
  // Atualiza o display OLED com informações iniciais:
  // 1. Limpa o display
  // 2. Desenha um retângulo e três strings com informações (nome da instituição, projeto e autor)
  ssd1306_fill(&ssd, 0);
  ssd1306_rect(&ssd, 3, 3, 122, 58, 1, 0);
  ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 10);
  ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 30);
  ssd1306_draw_string(&ssd, "  GABRIEL L.", 15, 48);
  ssd1306_send_data(&ssd);
  sleep_ms(1000);  // Aguarda 1 segundo para exibir a mensagem inicial
  
  // Loop principal do programa
  while (true) 
  {
      // Verifica se há conexão via USB (para comunicação serial)
      if (stdio_usb_connected()) 
      {   
          char c;
          // Se um caractere for recebido via Serial Monitor, processa-o
          if (scanf("%c", &c) == 1) 
          {
              printf("RECEBIDO %c\n", c);  // Imprime no console o caractere recebido
              
              // Atualiza o display para mostrar que um caractere foi recebido
              ssd1306_fill(&ssd, false);
              ssd1306_rect(&ssd, 3, 3, 122, 58, 1, 0);
              ssd1306_draw_string(&ssd, " RECEBIDO", 20, 30);
              ssd1306_draw_string(&ssd, &c, 100, 30);
              ssd1306_send_data(&ssd);

              // Se o caractere recebido for um dígito (0-9)
              if (c >= '0' && c <= '9') 
              {
                  int num = c - '0';  // Converte o caractere para o valor numérico correspondente
                  // Chama a função para desenhar o padrão correspondente na matriz de LEDs
                  desenho_pio(numeros[num], valor_led, pio, sm, r, g, b);
              }
          }
      }
  }
}
