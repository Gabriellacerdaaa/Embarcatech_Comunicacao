# Projeto: Interface e Controle de Periféricos com RP2040 e BitDogLab 📌

## 1. Descrição Geral

Este projeto demonstra a integração de múltiplos periféricos utilizando a placa de desenvolvimento **BitDogLab** (baseada no microcontrolador **RP2040**). O sistema utiliza:

- **Display OLED SSD1306** via I2C para exibição de mensagens e informações.
- **Matriz de LEDs** (configurada como 5×5 pixels) controlada via PIO, para exibição de padrões numéricos.
- **LEDs RGB** para feedback visual (LED verde e azul).
- **Botões com interrupção e debouncing** para interação (Botão A e Botão B).
- **Entrada Serial USB** para receber caracteres e, se for dígito (0–9), desenhar o padrão correspondente na matriz de LEDs.

## 2. Funcionalidades

- **Exibição no Display OLED:**
  - Mensagem inicial com identificação do projeto.
  - Atualização dinâmica do display conforme a interação dos botões e entrada serial.
  
- **Controle da Matriz de LEDs:**
  - Exibição de padrões numéricos definidos em uma matriz 5×5 (usando 0.0 para desligado e 1.0 para ligado).
  - Conversão de intensidade para o formato RGB de 32 bits.

- **Interrupções e Debouncing:**
  - Botão A (GPIO 5): Alterna o estado do LED verde e atualiza o display com "LED VERDE ON/OFF".
  - Botão B (GPIO 6): Alterna o estado do LED azul e atualiza o display com "LED AZUL ON/OFF".
  - Implementação de debouncing (mínimo de 200 ms entre eventos) para evitar leituras incorretas.

- **Comunicação Serial USB:**
  - Recepção de caracteres via Serial Monitor.
  - Desenho do padrão correspondente na matriz de LEDs ao receber dígitos (0–9).

## 3. Requisitos de Hardware

- **Placa BitDogLab** com microcontrolador RP2040.
- **Display OLED SSD1306**  
  - I2C: SDA no GPIO 14, SCL no GPIO 15.
- **Matriz de LEDs** (WS2812 ou similar) conectada ao GPIO 7.
- **LED RGB:**  
  - LED verde: GPIO 11.  
  - LED azul: GPIO 12.
- **Botões:**  
  - Botão A: GPIO 5.  
  - Botão B: GPIO 6.

## 4. Requisitos de Software

- **Pico SDK** para desenvolvimento com RP2040.
- **CMake** e **GNU Arm Toolchain** para compilação.
- **VS Code** (ou outro editor de sua preferência).
- Bibliotecas utilizadas:
  - `ssd1306.h` – controle do display OLED.
  - `pio_matrix.pio.h` – controle da matriz de LEDs via PIO.
  - Bibliotecas nativas do Pico (pico/stdlib, hardware/clocks, hardware/i2c, etc.).

## 5. Estrutura do Projeto

- **main.c:** Contém o código principal que:
  - Inicializa o sistema (clock, GPIO, I2C, PIO, display).
  - Define os padrões de exibição para a matriz de LEDs (array `numeros`).
  - Converte valores de intensidade para o formato RGB (função `matrix_rgb`).
  - Desenha padrões na matriz de LEDs (função `desenho_pio`).
  - Trata interrupções dos botões (função `gpio_irq_handler_BOTAO`).
  - Processa a entrada serial e atualiza a exibição.

## 6. Instruções de Instalação e Execução

### 6.1 Ambiente de Desenvolvimento

1. **Instale o Pico SDK:**  
   Siga as instruções oficiais do [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk).

2. **Configure o Ambiente:**  
   Utilize o VS Code (ou outro editor) e certifique-se de que CMake e o GNU Arm Toolchain estão instalados.

### 6.2 Compilação

1. Crie um diretório de build e entre nele:
   ```bash
   mkdir build && cd build
   ```
2. Execute o CMake para configurar o projeto:
   ```bash
   cmake ..
   ```
3. Compile o projeto:
   ```bash
   make
   ```

### 6.3 Upload do Firmware

1. Conecte a placa **BitDogLab** ao computador.
2. Coloque a placa em modo bootloader (por exemplo, segurando um botão ou via comando específico).
3. Transfira o arquivo UF2 gerado para a unidade USB do RP2040 (drag & drop).

### 6.4 Testando o Projeto

- Abra o **Serial Monitor** (certifique-se da taxa de transmissão correta).
- Envie caracteres via Serial Monitor e interaja com os botões para verificar a atualização dos LEDs e do display.

## 7. Demonstração

Recomenda-se a produção de um vídeo demonstrativo (até 2 minutos), que inclua:
- A exibição inicial e as mensagens no display OLED.
- A alternância dos LEDs (verde e azul) via botões.
- A atualização dos padrões na matriz de LEDs ao receber dígitos via Serial Monitor.

## 8. Contribuições

Contribuições são bem-vindas! Para colaborar:
- Faça um fork deste repositório.
- Crie uma branch para sua funcionalidade.
- Envie um pull request documentando suas alterações.

## 9. Licença

Este projeto está licenciado sob a [MIT License](LICENSE). Consulte o arquivo LICENSE para mais detalhes.

## 10. Contato

- **Autor:** Gabriel Lacerda
- **Email:** gabriel.lacerda0563@gmail.com

