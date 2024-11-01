#include "servo.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "FreeRTOS.h"
#include "task.h"

// Definição dos pinos dos servos e do joystick
#define SERVO_PIN_ONE 2
#define SERVO_PIN_TWO 3
#define JOYSTICK_X_PIN 26 // Joystick X no ADC0 (GPIO 26)
#define JOYSTICK_Y_PIN 27 // Joystick Y no ADC1 (GPIO 27)

// Variáveis para controlar a posição dos servos
int currentMillisOne = 1600; // Posição inicial do servo 1 (valor médio)
int currentMillisTwo = 1600; // Posição inicial do servo 2 (valor médio)
const int servoMin = 400;
const int servoMax = 2400;
const int joystickMin = 0;     // Valor mínimo do joystick (0)
const int joystickMax = 4095;  // Valor máximo do joystick (4095)

// Definições para a conversão do joystick
#define MAX_VALUE 4095       // Valor máximo do ADC para 12 bits
#define SCALE_FACTOR 255     // Escala para mapeamento de -255 a +255
#define DEAD_ZONE 100         // Tamanho da zona morta
#define MAX_SERVO_INCREMENT 10 // Incremento máximo por leitura do joystick

// Funções de configuração e utilitárias
void setup() {
    // Configuração dos pinos dos servos
    setServo(SERVO_PIN_ONE, currentMillisOne);
    setServo(SERVO_PIN_TWO, currentMillisTwo);

    // Inicialização do ADC para leitura analógica
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);

    // Inicialização da UART para envio de dados do sensor
    uart_init(uart0, 9600);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
}

int readJoystick(int channel) {
    adc_select_input(channel);
    return adc_read();
}

// Função para converter valor do joystick em movimento (-255 a +255)
int converter_valor(int adc_value) {
    int valor = ((adc_value - (MAX_VALUE / 2)) * SCALE_FACTOR) / (MAX_VALUE / 2);

    if (valor > -DEAD_ZONE && valor < DEAD_ZONE) {
        return 0; // Retorna 0 se o valor estiver dentro da zona morta
    }
    return valor;
}

// Mapeamento de valores
int mapValue(int value, int in_min, int in_max, int out_min, int out_max) {
    if (value < in_min) value = in_min;
    if (value > in_max) value = in_max;
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Tarefas FreeRTOS
void vTaskServoControl(void *pvParameters) {
    while (1) {
        // Leitura dos valores do joystick
        int xValue = readJoystick(0); // Eixo X do joystick (canal 0)
        int yValue = readJoystick(1); // Eixo Y do joystick (canal 1)
        printf("X: %d, Y: %d\n", xValue, yValue);

        // Conversão dos valores do joystick
        int xMovement = converter_valor(xValue);
        int yMovement = converter_valor(yValue);
        printf("X modificado: %d, Y modificado: %d\n", xMovement, yMovement);

        // Ajustando a posição do servo de forma suave
        if (xMovement != 0) {
            currentMillisOne += (xMovement / 255.0) * MAX_SERVO_INCREMENT;
            currentMillisOne = mapValue(currentMillisOne, servoMin, servoMax, servoMin, servoMax);
        }

        if (yMovement != 0) {
            currentMillisTwo += (yMovement / 255.0) * MAX_SERVO_INCREMENT;
            currentMillisTwo = mapValue(currentMillisTwo, servoMin, servoMax, servoMin, servoMax);
        }

        // Definindo a posição dos servos
        setMillis(SERVO_PIN_ONE, currentMillisOne);
        setMillis(SERVO_PIN_TWO, currentMillisTwo);

        vTaskDelay(pdMS_TO_TICKS(10)); // Delay de 10 ms
    }
}

void vTaskSensorRead(void *pvParameters) {
    while (1) {
        // Leitura do sensor adicional
        adc_select_input(2); // Leitura do sensor no canal 2
        int sensorValue = adc_read();

        // Processamento (aqui simplificado, mas pode incluir filtragem, etc.)
        char sensorData[20];
        snprintf(sensorData, sizeof(sensorData), "Sensor: %d\r\n", sensorValue);

        // Envio dos dados do sensor via UART
        uart_puts(uart0, sensorData);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay de 1 segundo
    }
}

int main() {
    setup();

    // Criação das tarefas
    xTaskCreate(vTaskServoControl, "ServoControl", 256, NULL, 1, NULL);
    xTaskCreate(vTaskSensorRead, "SensorRead", 256, NULL, 1, NULL);

    // Início do agendador do FreeRTOS
    vTaskStartScheduler();

    // O código não deve chegar aqui
    while (1) {}
}