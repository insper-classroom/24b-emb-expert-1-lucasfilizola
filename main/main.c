/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "servo.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

#define SERVO_PIN_X 16
#define SERVO_PIN_Y 15
#define JOYSTICK_X_CHANNEL 0 // Canal do ADC para eixo X do joystick
#define JOYSTICK_Y_CHANNEL 1 // Canal do ADC para eixo Y do joystick


// Função para inicializar UART
void init_uart() {
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
}

// Função para configurar ADC
void init_adc() {
    adc_init();
    adc_gpio_init(26); // GPIO correspondente ao ADC0
    adc_gpio_init(27); // GPIO correspondente ao ADC1
}

// Variáveis globais para armazenar os valores atuais dos servos
int currentMillisX = 1500;
int currentMillisY = 1500;


// Tarefa para ler o joystick e ajustar servos
void joystick_task(void *params) {

    while (1) {
        // Leitura do Joystick
        adc_select_input(JOYSTICK_X_CHANNEL);
        uint16_t x_value = adc_read();

        x_value = ((x_value - 2047) * 255) / 2048;

        //if (x_value > -120 && x_value < 120) {
            //x_value = 0; 
        //}
        
        adc_select_input(JOYSTICK_Y_CHANNEL);
        uint16_t y_value = adc_read();

        y_value = ((y_value - 2047) * 255) / 2048;

        //if (y_value > -120 && y_value < 120) {
            //y_value = 0; 
        //}
        
        // Mapear o valor lido (0-4095) para pulsos (400-2400)
        currentMillisX = 400 + (x_value * 2000 / 4095);
        currentMillisY = 400 + (y_value * 2000 / 4095);

        // Controlar os servos
        setServo(SERVO_PIN_X, currentMillisX);
        setServo(SERVO_PIN_Y, currentMillisY);

        vTaskDelay(pdMS_TO_TICKS(20)); // Atualiza a cada 20ms
    }
}

// Tarefa para enviar dados via UART
void uart_task(void *params) {
    char uart_buffer[50];
    while (1) {
        // Aqui processamos a informação dos valores atuais dos servos
        snprintf(uart_buffer, sizeof(uart_buffer), "Servo X: %d, Servo Y: %d\n", currentMillisX, currentMillisY);
        uart_puts(uart0, uart_buffer);

        vTaskDelay(pdMS_TO_TICKS(100)); // Envia a cada 100ms
    }
}

int main() {
    stdio_init_all();
    init_uart();
    init_adc();

    // Criação das tarefas
    xTaskCreate(joystick_task, "Joystick Control", 256, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART Communication", 256, NULL, 1, NULL);

    // Início do scheduler
    vTaskStartScheduler();

    while (1) {
        // Não deve chegar aqui
    }

    return 0;
}