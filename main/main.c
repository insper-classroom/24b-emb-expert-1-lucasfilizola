/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "servo.h"



bool direction = true;
int currentMillis = 400;
int servoPin = 0; 

int main()
{
    stdio_init_all();
    setServo(servoPin, currentMillis);
    while (true)
    {
        currentMillis += (direction)?5:-5;
        if (currentMillis >= 2400) direction = false;
        if (currentMillis <= 400) direction = true;
        setMillis(servoPin, currentMillis);
        sleep_ms(10);
    }
}
