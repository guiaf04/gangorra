#ifndef CONTROL_HANDLER_H
#define CONTROL_HANDLER_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "dataStructure.h"
#include "hardware/adc.h"

#define MIN_DUTY 0   // Duty cycle mínimo em porcentagem
#define MAX_DUTY 100    // Duty cycle máximo em porcentagem


#define throttle 1300
#define desired_angle 0

void potentiometerTask(void *pvParameters);
void vPIDParametersTask(void *pvParameters);

#endif //CONTROL_HANDLER_H