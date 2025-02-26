#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "dataStructure.h"
#include "mpu6050_handler.h"
#include "serial.h"
#include "motor_handler.h"
#include "control_handler.h"

#define LOG_BUFFER_SIZE 512
#define RUN_TIME_BUFFER_SIZE 512
float tof_r, tof_l, accel_x, accel_y, accel_z, angle, current;
float current_tof_r, current_tof_l, current_accel_x, current_accel_y, current_accel_z, current_angle, current_current;
void vSystemLogTask(void *pvParameters);
void vInitializeMotors();
void vInitializeStructParams();

// Definição do heap para o FreeRTOS
__attribute__((section(".rtos_heap"))) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

#define QUEUE_SIZE 1 // Define o tamanho da fila (pode ser ajustado conforme necessário)

QueueHandle_t xMailbox;
QueueHandle_t xSendEsp;
QueueHandle_t xReceiveEsp;
QueueHandle_t xLeftMotor;
QueueHandle_t xRightMotor;

mpu6050 sensor(i2c0, 0, 1);
Motor left_motor(300, 15);
Motor right_motor(300, 14);

mpu_params mpu;
motor_params left_motor_params;
motor_params right_motor_params;
PIDParams_t pid_params;

int main() {
    stdio_init_all(); 
    setup_uart();  
    //cria o Mailbox pra cada sensor
    xMailbox = xQueueCreate(1, sizeof(MpuMailbox_t));
    xLeftMotor = xQueueCreate(10, sizeof(int));
    xRightMotor = xQueueCreate(10, sizeof(int));
    xSendEsp = xQueueCreate(5, sizeof(char[MESSAGE_MAX_LENGTH]));
    xReceiveEsp = xQueueCreate(5, sizeof(char[MESSAGE_MAX_LENGTH]));
    PMEParams_t pmeParams;
    pmeParams.receive = xReceiveEsp;
    pmeParams.send = xSendEsp;

    char buffer[0xffff];

    vInitializeStructParams();
    vInitializeMotors();
    

    //cria as tarefas de ler o sensor e adicionar o dado no mailbox
    // e a outra de ler do mailbox pra testar se ta ok a comunicação entre as tasks
    xTaskCreate(vTaskReadMpu, "Task MPU read", 256, &mpu, 2, NULL); // Tarefa 1 com prioridade 1
    xTaskCreate(vTaskPrintMpu, "Task MPU print", 256, &mpu, 2, NULL); // Tarefa 2 com prioridade 1
    xTaskCreate(vTaskProcessMessageEsp,"menssage print", 1000, &pmeParams, 2, NULL);
    xTaskCreate(vTaskSendMessage,"menssage envio", 1000, xSendEsp, 2, NULL);
    xTaskCreate(vTaskReceive,"menssage receive", 1000, xReceiveEsp, 2, NULL);
    xTaskCreate(vTaskGenerateMessage,"envio esp", 1000, xSendEsp, 2, NULL);
    xTaskCreate(vSystemLogTask,"Task Log", 256, buffer, 1, NULL);
    xTaskCreate(vTaskMotorControl, "Left Motor Task", 1000, &left_motor_params, 1, NULL);
    xTaskCreate(vTaskMotorControl, "Right Motor Task", 1000, &right_motor_params, 1, NULL);
    xTaskCreate(potentiometerTask, "Potentiometer Task", 1000, xLeftMotor, 2, NULL);
    //xTaskCreate(vPIDParametersTask, "PID Task", 1000, &pid_params, 3, NULL);

    //escalona as tarefas
    vTaskStartScheduler();

    while (1) {
        printf("Falha ao iniciar o scheduler do FreeRTOS.\n");
    }
    return 0;
}

void vSystemLogTask(void *pvParameters) {
    char* pcWrite = (char*)pvParameters;
    const TickType_t logInterval = pdMS_TO_TICKS(4000); // Log a cada 4 segundos

    while (true) {
        printf("\n================= ESTADO DO SISTEMA =================\n");

        // Log do uso de memória heap
        size_t freeHeap = xPortGetFreeHeapSize();
        size_t minHeap = xPortGetMinimumEverFreeHeapSize();
        printf("Uso de Memória:\n");
        printf("Heap Livre: %lu bytes\n", freeHeap);
        printf("Heap Mínimo Já Registrado: %lu bytes\n\n", minHeap);

        // Log do estado das tasks
        printf("Estado das Tasks:\n");
        printf("Task Name\tStatus\tPri\tStack\tTask\tCoreAf#\n");
        vTaskList(pcWrite);
        printf("%s\n", pcWrite);

        // Estatísticas de tempo de execução
#if configGENERATE_RUN_TIME_STATS
        printf("Tempo de Execução das Tasks:\n");
        vTaskGetRunTimeStats(pcWrite);
        printf("%s\n", pcWrite);
#endif

        printf("=====================================================\n");

        // Delay para o próximo log
        vTaskDelay(logInterval);
    }
}

void vInitializeMotors() {
    sensor.mpu6050_init();

    left_motor.vInitMotors();
    right_motor.vInitMotors();

    _delay_ms(3000);
    printf("Começou, colocando 40%% como vel padrão\n");
    left_motor.setSpeed(40);
    right_motor.setSpeed(40);
    //sleep_ms(1000);
    _delay_ms(1000);
    printf("acabei a função de init \n");
}

void vInitializeStructParams() {
    vQueueAddToRegistry(xLeftMotor, "Left Motor");
    vQueueAddToRegistry(xRightMotor, "Right Motor");

    mpu.sensor = &sensor;
    mpu.mailbox = xMailbox;

    left_motor_params.motor = &left_motor;
    left_motor_params.mailbox = xLeftMotor;

    right_motor_params.motor = &right_motor;
    right_motor_params.mailbox = xRightMotor;

    pid_params.leftMotorQueue = xLeftMotor;
    pid_params.rightMotorQueue = xRightMotor;
    pid_params.mpuMailbox = xMailbox;
}