#include "mpu6050_handler.h"

#include <kalman_filter.h>

void vTaskReadMpu(void *pvParameters) {
    mpu_params* mpu = (mpu_params*)pvParameters;
    QueueHandle_t mailbox = mpu->mailbox;
    mpu6050* sensor = mpu->sensor;

    MpuMailbox_t mailboxData;
    KalmanFilter kalman_filter;
    constexpr float df = 1; // Intervalo de tempo em segundos (50 ms)

    sensor->getAccel(&mailboxData.mpu.accelData);
    sensor->getGyro(&mailboxData.mpu.gyroData);
    mailboxData.xTimeStamp = xTaskGetTickCount();
    kalman_filter.getAngleWithEKF(
        mailboxData.mpu.accelData,
        mailboxData.mpu.gyroData,
        mailboxData.angle,
        df
    );

    while (1) {
        // Envia uma mensagem para a mailbox
        if (xQueueOverwrite(mailbox, &mailboxData) == pdPASS) {
            printf("Producer: MPU Send\n");
        } else {
            printf("Producer: Falha ao enviar para a mailbox\n");
        }
        // Aguarda 1 segundo antes de enviar a próxima mensagem
        vTaskDelay(pdMS_TO_TICKS(1000));
        sensor->getAccel(&mailboxData.mpu.accelData);
        sensor->getGyro(&mailboxData.mpu.gyroData);
        mailboxData.xTimeStamp = xTaskGetTickCount();
        kalman_filter.getAngleWithEKF(
            mailboxData.mpu.accelData,
            mailboxData.mpu.gyroData,
            mailboxData.angle,
            df
        );
    }
}

// Implementação da Tarefa 2
void vTaskPrintMpu(void *pvParameters) {
    mpu_params* mpu = (mpu_params*)pvParameters;
    QueueHandle_t mailbox = mpu->mailbox;
    mpu6050* sensor = mpu->sensor;

    MpuMailbox_t receivedMessage;
    while (1) {
        // Lê o valor da mailbox (aguarda indefinidamente até receber)
        if (xQueueReceive(mailbox, &receivedMessage, portMAX_DELAY) == pdPASS) {
            printf("Dados do MPU6050\n");
            sensor->print_raw_data(receivedMessage.mpu.accelData, receivedMessage.mpu.gyroData);
            printf("Filtered Angle: %.2f\n", receivedMessage.angle);
            printf("TimeStamp: %d\n\n", receivedMessage.xTimeStamp);
        } else {
            printf("Consumer: Falha ao receber da mailbox\n");
        }

        // Aguarda 500 ms antes de tentar novamente
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}