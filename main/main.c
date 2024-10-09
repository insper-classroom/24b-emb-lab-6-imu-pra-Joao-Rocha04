#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include <Fusion.h>
#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
QueueHandle_t xQueueAdc;
typedef struct adc
{
    int axis;
    int val;
} adc_t;


static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    adc_t adc;

    while(1) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        //printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        //printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        //printf("Temp. = %f\n", (temp / 340.0) + 36.53);
        
        FusionVector gyroscope = {
        .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
        .axis.y = gyro[1] / 131.0f,
        .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        euler.angle.roll = -euler.angle.roll;
        //float alpha = 0.9;  // Constante de suavização (ajustável)
        //static float filtered_roll = 0;
        //static float filtered_pitch = 0;
        //filtered_roll = alpha * filtered_roll + (1.0 - alpha) * euler.angle.roll;
        //filtered_pitch = alpha * filtered_pitch + (1.0 - alpha) * euler.angle.pitch;

        euler.angle.roll = fminf(fmaxf(euler.angle.roll, -30.0f), 30.0f);
        euler.angle.pitch = fminf(fmaxf(euler.angle.pitch, -30.0f), 30.0f);

        

        
        //if (euler.angle.roll > 30 || euler.angle.roll< -30){
        //    adc.axis = 0;
        //    adc.val = (int) euler.angle.roll;
        //    xQueueSend(xQueueAdc,&adc,portMAX_DELAY);
        //    vTaskDelay(pdMS_TO_TICKS(10));
        //}
        adc.axis = 1;
        adc.val = (int) euler.angle.roll;
        xQueueSend(xQueueAdc,&adc,portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10));
        //if (euler.angle.pitch > 30 || euler.angle.pitch < -30){
        //    adc.axis = 1;
        //    adc.val = (int) euler.angle.pitch;
        //    xQueueSend(xQueueAdc,&adc,portMAX_DELAY);
        //    vTaskDelay(pdMS_TO_TICKS(10));
        //}
        adc.axis = 0;
        adc.val = (int) euler.angle.pitch;
        xQueueSend(xQueueAdc,&adc,portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10));

        if (accelerometer.axis.z > 1.5f) {
            adc.axis = 2;
            adc.val = (int)(accelerometer.axis.z * 1000);
            xQueueSend(xQueueAdc, &adc, portMAX_DELAY);
        }
        
        //printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}
void uart_task(void *p) {
    adc_t data;

    while (1) {
        xQueueReceive(xQueueAdc, &data, portMAX_DELAY);
        //printf("Axis: %d, Value: %d\n", data.axis, data.val);
        // Axis (1 para Y, 0 para X)
        u_int8_t axis = data.axis;
        putchar(axis);

        int16_t val = data.val;

        u_int8_t val_1 = (val >> 8) & 0xFF;  
        putchar(val_1);
        u_int8_t val_0 = val & 0xFF;         
        putchar(val_0);

        u_int8_t eop = 0xFF;
        putchar(eop);

    }
}

int main() {
    stdio_init_all();
    xQueueAdc = xQueueCreate(32, sizeof(adc_t));
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 1, NULL);
    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
