// Tutorial :
// Use board "ESP32 Dev Module" (last tested on v3.3.2)

#include "JPEGDEC.h"
#include <SD_MMC.h>   // Included with the Espressif Arduino Core
#include "amoled.h"   // Display driver, included in the project
#include "FT3168.h"   // Capacitive Touch functions, included in the project
#include "qmi8658c.h" // QMI8658 6-axis IMU (3-axis accelerometer and 3-axis gyroscope) functions
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Amoled amoled; // Main object for the display

// Touch global variables
volatile uint16_t touchX = 0;
volatile uint16_t touchY = 0;
volatile bool skipVideo = false;
static TaskHandle_t touchTaskHandle = nullptr;

// Global to store the latest sample read the accelerometer and gyroscope (QMI8658)
typedef struct
{
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
} ImuData;
volatile ImuData g_imu;
#define READ_SAMPLE_INTERVAL_MS 50     // Interval in ms to read a sample from the QMI8658

static void touchTask(void *pvParameter);

void setup()
{
    Serial.begin(115200);
    delay(4000); // Give time to the serial port to show initial messages printed on the serial port upon reset

    // Display initialization
    if (!amoled.begin())
    {
        Serial.println("Display initialization failed!");
        while (true)
        {
            /* no need to continue */
        }
    }
    amoled.fillScreen(AMOLED_COLOR_BLACK);
    Serial.printf("Display controller name is %s (id=%d)\n", amoled.name(), amoled.ID());

    Touch_Init(); // Init the Touch Controller

    // Initialize QMI8658 6-axis IMU
    Serial.println("QMI8658 6-axis IMU initialization");
    qmi8658_init();
    delay(1000); // Do not change this delay, it is required by the QMI8658

    // Create a task to read touch events
    if (xTaskCreatePinnedToCore(
            touchTask,
            "TouchTask",
            4096,
            nullptr,
            1,
            &touchTaskHandle,
            0) != pdPASS)
    {
        Serial.println("ERROR: Failed to create TouchTask");
    }
    // Create the task to read QMI8658 6-axis IMU (3-axis accelerometer and 3-axis gyroscope)
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, NULL, 2, NULL, 0);
}

void loop()
{
}

// Task to read the values of QMI8658 6-axis IMU (3-axis accelerometer and 3-axis gyroscope)
static void imu_task(void *arg)
{
    for (;;)
    {
        float acc[3], gyro[3];
        float temp = 0;
        qmi8658_read_xyz(acc, gyro);
        temp = qmi8658_readTemp();
        g_imu.ax = acc[0];
        g_imu.ay = acc[1];
        g_imu.az = acc[2];
        g_imu.gx = gyro[0];
        g_imu.gy = gyro[1];
        g_imu.gz = gyro[2];
        g_imu.temp = temp;

        vTaskDelay(pdMS_TO_TICKS(READ_SAMPLE_INTERVAL_MS));
    }
}

// Task to read touche events
static void touchTask(void *pvParameter)
{
    (void)pvParameter;
    uint16_t detectedX = 0;
    uint16_t detectedY = 0;

    for (;;)
    {
        if (getTouch(&detectedX, &detectedY))
        {
            touchX = detectedX;
            touchY = detectedY;
            if (!skipVideo)
            {
                skipVideo = true;
                Serial.printf("Touch detected at (%u, %u) - skipping video\n", detectedX, detectedY);
            }

            // Wait for finger release before resuming detection
            do
            {
                vTaskDelay(pdMS_TO_TICKS(50));
            } while (getTouch(&detectedX, &detectedY));

            touchX = 0;
            touchY = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
