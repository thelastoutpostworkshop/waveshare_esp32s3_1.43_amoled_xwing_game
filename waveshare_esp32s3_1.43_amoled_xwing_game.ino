// Tutorial :
// Use board "ESP32 Dev Module" (last tested on v3.3.2)

#include <SD_MMC.h>     // Included with the Espressif Arduino Core
#include "amoled.h"     // Display driver, included in the project
#include "FT3168.h"     // Capacitive Touch functions, included in the project
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Amoled amoled; // Main object for the display

// Touch global variables
volatile uint16_t touchX = 0;
volatile uint16_t touchY = 0;
volatile bool skipVideo = false;
static TaskHandle_t touchTaskHandle = nullptr;

static void touchTask(void *pvParameter);

void setup()
{
    Serial.begin(115200);
    delay(4000); // Give time to the serial port to show initial messages printed on the serial port upon reset

    // SD Card initialization
    SD_MMC.setPins(SD_CLK, SD_MOSI, SD_MISO);
    if (!SD_MMC.begin("/sdcard", true, false))
    {
        Serial.println("ERROR: SD Card mount failed!");
        while (true)
        {
            /* no need to continue */
        }
    }
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

    Touch_Init();   // Init the Touch Controller

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

}

void loop()
{

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
