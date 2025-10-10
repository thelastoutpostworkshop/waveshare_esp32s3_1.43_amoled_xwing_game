// Tutorial :
// Use board "ESP32 Dev Module" (last tested on v3.3.2)

#include "MjpegClass.h" // Install "JPEGDEC" with the Library Manager (last tested on v1.8.2)
#include <SD_MMC.h>     // Included with the Espressif Arduino Core
#include "amoled.h"     // Display driver, included in the project
#include "FT3168.h"     // Capacitive Touch functions, included in the project
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Amoled amoled; // Main object for the display

const char *MJPEG_FOLDER = "/mjpeg"; // Name of the mjpeg folder on the SD Card

// Storage for files to read on the SD card
#define MAX_FILES 20 // Maximum number of files, adjust as needed
String mjpegFileList[MAX_FILES];
uint32_t mjpegFileSizes[MAX_FILES] = {0}; // Store each GIF file's size in bytes
int mjpegCount = 0;
static int currentMjpegIndex = 0;
static File mjpegFile; // temp gif file holder

// Uncomment the next line to repeat a single video endlessly, use the index number of the video shown on the serial monitor
// #define REPEAT_INDEX 0 

// Global variables for mjpeg
MjpegClass mjpeg;
int total_frames;
unsigned long total_read_video;
unsigned long total_decode_video;
unsigned long total_show_video;
unsigned long start_ms, curr_ms;
uint8_t *mjpeg_buf;
uint8_t *frame_buf;

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

    mjpeg_buf = (uint8_t *)heap_caps_malloc(DISPLAY_WIDTH * DISPLAY_HEIGHT * 2 / 5, MALLOC_CAP_8BIT);
    if (!mjpeg_buf)
    {
        Serial.println("MJPEG Buffer allocation failed!");
        while (true)
        {
            /* no need to continue */
        }
    }
    frame_buf = (uint8_t *)heap_caps_malloc(DISPLAY_WIDTH * DISPLAY_HEIGHT * 2, MALLOC_CAP_8BIT);
    if (!frame_buf)
    {
        Serial.println("Frame Buffer allocation failed!");
        while (true)
        {
            /* no need to continue */
        }
    }

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

    loadMjpegFilesList(); // Load the list of mjpeg to play from the SD card
}

void loop()
{
#ifdef REPEAT_INDEX
    if (REPEAT_INDEX >= 0 && REPEAT_INDEX < mjpegCount)
    {
        Serial.printf("Repeating File Index %d\n", REPEAT_INDEX);
        playSelectedMjpeg(REPEAT_INDEX);
    }
    else
    {
        Serial.printf("File Index %d does not exist\n", REPEAT_INDEX);
    }
#else
    playSelectedMjpeg(currentMjpegIndex);
    currentMjpegIndex++;
    if (currentMjpegIndex >= mjpegCount)
    {
        currentMjpegIndex = 0;
    }
#endif
}

// Play the current mjpeg
void playSelectedMjpeg(int mjpegIndex)
{
    skipVideo = false;
    // Build the full path for the selected mjpeg
    String fullPath = String(MJPEG_FOLDER) + "/" + mjpegFileList[mjpegIndex];
    char mjpegFilename[128];
    fullPath.toCharArray(mjpegFilename, sizeof(mjpegFilename));

    Serial.printf("\nPlaying %s\n", mjpegFilename);
    mjpegPlayFromSDCard(mjpegFilename);
}

// Callback function to draw a JPEG
int jpegDrawCallback(JPEGDRAW *pDraw)
{
    unsigned long s = millis();

    uint8_t *src = (uint8_t *)pDraw->pPixels;

    for (int row = 0; row < pDraw->iHeight; row++)
    {
        // Compute destination offset for this row in the frame_buf
        int dstY = pDraw->y + row;
        if (dstY >= DISPLAY_HEIGHT)
            break;

        uint8_t *dst = frame_buf + (dstY * DISPLAY_WIDTH + pDraw->x) * 2;

        // Copy a single scanline (width in pixels × 2 bytes)
        memcpy(dst, src + row * pDraw->iWidth * 2, pDraw->iWidth * 2);
    }
    total_show_video += millis() - s;

    return 1;
}

// Play a mjpeg stored on the SD card
void mjpegPlayFromSDCard(char *mjpegFilename)
{
    Serial.printf("Opening %s\n", mjpegFilename);
    File mjpegFile = SD_MMC.open(mjpegFilename, "r");

    if (!mjpegFile || mjpegFile.isDirectory())
    {
        Serial.printf("ERROR: Failed to open %s file for reading\n", mjpegFilename);
    }
    else
    {
        Serial.println("MJPEG start");
        amoled.fillScreen(AMOLED_COLOR_BLACK);

        start_ms = millis();
        curr_ms = millis();
        total_frames = 0;
        total_read_video = 0;
        total_decode_video = 0;
        total_show_video = 0;

        mjpeg.setup(
            &mjpegFile, mjpeg_buf, jpegDrawCallback, true /* useBigEndian */,
            0 /* x */, 0 /* y */, DISPLAY_WIDTH /* widthLimit */, DISPLAY_HEIGHT /* heightLimit */);

        while (!skipVideo && mjpegFile.available() && mjpeg.readMjpegBuf())
        {
            // Read video
            total_read_video += millis() - curr_ms;
            curr_ms = millis();

            // Play video
            mjpeg.drawJpg();
            amoled.drawBitmap(0, 0, (uint16_t *)frame_buf, mjpeg.getWidth(), mjpeg.getWidth());
            // amoled.drawBitmap(0, 0, (uint16_t *)frame_buf, DISPLAY_WIDTH, DISPLAY_HEIGHT);

            total_decode_video += millis() - curr_ms;

            curr_ms = millis();
            total_frames++;
        }

        int time_used = millis() - start_ms;
        Serial.println(F("MJPEG end"));
        mjpegFile.close();
        float fps = 1000.0 * total_frames / time_used;
        total_decode_video -= total_show_video;
        Serial.printf("Total frames: %d\n", total_frames);
        Serial.printf("Time used: %d ms\n", time_used);
        Serial.printf("Average FPS: %0.1f\n", fps);
        Serial.printf("Read MJPEG: %lu ms (%0.1f %%)\n", total_read_video, 100.0 * total_read_video / time_used);
        Serial.printf("Decode video: %lu ms (%0.1f %%)\n", total_decode_video, 100.0 * total_decode_video / time_used);
        Serial.printf("Show video: %lu ms (%0.1f %%)\n", total_show_video, 100.0 * total_show_video / time_used);
        Serial.printf("Video size (wxh): %d×%d, scale factor=%d\n", mjpeg.getWidth(), mjpeg.getHeight(), mjpeg.getScale());
    }
    skipVideo = false;
}

// Read the mjpeg file list in the mjpeg folder of the SD card
void loadMjpegFilesList()
{
    File mjpegDir = SD_MMC.open(MJPEG_FOLDER);
    if (!mjpegDir)
    {
        Serial.printf("Failed to open %s folder\n", MJPEG_FOLDER);
        while (true)
        {
            /* code */
        }
    }
    mjpegCount = 0;
    while (true)
    {
        File file = mjpegDir.openNextFile();
        if (!file)
            break;
        if (!file.isDirectory())
        {
            String name = file.name();
            if (name.endsWith(".mjpeg"))
            {
                mjpegFileList[mjpegCount] = name;
                mjpegFileSizes[mjpegCount] = file.size(); // Save file size (in bytes)
                mjpegCount++;
                if (mjpegCount >= MAX_FILES)
                    break;
            }
        }
        file.close();
    }
    mjpegDir.close();
    Serial.printf("%d mjpeg files read\n", mjpegCount);
    // Optionally, print out each file's size for debugging:
    for (int i = 0; i < mjpegCount; i++)
    {
        Serial.printf("File Index %d: %s, Size: %lu bytes (%s)\n", i, mjpegFileList[i].c_str(), mjpegFileSizes[i], formatBytes(mjpegFileSizes[i]).c_str());
    }
}

// Function helper display sizes on the serial monitor
String formatBytes(size_t bytes)
{
    if (bytes < 1024)
    {
        return String(bytes) + " B";
    }
    else if (bytes < (1024 * 1024))
    {
        return String(bytes / 1024.0, 2) + " KB";
    }
    else
    {
        return String(bytes / 1024.0 / 1024.0, 2) + " MB";
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
