// Tutorial :
// Use board "Waveshare ESP32-S3-Touch-AMOLED-1.43" (last tested on v3.3.2)

// Libraries to be installed
#include "JPEGDEC.h"             // Install "JPEGDEC" library version 1.8.2
#include <Arduino_GFX_Library.h> // Install "GFX Library for Arduino" version 1.6.2

// Main header files included in this project
#include "board_config.h"         // Waveshare ESP32-S3-Touch-AMOLED-1.43 pins & other configurations
#include "FT3168.h"               // Capacitive Touch functions, included in the project
#include "qmi8658c.h"             // QMI8658 6-axis IMU (3-axis accelerometer and 3-axis gyroscope) functions
#include "JpegAnimation.h"        // JPEG animation class
#include "CanvasTypes.h"          // Canvas class
#include "animationsDefintions.h" // Animation frame definitions

// Game assets
#include "animationsDefintions.h"              // Animation assets
#include "images/image_assets.h"               // Image assets
#include "fonts/Aurebesh_Bold25pt7b.h"         // Score and timer font
#include "fonts/Aurebesh_Bold7pt7b.h"          // Sensor font
#include "fonts/square_sans_serif_717pt7b.h"   // Best score font

// Header files helpers
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp32-hal-psram.h"

// Game instructions
// Game sensitivity adjustments (lower value = easier; higher value = harder)
#define ACCEL_SCALE 1.5f     // Tilt acceleration scale
#define GYRO_SCALE 0.05f     // Gyro rotation influence
#define DAMPING 0.92f        // 1.0 = glide forever, 0.0 = stop instantly
#define XWING_TARGET_AREA 30 // Lower = larger bullseye (easier), higher = tighter bullseye (harder)

// Direction modes: 0 = normal, 1 = invert pitch, 2 = invert roll, 3 = invert both
// You can make the game harder by choosing a mode that is unatural to you
#define XWING_DIRECTION_MODE 2

// Game goals
#define ROUND_TARGET_HITS 3           // Number of hits before times run out
#define ROUND_DURATION_SECONDS 20     // Time to score hits in seconds
#define ROUND_DURATION_MS (ROUND_DURATION_SECONDS * 1000U)

JPEGDEC jpeg;

// Touch global variables
volatile uint16_t touchX = 0;
volatile uint16_t touchY = 0;
static TaskHandle_t touchTaskHandle = nullptr;

// Global to store the latest sample read the accelerometer and gyroscope (QMI8658)
typedef struct
{
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
} ImuData;
volatile ImuData g_imu;
#define READ_SAMPLE_INTERVAL_MS 50 // Interval in ms to read a sample from the QMI8658

static inline uint16_t toBE565(uint16_t color);
static void touchTask(void *pvParameter);
static const char *jpegErrorToString(int error);
static void printJpegError(const char *context, int error);
static bool initFramebuffers();
static void clearBuffer(uint16_t *buffer, uint16_t colorBE);
static bool decodeJpegToBuffer(uint16_t *buffer, int pitch, int bufferHeight, int x, int y, const uint8_t *data, size_t size, int decodeOptions = 0);
static void blitSprite(uint16_t *dst, int pitch, int x, int y, const uint16_t *sprite, int spriteW, int spriteH);
static bool loadXWingSprite();
static bool buildStaticBackground();
static bool showJpegAt(int x, int y, const uint8_t *data, size_t size, int decodeOptions = 0);
static void playIntroAnimation();
static void waitForTouchRelease();
static void printBestTimeAt(int16_t x, int16_t y);
static void playBlockingAnimation(JpegAnimation &animation, int16_t bestTextX, int16_t bestTextY);
static void playGameOverAnimation();
static void playYouWinAnimation();
static void startGameRound();
static int formatSensorDisplayValue(float value);
static void beginRoundTimerPause();
static void endRoundTimerPause();
static void resetRoundTimerPause();
static uint32_t getRoundElapsedMs();
static void updateSpritePosition();
static void renderFrame();
static void drawHud();
static void blitCanvasToBuffer(Arduino_Canvas &canvas, uint16_t *dest, uint16_t transparentColor = 0x0000);
int jpegDrawCallback(JPEGDRAW *pDraw);

// Game global variables, no need to change this, unless you redesign the game
#define SPRITE_COLORKEY_BRIGHTNESS_THRESHOLD 6 // Raise to keep darker pixels opaque; lower to treat more near-black shades as transparent
#define SCORE_POS_X 70                         // Horizontal position for score text
#define SCORE_POS_Y 370                        // Vertical baseline for score text
#define XWING_VISIBLE_MARGIN 10                // Pixels guaranteed to remain on-screen when drifting off the edge
#define BLINK_INTRO_POS_X 110
#define BLINK_INTRO_POS_Y 375
#define SENSOR_POS_X 360
#define SENSOR_POS_Y 220
#define TIMER_POS_X 280
#define TIMER_POS_Y 370
#define INTRO_BEST_TEXT_POS_X 20
#define INTRO_BEST_TEXT_POS_Y 305
#define GAME_OVER_BEST_TEXT_POS_X 20
#define GAME_OVER_BEST_TEXT_POS_Y 320
#define YOU_WIN_BEST_TEXT_POS_X 20
#define YOU_WIN_BEST_TEXT_POS_Y 290

static JpegRenderContext g_jpegContext = {JpegRenderMode::Panel, nullptr, 0, 0, 0, 0, 0};

static Arduino_DataBus *g_displayBus = nullptr;
static Arduino_CO5300 *g_outputDisplay = nullptr;
static Arduino_Canvas *g_display = nullptr;
static constexpr uint16_t COLOR_BLACK = 0x0000;
static constexpr uint16_t COLOR_WHITE = 0xFFFF;
static constexpr uint16_t COLOR_RED = 0xF800;

static uint16_t *g_frameBuffers[2] = {nullptr, nullptr};
static int g_frontBufferIndex = 0;
static bool g_framebuffersReady = false;
static uint16_t g_backgroundColorBE = 0;
static uint16_t *g_staticBackground = nullptr;
static bool g_backgroundReady = false;
static size_t g_framebufferBytes = 0;
static PSRAMCanvas16 g_textCanvas(DISPLAY_WIDTH, DISPLAY_HEIGHT);

static JpegAnimation g_introAnimation(g_introFrames, kIntroFrameCount, decodeJpegToBuffer);
static JpegAnimation g_blinkAnimation(g_blinkFrames, kBlinkFrameCount, decodeJpegToBuffer);
static JpegAnimation g_gameOverAnimation(g_gameOverFrames, kGameOverFrameCount, decodeJpegToBuffer);
static JpegAnimation g_youWinAnimation(g_youWinFrames, kYouWinFrameCount, decodeJpegToBuffer);
static JpegAnimation g_explosionAnimation(g_explosionFrames, kExplosionFrameCount, decodeJpegToBuffer);

static bool g_gameActive = false;
static uint32_t g_roundStartMs = 0;
static int g_roundHits = 0;
static bool g_roundTimerPaused = false;
static uint32_t g_roundPauseStartMs = 0;
static uint32_t g_roundPauseAccumulatedMs = 0;

static uint16_t *g_xWingBoldSprite = nullptr;
static uint16_t *g_xWingFaintSprite = nullptr;
static int g_xWingWidth = 0;
static int g_xWingHeight = 0;
static bool g_spriteReady = false;
static bool g_shipInTarget = false;
static int g_shipCenterX = DISPLAY_WIDTH / 2;
static int g_shipCenterY = DISPLAY_HEIGHT / 2;
static int g_currentLeeway = 0;
static uint32_t g_score = 0;
static uint32_t g_bestRoundTimeMs = 0;
static volatile bool g_touchTriggered = false;
static volatile uint16_t g_touchStartX = 0;
static volatile uint16_t g_touchStartY = 0;

static float g_spritePosX = 0.0f;
static float g_spritePosY = 0.0f;
static float g_spriteVelX = 0.0f;
static float g_spriteVelY = 0.0f;
static int g_spriteDrawX = 0;
static int g_spriteDrawY = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000); // Give time to the serial port to show initial messages printed on the serial port upon reset

    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    if (!psramFound())
    {
        Serial.println("ERROR: PSRAM not detected. Enable PSRAM first.");
        while (true)
        {
            delay(1000);
        }
    }

    g_displayBus = new Arduino_ESP32QSPI(PIN_NUM_LCD_CS,
                                         PIN_NUM_LCD_PCLK,
                                         PIN_NUM_LCD_DATA0,
                                         PIN_NUM_LCD_DATA1,
                                         PIN_NUM_LCD_DATA2,
                                         PIN_NUM_LCD_DATA3,
                                         false);
    if (!g_displayBus)
    {
        Serial.println("ERROR: Failed to allocate display bus");
        while (true)
        {
            delay(1000);
        }
    }
    g_outputDisplay = new Arduino_CO5300(g_displayBus,
                                         PIN_NUM_LCD_RST,
                                         0,
                                         DISPLAY_WIDTH,
                                         DISPLAY_HEIGHT,
                                         6, 0, 6, 0);
    if (!g_outputDisplay)
    {
        Serial.println("ERROR: Failed to allocate display driver");
        while (true)
        {
            delay(1000);
        }
    }

    g_display = new Arduino_Canvas(DISPLAY_WIDTH, DISPLAY_HEIGHT, g_outputDisplay);
    if (!g_display || !g_display->begin(BUS_SPEED))
    {
        Serial.println("Display initialization failed!");
        while (true)
        {
            delay(1000);
        }
    }
    g_display->setRotation(0);
    g_display->fillScreen(COLOR_BLACK);
    g_display->flush();
    Serial.println("Display initialized (Arduino_GFX Canvas CO5300)");

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

    g_framebuffersReady = initFramebuffers();
    if (!g_framebuffersReady)
    {
        Serial.println("ERROR: Failed to allocate framebuffers; animation disabled");
    }

    if (g_framebuffersReady)
    {
        g_backgroundReady = buildStaticBackground();
        if (!g_backgroundReady)
        {
            Serial.println("WARNING: Static background not available; falling back to solid color");
        }
    }

    if (!g_textCanvas.begin())
    {
        Serial.println("ERROR: Failed to allocate text canvas");
    }
    else
    {
        g_textCanvas.setTextWrap(false);
        g_textCanvas.setTextSize(1);
        g_textCanvas.setRotation(0);
    }

    g_spriteReady = loadXWingSprite();
    if (!g_spriteReady)
    {
        Serial.println("ERROR: Failed to load X-Wing sprite");
    }

    playIntroAnimation();

    if (g_framebuffersReady && g_spriteReady)
    {
        startGameRound();
        renderFrame();
    }
    else
    {
        g_gameActive = false;
    }
}

void loop()
{
    if (!g_framebuffersReady || !g_spriteReady)
    {
        delay(50);
        return;
    }

    if (!g_gameActive)
    {
        delay(16);
        return;
    }

    uint32_t elapsedMs = getRoundElapsedMs();
    if (elapsedMs >= ROUND_DURATION_MS)
    {
        g_gameActive = false;
        endRoundTimerPause();
        g_explosionAnimation.stop();
        playGameOverAnimation();
        playIntroAnimation();
        startGameRound();
        renderFrame();
        return;
    }

    bool explosionWasActive = g_explosionAnimation.isActive();
    g_explosionAnimation.update();
    if (explosionWasActive && !g_explosionAnimation.isActive())
    {
        endRoundTimerPause();
    }

    updateSpritePosition();
    renderFrame();

    if (g_gameActive && g_touchTriggered)
    {
        uint16_t touchXCopy = g_touchStartX;
        uint16_t touchYCopy = g_touchStartY;
        g_touchTriggered = false;

        if (g_shipInTarget)
        {
            int dx = touchXCopy - g_shipCenterX;
            if (dx < 0)
                dx = -dx;
            int dy = touchYCopy - g_shipCenterY;
            if (dy < 0)
                dy = -dy;

            if (dx <= g_currentLeeway && dy <= g_currentLeeway)
            {
                ++g_score;
                g_explosionAnimation.start(g_spriteDrawX, g_spriteDrawY);
                beginRoundTimerPause();
                g_spriteVelX = 0.0f;
                g_spriteVelY = 0.0f;
                ++g_roundHits;
                if (g_roundHits >= ROUND_TARGET_HITS)
                {
                    uint32_t roundTimeMs = getRoundElapsedMs();
                    if (roundTimeMs == 0)
                        roundTimeMs = 1;
                    if (g_bestRoundTimeMs == 0 || roundTimeMs < g_bestRoundTimeMs)
                        g_bestRoundTimeMs = roundTimeMs;

                    g_gameActive = false;
                    endRoundTimerPause();
                    g_explosionAnimation.stop();
                    playYouWinAnimation();
                    playIntroAnimation();
                    startGameRound();
                    renderFrame();
                    return;
                }
            }
        }
    }

    delay(16);
}

static inline uint16_t toBE565(uint16_t color)
{
    return (uint16_t)((color << 8) | (color >> 8));
}

static void clearBuffer(uint16_t *buffer, uint16_t colorBE)
{
    if (!buffer)
        return;

    const size_t pixelCount = (size_t)DISPLAY_WIDTH * (size_t)DISPLAY_HEIGHT;
    const uint32_t pattern = ((uint32_t)colorBE << 16) | colorBE;
    uint32_t *dst32 = reinterpret_cast<uint32_t *>(buffer);
    const size_t quadCount = pixelCount / 2;

    for (size_t i = 0; i < quadCount; ++i)
    {
        dst32[i] = pattern;
    }

    if (pixelCount & 1)
    {
        buffer[pixelCount - 1] = colorBE;
    }
}

static inline bool isSpriteTransparent(uint16_t bePixel)
{
    uint16_t native = (uint16_t)((bePixel << 8) | (bePixel >> 8));
    uint8_t r5 = (native >> 11) & 0x1F;
    uint8_t g6 = (native >> 5) & 0x3F;
    uint8_t b5 = native & 0x1F;
    uint16_t brightness = (uint16_t)(r5 + (g6 >> 1) + b5);
    return brightness <= SPRITE_COLORKEY_BRIGHTNESS_THRESHOLD;
}

static bool decodeJpegToBuffer(uint16_t *buffer, int pitch, int bufferHeight, int x, int y, const uint8_t *data, size_t size, int decodeOptions)
{
    if (!buffer || pitch <= 0 || bufferHeight <= 0 || !data || size == 0)
        return false;

    if (!jpeg.openFLASH((uint8_t *)data, size, jpegDrawCallback))
    {
        printJpegError("Failed to open JPEG image", jpeg.getLastError());
        return false;
    }

    g_jpegContext.mode = JpegRenderMode::Buffer;
    g_jpegContext.buffer = buffer;
    g_jpegContext.pitch = pitch;
    g_jpegContext.originX = x;
    g_jpegContext.originY = y;
    g_jpegContext.limitWidth = pitch;
    g_jpegContext.limitHeight = bufferHeight;

    jpeg.setPixelType(RGB565_BIG_ENDIAN);
    bool decoded = jpeg.decode(0, 0, decodeOptions);
    int lastError = jpeg.getLastError();
    jpeg.close();

    g_jpegContext.mode = JpegRenderMode::Panel;
    g_jpegContext.buffer = nullptr;
    g_jpegContext.pitch = 0;
    g_jpegContext.originX = 0;
    g_jpegContext.originY = 0;
    g_jpegContext.limitWidth = 0;
    g_jpegContext.limitHeight = 0;

    if (!decoded)
    {
        printJpegError("Failed to decode JPEG image", lastError);
        return false;
    }

    return true;
}

static bool decodeSprite(const uint8_t *data, size_t size, uint16_t **outPixels, int *outWidth, int *outHeight)
{
    if (!outPixels)
        return false;

    if (!jpeg.openFLASH((uint8_t *)data, size, jpegDrawCallback))
    {
        printJpegError("Failed to open sprite JPEG", jpeg.getLastError());
        return false;
    }

    int width = jpeg.getWidth();
    int height = jpeg.getHeight();

    if (outWidth && outHeight)
    {
        *outWidth = width;
        *outHeight = height;
    }
    else
    {
        if (width != g_xWingWidth || height != g_xWingHeight)
        {
            Serial.println("ERROR: Sprite dimensions mismatch");
            jpeg.close();
            return false;
        }
    }

    size_t bytes = (size_t)width * (size_t)height * sizeof(uint16_t);
    uint16_t *pixels = (uint16_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pixels)
    {
        Serial.println("ERROR: Failed to allocate sprite buffer");
        jpeg.close();
        return false;
    }

    g_jpegContext.mode = JpegRenderMode::Buffer;
    g_jpegContext.buffer = pixels;
    g_jpegContext.pitch = width;
    g_jpegContext.originX = 0;
    g_jpegContext.originY = 0;
    g_jpegContext.limitWidth = width;
    g_jpegContext.limitHeight = height;

    jpeg.setPixelType(RGB565_BIG_ENDIAN);
    bool decoded = jpeg.decode(0, 0, 0);
    int lastError = jpeg.getLastError();
    jpeg.close();

    g_jpegContext.mode = JpegRenderMode::Panel;
    g_jpegContext.buffer = nullptr;
    g_jpegContext.pitch = 0;
    g_jpegContext.originX = 0;
    g_jpegContext.originY = 0;
    g_jpegContext.limitWidth = 0;
    g_jpegContext.limitHeight = 0;

    if (!decoded)
    {
        printJpegError("Failed to decode sprite JPEG", lastError);
        heap_caps_free(pixels);
        return false;
    }

    *outPixels = pixels;
    return true;
}

static bool initFramebuffers()
{
    const size_t bytes = (size_t)DISPLAY_WIDTH * (size_t)DISPLAY_HEIGHT * sizeof(uint16_t);
    g_framebufferBytes = bytes;

    for (int i = 0; i < 2; ++i)
    {
        g_frameBuffers[i] = (uint16_t *)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!g_frameBuffers[i])
        {
            Serial.printf("Failed to allocate framebuffer %d (%u bytes)\n", i, (unsigned int)bytes);
            for (int j = 0; j < i; ++j)
            {
                heap_caps_free(g_frameBuffers[j]);
                g_frameBuffers[j] = nullptr;
            }
            g_framebufferBytes = 0;
            return false;
        }
    }

    g_backgroundColorBE = toBE565(COLOR_BLACK);
    for (int i = 0; i < 2; ++i)
    {
        clearBuffer(g_frameBuffers[i], g_backgroundColorBE);
    }
    g_frontBufferIndex = 0;
    return true;
}

static void blitSprite(uint16_t *dst, int pitch, int x, int y, const uint16_t *sprite, int spriteW, int spriteH)
{
    if (!dst || !sprite || spriteW <= 0 || spriteH <= 0)
        return;

    const int startX = max(0, x);
    const int startY = max(0, y);
    const int endX = min(DISPLAY_WIDTH, x + spriteW);
    const int endY = min(DISPLAY_HEIGHT, y + spriteH);

    if (startX >= endX || startY >= endY)
        return;

    const int srcOffsetX = startX - x;
    const int srcOffsetY = startY - y;
    const int rowWidth = endX - startX;

    const int rows = endY - startY;
    for (int row = 0; row < rows; ++row)
    {
        const uint16_t *srcRowPtr = sprite + (srcOffsetY + row) * spriteW + srcOffsetX;
        uint16_t *dstRowPtr = dst + (startY + row) * pitch + startX;
        for (int col = 0; col < rowWidth; ++col)
        {
            uint16_t pixel = srcRowPtr[col];
            if (!isSpriteTransparent(pixel))
            {
                dstRowPtr[col] = pixel;
            }
        }
    }
}

static bool buildStaticBackground()
{
    if (!g_framebuffersReady || g_framebufferBytes == 0)
        return false;

    if (!g_staticBackground)
    {
        g_staticBackground = (uint16_t *)heap_caps_malloc(g_framebufferBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!g_staticBackground)
        {
            Serial.println("ERROR: Failed to allocate static background buffer");
            return false;
        }
    }

    clearBuffer(g_staticBackground, g_backgroundColorBE);

    bool ok = true;
    if (!decodeJpegToBuffer(g_staticBackground, DISPLAY_WIDTH, DISPLAY_HEIGHT, 0, 0, target, sizeof(target), 0))
    {
        ok = false;
    }

    if (ok)
    {
        for (int i = 0; i < 2; ++i)
        {
            if (g_frameBuffers[i])
            {
                memcpy(g_frameBuffers[i], g_staticBackground, g_framebufferBytes);
            }
        }
    }

    g_backgroundReady = ok;
    return ok;
}

static int formatSensorDisplayValue(float value)
{
    float scaled = fabsf(value) * 100.0f;
    int display = static_cast<int>(scaled + 0.5f);
    if (display > 999)
        display = 999;
    return display;
}

static void beginRoundTimerPause()
{
    if (!g_roundTimerPaused)
    {
        g_roundTimerPaused = true;
        g_roundPauseStartMs = millis();
    }
}

static void endRoundTimerPause()
{
    if (g_roundTimerPaused)
    {
        uint32_t now = millis();
        g_roundPauseAccumulatedMs += now - g_roundPauseStartMs;
        g_roundTimerPaused = false;
        g_roundPauseStartMs = 0;
    }
}

static void resetRoundTimerPause()
{
    g_roundTimerPaused = false;
    g_roundPauseStartMs = 0;
    g_roundPauseAccumulatedMs = 0;
}

static uint32_t getRoundElapsedMs()
{
    uint32_t now = millis();
    uint32_t paused = g_roundPauseAccumulatedMs;
    if (g_roundTimerPaused)
    {
        paused += now - g_roundPauseStartMs;
    }

    uint32_t rawElapsed = now - g_roundStartMs;
    return (rawElapsed >= paused) ? (rawElapsed - paused) : 0;
}

static void playIntroAnimation()
{
    if (!g_framebuffersReady || !g_display)
    {
        while (!g_touchTriggered)
        {
            delay(16);
        }
        g_touchTriggered = false;
        while (touchX != 0 || touchY != 0)
        {
            delay(10);
        }
        g_touchStartX = 0;
        g_touchStartY = 0;
        return;
    }

    uint16_t *buffer = g_frameBuffers[g_frontBufferIndex];
    if (!buffer)
    {
        buffer = g_frameBuffers[0];
    }
    if (!buffer)
        return;

    g_introAnimation.start(0, 0);
    g_blinkAnimation.start(BLINK_INTRO_POS_X, BLINK_INTRO_POS_Y);
    bool playingIntro = true;
    int lastIntroFrame = -1;
    int lastBlinkFrame = -1;
    uint32_t blinkRestartMs = millis();

    while (true)
    {
        if (playingIntro)
        {
            g_introAnimation.update();
            int currentFrame = g_introAnimation.currentFrame();
            if (lastIntroFrame != currentFrame)
            {
                if (g_introAnimation.render(buffer, DISPLAY_WIDTH, DISPLAY_HEIGHT))
                {
                    g_display->draw16bitBeRGBBitmap(0, 0, buffer, DISPLAY_WIDTH, DISPLAY_HEIGHT);
                    g_display->flush();
                }
                lastIntroFrame = currentFrame;
            }

            if (!g_introAnimation.isActive())
            {
                playingIntro = false;
                printBestTimeAt(INTRO_BEST_TEXT_POS_X, INTRO_BEST_TEXT_POS_Y);
                g_blinkAnimation.start(BLINK_INTRO_POS_X, BLINK_INTRO_POS_Y);
                lastBlinkFrame = -1;
                blinkRestartMs = millis();
            }
        }
        else
        {
            g_blinkAnimation.update();
            if (!g_blinkAnimation.isActive())
            {
                uint32_t now = millis();
                if (now - blinkRestartMs >= 200)
                {
                    g_blinkAnimation.start(BLINK_INTRO_POS_X, BLINK_INTRO_POS_Y);
                    lastBlinkFrame = -1;
                    blinkRestartMs = now;
                }
            }

            int blinkFrame = g_blinkAnimation.currentFrame();
            if (lastBlinkFrame != blinkFrame)
            {
                if (g_blinkAnimation.render(buffer, DISPLAY_WIDTH, DISPLAY_HEIGHT))
                {
                    g_display->draw16bitBeRGBBitmap(0, 0, buffer, DISPLAY_WIDTH, DISPLAY_HEIGHT);
                    printBestTimeAt(INTRO_BEST_TEXT_POS_X, INTRO_BEST_TEXT_POS_Y);
                }
                lastBlinkFrame = blinkFrame;
            }
        }

        if (g_touchTriggered)
        {
            g_touchTriggered = false;
            break;
        }

        delay(16);
    }

    g_introAnimation.stop();
    g_blinkAnimation.stop();
    g_frontBufferIndex = 0;

    while (touchX != 0 || touchY != 0)
    {
        delay(10);
    }

    g_touchStartX = 0;
    g_touchStartY = 0;
}

static void printBestTimeAt(int16_t x, int16_t y)
{
    if (!g_display)
        return;

    g_display->setFont(&square_sans_serif_717pt7b);
    g_display->setTextColor(COLOR_WHITE, COLOR_BLACK);
    g_display->setCursor(x, y);

    char buf[32];
    if (g_bestRoundTimeMs == 0)
    {
        snprintf(buf, sizeof(buf), "Best Score:--.--");
    }
    else
    {
        uint32_t ms = g_bestRoundTimeMs;
        uint32_t secs = ms / 1000U;
        uint32_t hundredths = (ms % 1000U) / 10U;
        if (secs > 999U)
        {
            secs = 999U;
            hundredths = 99U;
        }
        snprintf(buf, sizeof(buf), "Best Score:%lu.%02lu", (unsigned long)secs, (unsigned long)hundredths);
    }

    g_display->print(buf);
    g_display->flush();
}

static void waitForTouchRelease()
{
    while (touchX != 0 || touchY != 0)
    {
        delay(10);
    }
}

static void playBlockingAnimation(JpegAnimation &animation, int16_t bestTextX, int16_t bestTextY)
{
    if (!g_framebuffersReady || !g_display)
    {
        g_touchTriggered = false;
        while (!g_touchTriggered)
        {
            delay(16);
        }
        g_touchTriggered = false;
        waitForTouchRelease();
        g_touchStartX = 0;
        g_touchStartY = 0;
        return;
    }

    waitForTouchRelease();
    g_touchTriggered = false;

    uint16_t *buffer = g_frameBuffers[g_frontBufferIndex];
    if (!buffer)
    {
        buffer = g_frameBuffers[0];
    }
    if (!buffer)
        return;

    animation.stop();
    animation.start(0, 0);

    int lastFrame = -1;
    bool animationFinished = false;
    bool bestPrinted = false;

    while (true)
    {
        if (!animationFinished)
        {
            animation.update();
            int currentFrame = animation.currentFrame();
            if (currentFrame != lastFrame)
            {
                if (animation.render(buffer, DISPLAY_WIDTH, DISPLAY_HEIGHT))
                {
                    g_display->draw16bitBeRGBBitmap(0, 0, buffer, DISPLAY_WIDTH, DISPLAY_HEIGHT);
                    g_display->flush();
                }
                lastFrame = currentFrame;
            }

            if (!animation.isActive())
            {
                animationFinished = true;
            }
        }

        if (animationFinished && !bestPrinted)
        {
            printBestTimeAt(bestTextX, bestTextY);
            bestPrinted = true;
        }

        if (!animationFinished && g_touchTriggered)
        {
            g_touchTriggered = false;
            waitForTouchRelease();
        }
        else if (animationFinished && g_touchTriggered)
        {
            if (!bestPrinted)
            {
                printBestTimeAt(bestTextX, bestTextY);
                bestPrinted = true;
            }
            g_touchTriggered = false;
            break;
        }

        delay(16);
    }

    if (!bestPrinted)
    {
        printBestTimeAt(bestTextX, bestTextY);
    }

    waitForTouchRelease();

    g_touchStartX = 0;
    g_touchStartY = 0;
    animation.stop();
    g_frontBufferIndex = 0;
}

static void playGameOverAnimation()
{
    playBlockingAnimation(g_gameOverAnimation, GAME_OVER_BEST_TEXT_POS_X, GAME_OVER_BEST_TEXT_POS_Y);
}

static void playYouWinAnimation()
{
    playBlockingAnimation(g_youWinAnimation, YOU_WIN_BEST_TEXT_POS_X, YOU_WIN_BEST_TEXT_POS_Y);
}

static void startGameRound()
{
    g_explosionAnimation.stop();
    resetRoundTimerPause();
    g_roundHits = 0;
    g_roundStartMs = millis();
    g_gameActive = true;
    g_score = 0;
}

static bool loadXWingSprite()
{
    if (g_xWingBoldSprite)
    {
        heap_caps_free(g_xWingBoldSprite);
        g_xWingBoldSprite = nullptr;
    }
    if (g_xWingFaintSprite)
    {
        heap_caps_free(g_xWingFaintSprite);
        g_xWingFaintSprite = nullptr;
    }

    if (!decodeSprite(x_wing_bold, sizeof(x_wing_bold), &g_xWingBoldSprite, &g_xWingWidth, &g_xWingHeight))
    {
        g_xWingWidth = 0;
        g_xWingHeight = 0;
        return false;
    }

    if (!decodeSprite(x_wing_faint, sizeof(x_wing_faint), &g_xWingFaintSprite, nullptr, nullptr))
    {
        if (g_xWingBoldSprite)
        {
            heap_caps_free(g_xWingBoldSprite);
            g_xWingBoldSprite = nullptr;
        }
        g_xWingWidth = 0;
        g_xWingHeight = 0;
        return false;
    }

    g_spritePosX = (DISPLAY_WIDTH - g_xWingWidth) / 2.0f;
    g_spritePosY = (DISPLAY_HEIGHT - g_xWingHeight) / 2.0f;
    g_spriteVelX = 0.0f;
    g_spriteVelY = 0.0f;
    g_spriteDrawX = (int)(g_spritePosX + 0.5f);
    g_spriteDrawY = (int)(g_spritePosY + 0.5f);

    Serial.printf("Loaded X-Wing sprites (%d x %d)\n", g_xWingWidth, g_xWingHeight);
    return true;
}

static void updateSpritePosition()
{
    if (!g_spriteReady || g_xWingWidth <= 0 || g_xWingHeight <= 0)
        return;

    if (g_explosionAnimation.isActive())
        return;

    ImuData sample;
    sample.ax = g_imu.ax;
    sample.ay = g_imu.ay;
    sample.gx = g_imu.gx;
    sample.gy = g_imu.gy;

    int xSign = (XWING_DIRECTION_MODE == 2 || XWING_DIRECTION_MODE == 3) ? -1 : 1;
    int ySign = (XWING_DIRECTION_MODE == 1 || XWING_DIRECTION_MODE == 3) ? -1 : 1;

    g_spriteVelX += xSign * (sample.ax * ACCEL_SCALE + sample.gx * GYRO_SCALE);
    g_spriteVelY += ySign * (sample.ay * ACCEL_SCALE + sample.gy * GYRO_SCALE);

    g_spriteVelX *= DAMPING;
    g_spriteVelY *= DAMPING;

    g_spritePosX += g_spriteVelX;
    g_spritePosY += g_spriteVelY;

    const float minX = -(float)max(0, g_xWingWidth - XWING_VISIBLE_MARGIN);
    const float maxX = (float)max(0, DISPLAY_WIDTH - XWING_VISIBLE_MARGIN);
    const float minY = -(float)max(0, g_xWingHeight - XWING_VISIBLE_MARGIN);
    const float maxY = (float)max(0, DISPLAY_HEIGHT - XWING_VISIBLE_MARGIN);

    if (g_spritePosX < minX)
    {
        g_spritePosX = minX;
        g_spriteVelX = 0.0f;
    }
    else if (g_spritePosX > maxX)
    {
        g_spritePosX = maxX;
        g_spriteVelX = 0.0f;
    }

    if (g_spritePosY < minY)
    {
        g_spritePosY = minY;
        g_spriteVelY = 0.0f;
    }
    else if (g_spritePosY > maxY)
    {
        g_spritePosY = maxY;
        g_spriteVelY = 0.0f;
    }

    g_spriteDrawX = (int)(g_spritePosX + 0.5f);
    g_spriteDrawY = (int)(g_spritePosY + 0.5f);
}

static void drawHud()
{
    if (!g_textCanvas.getFramebuffer())
        return;

    g_textCanvas.fillScreen(COLOR_BLACK);
    g_textCanvas.setTextColor(COLOR_WHITE, COLOR_BLACK);

    g_textCanvas.setFont(&Aurebesh_Bold25pt7b);
    g_textCanvas.setCursor(SCORE_POS_X, SCORE_POS_Y);
    char scoreBuf[16];
    snprintf(scoreBuf, sizeof(scoreBuf), "%lu", (unsigned long)g_score);
    g_textCanvas.print(scoreBuf);

    g_textCanvas.setFont(&Aurebesh_Bold7pt7b);
    char sensorText[16];
    int accelMag = formatSensorDisplayValue(g_imu.ax);
    int gyroMag = formatSensorDisplayValue(g_imu.gx);
    snprintf(sensorText, sizeof(sensorText), "X-%03d-%03d", accelMag, gyroMag);
    g_textCanvas.setCursor(SENSOR_POS_X, SENSOR_POS_Y);
    g_textCanvas.print(sensorText);

    uint32_t elapsed = g_gameActive ? getRoundElapsedMs() : 0;
    uint32_t remainingMs = (elapsed >= ROUND_DURATION_MS) ? 0 : (ROUND_DURATION_MS - elapsed);
    int remainingSeconds = (int)(remainingMs / 1000U);
    if (remainingSeconds > 999)
        remainingSeconds = 999;
    uint16_t timerColor = (remainingSeconds <= 3 && g_gameActive) ? COLOR_RED : COLOR_WHITE;
    g_textCanvas.setFont(&Aurebesh_Bold25pt7b);
    g_textCanvas.setTextColor(timerColor, COLOR_BLACK);
    char timerText[16];
    snprintf(timerText, sizeof(timerText), "%d", remainingSeconds);
    g_textCanvas.setCursor(TIMER_POS_X, TIMER_POS_Y);
    g_textCanvas.print(timerText);
    g_textCanvas.setTextColor(COLOR_WHITE, COLOR_BLACK);
}

static void blitCanvasToBuffer(Arduino_Canvas &canvas, uint16_t *dest, uint16_t transparentColor)
{
    if (!dest)
        return;

    uint16_t *src = canvas.getFramebuffer();
    if (!src)
        return;

    const int16_t w = canvas.width();
    const int16_t h = canvas.height();
    if (w <= 0 || h <= 0)
        return;

    for (int16_t y = 0; y < h; ++y)
    {
        uint16_t *srcRow = src + (size_t)y * w;
        uint16_t *dstRow = dest + (size_t)y * DISPLAY_WIDTH;
        for (int16_t x = 0; x < w; ++x)
        {
            uint16_t color = srcRow[x];
            if (color == transparentColor)
                continue;
            dstRow[x] = toBE565(color);
        }
    }
}

static void renderFrame()
{
    if (!g_framebuffersReady)
        return;

    const int backBufferIndex = 1 - g_frontBufferIndex;
    uint16_t *backBuffer = g_frameBuffers[backBufferIndex];
    if (g_backgroundReady && g_staticBackground && g_framebufferBytes)
    {
        memcpy(backBuffer, g_staticBackground, g_framebufferBytes);
    }
    else
    {
        clearBuffer(backBuffer, g_backgroundColorBE);
    }

    g_shipInTarget = false;
    g_shipCenterX = DISPLAY_WIDTH / 2;
    g_shipCenterY = DISPLAY_HEIGHT / 2;
    g_currentLeeway = 0;

    const bool explosionPlaying = g_explosionAnimation.isActive();
    uint16_t *activeSprite = nullptr;
    if (!explosionPlaying && g_spriteReady && g_xWingWidth > 0 && g_xWingHeight > 0)
    {
        int centerX = g_spriteDrawX + g_xWingWidth / 2;
        int centerY = g_spriteDrawY + g_xWingHeight / 2;
        int dx = centerX - DISPLAY_WIDTH / 2;
        if (dx < 0)
            dx = -dx;
        int dy = centerY - DISPLAY_HEIGHT / 2;
        if (dy < 0)
            dy = -dy;

        const int baseLeeway = 80;
        int leeway = baseLeeway - XWING_TARGET_AREA;
        if (leeway < 0)
            leeway = 0;

        g_shipCenterX = centerX;
        g_shipCenterY = centerY;
        g_currentLeeway = leeway;

        if (g_xWingBoldSprite && dx <= leeway && dy <= leeway)
        {
            activeSprite = g_xWingBoldSprite;
        }
        else if (g_xWingFaintSprite)
        {
            activeSprite = g_xWingFaintSprite;
        }
    }

    g_shipInTarget = (!explosionPlaying && activeSprite != nullptr && activeSprite == g_xWingBoldSprite);

    if (explosionPlaying)
    {
        g_explosionAnimation.render(backBuffer, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    }
    else if (activeSprite)
    {
        blitSprite(backBuffer, DISPLAY_WIDTH, g_spriteDrawX, g_spriteDrawY, activeSprite, g_xWingWidth, g_xWingHeight);
    }

    drawHud();
    blitCanvasToBuffer(g_textCanvas, backBuffer);

    if (g_display)
    {
        g_display->draw16bitBeRGBBitmap(0, 0, backBuffer, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        g_display->flush();
        g_frontBufferIndex = backBufferIndex;
    }
    else
    {
        Serial.println("ERROR: Display not available");
    }
}

static bool showJpegAt(int x, int y, const uint8_t *data, size_t size, int decodeOptions)
{
    if (!data || size == 0)
    {
        Serial.println("showJpegAt: invalid JPEG source");
        return false;
    }

    if (!jpeg.openFLASH((uint8_t *)data, size, jpegDrawCallback))
    {
        printJpegError("Failed to open JPEG image", jpeg.getLastError());
        return false;
    }

    Serial.println("Successfully opened JPEG image");
    Serial.printf("Image size: %d x %d, orientation: %d, bpp: %d\n",
                  jpeg.getWidth(), jpeg.getHeight(), jpeg.getOrientation(), jpeg.getBpp());

    g_jpegContext.mode = JpegRenderMode::Panel;
    g_jpegContext.buffer = nullptr;
    g_jpegContext.pitch = 0;
    g_jpegContext.originX = 0;
    g_jpegContext.originY = 0;

    jpeg.setPixelType(RGB565_BIG_ENDIAN);
    bool decoded = jpeg.decode(x, y, decodeOptions);

    if (!decoded)
    {
        printJpegError("Failed to decode JPEG image", jpeg.getLastError());
    }
    else if (g_display)
    {
        g_display->flush();
    }

    jpeg.close();
    return decoded;
}

// Callback function to draw a JPEG
int jpegDrawCallback(JPEGDRAW *pDraw)
{

    const uint16_t *src = pDraw->pPixels;

    if (g_jpegContext.mode == JpegRenderMode::Panel)
    {
        if (!g_display)
            return 0;
        g_display->draw16bitBeRGBBitmap(pDraw->x,
                                        pDraw->y,
                                        (uint16_t *)src,
                                        pDraw->iWidth,
                                        pDraw->iHeight);
        return 1;
    }
    else if (g_jpegContext.mode == JpegRenderMode::Buffer)
    {
        if (!g_jpegContext.buffer || g_jpegContext.pitch <= 0 || g_jpegContext.limitWidth <= 0 || g_jpegContext.limitHeight <= 0)
            return 0;

        const int destX = g_jpegContext.originX + pDraw->x;
        const int destY = g_jpegContext.originY + pDraw->y;

        const int srcWidth = pDraw->iWidth;
        for (int row = 0; row < pDraw->iHeight; ++row)
        {
            int y = destY + row;
            if (y < 0 || y >= g_jpegContext.limitHeight)
                continue;

            int xStart = destX;
            int srcOffset = 0;
            if (xStart < 0)
            {
                srcOffset = -xStart;
                xStart = 0;
            }

            int xEnd = destX + srcWidth;
            if (xEnd > g_jpegContext.limitWidth)
                xEnd = g_jpegContext.limitWidth;

            int span = xEnd - xStart;
            if (span <= 0)
                continue;

            const uint16_t *srcRow = src + row * srcWidth + srcOffset;
            uint16_t *dstRow = g_jpegContext.buffer + y * g_jpegContext.pitch + xStart;
            memcpy(dstRow, srcRow, span * sizeof(uint16_t));
        }
        return 1;
    }

    return 0;
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
    bool touchActive = false;

    for (;;)
    {
        if (getTouch(&detectedX, &detectedY))
        {
            if (!touchActive)
            {
                touchActive = true;
                g_touchStartX = detectedX;
                g_touchStartY = detectedY;
                g_touchTriggered = true;
            }
            touchX = detectedX;
            touchY = detectedY;

            // Wait for finger release before resuming detection
            do
            {
                vTaskDelay(pdMS_TO_TICKS(50));
            } while (getTouch(&detectedX, &detectedY));

            touchX = 0;
            touchY = 0;
            touchActive = false;
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

static const char *jpegErrorToString(int error)
{
    switch (error)
    {
    case JPEG_SUCCESS:
        return "Operation successful";
    case JPEG_INVALID_PARAMETER:
        return "Invalid parameter";
    case JPEG_DECODE_ERROR:
        return "Decode error";
    case JPEG_UNSUPPORTED_FEATURE:
        return "Unsupported feature";
    case JPEG_INVALID_FILE:
        return "Invalid file";
    case JPEG_ERROR_MEMORY:
        return "Memory allocation failed";
    default:
        return "Unknown error";
    }
}

static void printJpegError(const char *context, int error)
{
    const char *description = jpegErrorToString(error);
    Serial.printf("%s (error %d: %s)\n", context, error, description);
}
