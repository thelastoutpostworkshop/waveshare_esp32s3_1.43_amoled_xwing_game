// Tutorial :
// Use board "Waveshare ESP32-S3-Touch-AMOLED-1.43" (last tested on v3.3.2)

#include "JPEGDEC.h"
#include <Arduino_GFX_Library.h>
#include <cstring>
#include <cstdio>
#include "esp_log.h"
#include "board_config.h"
#include "FT3168.h"   // Capacitive Touch functions, included in the project
#include "qmi8658c.h" // QMI8658 6-axis IMU (3-axis accelerometer and 3-axis gyroscope) functions
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp32-hal-psram.h"

#include "images/target.h"
#include "images/x_wing_bold.h"
#include "images/x_wing_faint.h"
#include "images/explosion/explosion_00000.h"
#include "images/explosion/explosion_00001.h"
#include "images/explosion/explosion_00002.h"
#include "images/explosion/explosion_00003.h"
#include "images/explosion/explosion_00004.h"
#include "images/explosion/explosion_00005.h"
#include "images/explosion/explosion_00006.h"
#include "images/explosion/explosion_00007.h"
#include "images/explosion/explosion_00008.h"
#include "images/explosion/explosion_00009.h"
#include "images/explosion/explosion_00010.h"
#include "images/explosion/explosion_00011.h"
#include "images/explosion/explosion_00012.h"
#include "images/explosion/explosion_00013.h"
#include "images/explosion/explosion_00014.h"
#include "images/explosion/explosion_00015.h"
#include "images/explosion/explosion_00016.h"
#include "images/explosion/explosion_00017.h"
#include "images/explosion/explosion_00018.h"
#include "images/explosion/explosion_00019.h"
#include "images/explosion/explosion_00020.h"
#include "images/explosion/explosion_00021.h"
#include "images/explosion/explosion_00022.h"
#include "images/explosion/explosion_00023.h"
#include "images/explosion/explosion_00024.h"
#include "images/explosion/explosion_00025.h"
#include "images/explosion/explosion_00026.h"
#include "images/explosion/explosion_00027.h"
#include "images/explosion/explosion_00028.h"
#include "images/explosion/explosion_00029.h"

#include "fonts/Aurebesh_Bold20pt7b.h"

static inline uint16_t toBE565(uint16_t color);

class PSRAMCanvas16 : public Arduino_Canvas
{
public:
    PSRAMCanvas16(int16_t w, int16_t h)
        : Arduino_Canvas(w, h, nullptr) {}

    ~PSRAMCanvas16()
    {
        if (_framebuffer)
        {
            heap_caps_free(_framebuffer);
            _framebuffer = nullptr;
        }
    }

    bool begin(int32_t speed = GFX_NOT_DEFINED) override
    {
        (void)speed;
        if (!_framebuffer)
        {
            size_t bytes = _width * _height * sizeof(uint16_t);
            _framebuffer = (uint16_t *)(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (!_framebuffer)
            {
                return false;
            }
            memset(_framebuffer, 0, bytes);
        }
        return true;
    }
};

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
static void updateSpritePosition();
static void renderFrame();
static void drawHud();
static void blitCanvasToBuffer(Arduino_Canvas &canvas, uint16_t *dest, uint16_t transparentColor = 0x0000);
static void startExplosionAt(int x, int y);
static void updateExplosionPlayback();
static bool renderExplosionFrame(uint16_t *backBuffer);
int jpegDrawCallback(JPEGDRAW *pDraw);

// Game sensitivity adjustments (lower value = easier; higher value = harder)
#define ACCEL_SCALE 1.5f            // Tilt acceleration scale
#define GYRO_SCALE 0.05f            // Gyro rotation influence
#define DAMPING 0.92f               // 1.0 = glide forever, 0.0 = stop instantly
#define XWING_TARGET_AREA 30            // Lower = larger bullseye (easier), higher = tighter bullseye (harder)

// Direction modes: 0 = normal, 1 = invert pitch, 2 = invert roll, 3 = invert both
// You can make the game harder by choosing a mode that is unatural to you
#define XWING_DIRECTION_MODE 2

enum class JpegRenderMode
{
    Panel,
    Buffer
};

struct JpegRenderContext
{
    JpegRenderMode mode;
    uint16_t *buffer;
    int pitch;
    int originX;
    int originY;
    int limitWidth;
    int limitHeight;
};

#define SPRITE_COLORKEY_BRIGHTNESS_THRESHOLD 6 // Raise to keep darker pixels opaque; lower to treat more near-black shades as transparent
#define SCORE_POS_X 70  // Horizontal position for score text
#define SCORE_POS_Y 370 // Vertical baseline for score text

static JpegRenderContext g_jpegContext = {JpegRenderMode::Panel, nullptr, 0, 0, 0, 0, 0};

static Arduino_DataBus *g_displayBus = nullptr;
static Arduino_CO5300 *g_display = nullptr;
static constexpr uint16_t COLOR_BLACK = 0x0000;
static constexpr uint16_t COLOR_WHITE = 0xFFFF;

static uint16_t *g_frameBuffers[2] = {nullptr, nullptr};
static int g_frontBufferIndex = 0;
static bool g_framebuffersReady = false;
static uint16_t g_backgroundColorBE = 0;
static uint16_t *g_staticBackground = nullptr;
static bool g_backgroundReady = false;
static size_t g_framebufferBytes = 0;
static PSRAMCanvas16 g_textCanvas(DISPLAY_WIDTH, DISPLAY_HEIGHT);

static constexpr int EXPLOSION_FRAME_COUNT = 30;
static constexpr uint32_t EXPLOSION_FRAME_DELAY_MS = 50; // Controls explosion playback speed (ms per frame)

struct ExplosionFrame
{
    const uint8_t *data;
    size_t size;
};

static const ExplosionFrame g_explosionFrames[EXPLOSION_FRAME_COUNT] = {
    {explosion_00000, sizeof(explosion_00000)},
    {explosion_00001, sizeof(explosion_00001)},
    {explosion_00002, sizeof(explosion_00002)},
    {explosion_00003, sizeof(explosion_00003)},
    {explosion_00004, sizeof(explosion_00004)},
    {explosion_00005, sizeof(explosion_00005)},
    {explosion_00006, sizeof(explosion_00006)},
    {explosion_00007, sizeof(explosion_00007)},
    {explosion_00008, sizeof(explosion_00008)},
    {explosion_00009, sizeof(explosion_00009)},
    {explosion_00010, sizeof(explosion_00010)},
    {explosion_00011, sizeof(explosion_00011)},
    {explosion_00012, sizeof(explosion_00012)},
    {explosion_00013, sizeof(explosion_00013)},
    {explosion_00014, sizeof(explosion_00014)},
    {explosion_00015, sizeof(explosion_00015)},
    {explosion_00016, sizeof(explosion_00016)},
    {explosion_00017, sizeof(explosion_00017)},
    {explosion_00018, sizeof(explosion_00018)},
    {explosion_00019, sizeof(explosion_00019)},
    {explosion_00020, sizeof(explosion_00020)},
    {explosion_00021, sizeof(explosion_00021)},
    {explosion_00022, sizeof(explosion_00022)},
    {explosion_00023, sizeof(explosion_00023)},
    {explosion_00024, sizeof(explosion_00024)},
    {explosion_00025, sizeof(explosion_00025)},
    {explosion_00026, sizeof(explosion_00026)},
    {explosion_00027, sizeof(explosion_00027)},
    {explosion_00028, sizeof(explosion_00028)},
    {explosion_00029, sizeof(explosion_00029)}};

static bool g_explosionActive = false;
static int g_explosionFrameIndex = 0;
static uint32_t g_explosionNextFrameMs = 0;
static int g_explosionPosX = 0;
static int g_explosionPosY = 0;

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
                                         PIN_NUM_LCD_DATA3);
    if (!g_displayBus)
    {
        Serial.println("ERROR: Failed to allocate display bus");
        while (true)
        {
            delay(1000);
        }
    }
    g_display = new Arduino_CO5300(g_displayBus,
                                   PIN_NUM_LCD_RST,
                                   0,
                                   DISPLAY_WIDTH,
                                   DISPLAY_HEIGHT,
                                   6, 0, 6, 0);

    if (!g_display || !g_display->begin(BUS_SPEED))
    {
        Serial.println("Display initialization failed!");
        while (true)
        {
            delay(1000);
        }
    }
    g_display->setRotation(0);
    g_display->fillScreen(0x0000);
    Serial.println("Display initialized (Arduino_GFX CO5300)");

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

    if (g_framebuffersReady && g_spriteReady)
    {
        renderFrame();
    }
}

void loop()
{
    if (!g_framebuffersReady || !g_spriteReady)
    {
        delay(50);
        return;
    }

    updateExplosionPlayback();
    updateSpritePosition();
    renderFrame();

    if (g_touchTriggered)
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
                startExplosionAt(g_spriteDrawX, g_spriteDrawY);
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

    if (g_explosionActive)
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

    const float maxX = max(0, DISPLAY_WIDTH - g_xWingWidth);
    const float maxY = max(0, DISPLAY_HEIGHT - g_xWingHeight);

    if (g_spritePosX < 0.0f)
    {
        g_spritePosX = 0.0f;
        g_spriteVelX = 0.0f;
    }
    else if (g_spritePosX > maxX)
    {
        g_spritePosX = maxX;
        g_spriteVelX = 0.0f;
    }

    if (g_spritePosY < 0.0f)
    {
        g_spritePosY = 0.0f;
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

static void startExplosionAt(int x, int y)
{
    g_explosionActive = true;
    g_explosionFrameIndex = 0;
    g_explosionPosX = x;
    g_explosionPosY = y;
    g_explosionNextFrameMs = millis() + EXPLOSION_FRAME_DELAY_MS;
    g_spriteVelX = 0.0f;
    g_spriteVelY = 0.0f;
}

static void updateExplosionPlayback()
{
    if (!g_explosionActive)
        return;

    uint32_t now = millis();
    if ((int32_t)(now - g_explosionNextFrameMs) < 0)
        return;

    if (g_explosionFrameIndex + 1 < EXPLOSION_FRAME_COUNT)
    {
        ++g_explosionFrameIndex;
        g_explosionNextFrameMs = now + EXPLOSION_FRAME_DELAY_MS;
    }
    else
    {
        g_explosionActive = false;
    }
}

static bool renderExplosionFrame(uint16_t *backBuffer)
{
    if (!g_explosionActive || !backBuffer)
        return false;

    if (g_explosionFrameIndex < 0 || g_explosionFrameIndex >= EXPLOSION_FRAME_COUNT)
        return false;

    const ExplosionFrame &frame = g_explosionFrames[g_explosionFrameIndex];
    if (!frame.data || frame.size == 0)
        return false;

    if (!decodeJpegToBuffer(backBuffer, DISPLAY_WIDTH, DISPLAY_HEIGHT, g_explosionPosX, g_explosionPosY, frame.data, frame.size))
    {
        Serial.printf("Failed to render explosion frame %d\n", g_explosionFrameIndex);
        return false;
    }

    return true;
}

static void drawHud()
{
    if (!g_textCanvas.getFramebuffer())
        return;

    g_textCanvas.fillScreen(COLOR_BLACK);
    g_textCanvas.setFont(&Aurebesh_Bold20pt7b);
    g_textCanvas.setTextColor(COLOR_WHITE, COLOR_BLACK);
    g_textCanvas.setCursor(SCORE_POS_X, SCORE_POS_Y);

    char scoreBuf[16];
    snprintf(scoreBuf, sizeof(scoreBuf), "%lu", (unsigned long)g_score);
    g_textCanvas.print(scoreBuf);
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

    const bool explosionPlaying = g_explosionActive;
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
        renderExplosionFrame(backBuffer);
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


