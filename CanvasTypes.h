// Custom canvas and JPEG rendering helpers shared across the project.
#pragma once

#include <Arduino_GFX_Library.h>
#include <cstdint>

class PSRAMCanvas16 : public Arduino_Canvas
{
public:
    PSRAMCanvas16(int16_t w, int16_t h);
    ~PSRAMCanvas16();
    bool begin(int32_t speed = GFX_NOT_DEFINED) override;
};

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
