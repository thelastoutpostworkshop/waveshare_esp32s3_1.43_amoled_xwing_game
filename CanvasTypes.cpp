// Custom canvas and JPEG rendering helpers shared across the project.

#include "CanvasTypes.h"

#include <cstring>
#include "esp_heap_caps.h"

PSRAMCanvas16::PSRAMCanvas16(int16_t w, int16_t h)
    : Arduino_Canvas(w, h, nullptr)
{
}

PSRAMCanvas16::~PSRAMCanvas16()
{
    if (_framebuffer)
    {
        heap_caps_free(_framebuffer);
        _framebuffer = nullptr;
    }
}

bool PSRAMCanvas16::begin(int32_t speed)
{
    (void)speed;
    if (!_framebuffer)
    {
        size_t bytes = static_cast<size_t>(_width) * static_cast<size_t>(_height) * sizeof(uint16_t);
        _framebuffer = static_cast<uint16_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!_framebuffer)
        {
            return false;
        }
        memset(_framebuffer, 0, bytes);
    }
    return true;
}
