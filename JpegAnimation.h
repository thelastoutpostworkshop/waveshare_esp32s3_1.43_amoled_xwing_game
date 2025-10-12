// JpegAnimation.h
// Reusable helper to play JPEG frame sequences on RGB565 buffers.
#pragma once

#include <cstdint>
#include <cstddef>

struct JpegAnimationFrame
{
    const uint8_t *data;
    size_t size;
};

class JpegAnimation
{
public:
    using FrameDecoder = bool (*)(uint16_t *dest,
                                  int pitch,
                                  int bufferHeight,
                                  int x,
                                  int y,
                                  const uint8_t *data,
                                  size_t size,
                                  int decodeOptions);

    JpegAnimation(const JpegAnimationFrame *frames,
                  int frameCount,
                  uint32_t frameDelayMs,
                  FrameDecoder decoder,
                  int decodeOptions = 0);

    void start(int x, int y);
    void stop();
    void update();
    bool render(uint16_t *dest, int pitch, int bufferHeight) const;
    bool isActive() const;
    void setFrameDelay(uint32_t delayMs);
    void setDecodeOptions(int options);
    int currentFrame() const;

private:
    const JpegAnimationFrame *m_frames = nullptr;
    int m_frameCount = 0;
    uint32_t m_frameDelayMs = 0;
    FrameDecoder m_decoder = nullptr;
    int m_decodeOptions = 0;
    bool m_active = false;
    int m_frameIndex = 0;
    int m_posX = 0;
    int m_posY = 0;
    uint32_t m_nextFrameMs = 0;
};
