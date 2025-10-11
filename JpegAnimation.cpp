#include "JpegAnimation.h"

#include <Arduino.h>

JpegAnimation::JpegAnimation(const JpegAnimationFrame *frames,
                             int frameCount,
                             uint32_t frameDelayMs,
                             FrameDecoder decoder,
                             int decodeOptions)
    : m_frames(frames),
      m_frameCount(frameCount),
      m_frameDelayMs(frameDelayMs),
      m_decoder(decoder),
      m_decodeOptions(decodeOptions)
{
}

void JpegAnimation::start(int x, int y)
{
    if (!m_frames || m_frameCount <= 0 || !m_decoder)
        return;

    m_active = true;
    m_frameIndex = 0;
    m_posX = x;
    m_posY = y;
    m_nextFrameMs = millis() + m_frameDelayMs;
}

void JpegAnimation::stop()
{
    m_active = false;
}

void JpegAnimation::update()
{
    if (!m_active)
        return;

    uint32_t now = millis();
    if ((int32_t)(now - m_nextFrameMs) < 0)
        return;

    if (m_frameIndex + 1 < m_frameCount)
    {
        ++m_frameIndex;
        m_nextFrameMs = now + m_frameDelayMs;
    }
    else
    {
        m_active = false;
    }
}

bool JpegAnimation::render(uint16_t *dest, int pitch, int bufferHeight) const
{
    if (!m_active || !dest || !m_frames || !m_decoder)
        return false;

    if (m_frameIndex < 0 || m_frameIndex >= m_frameCount)
        return false;

    const JpegAnimationFrame &frame = m_frames[m_frameIndex];
    if (!frame.data || frame.size == 0)
        return false;

    if (!m_decoder(dest, pitch, bufferHeight, m_posX, m_posY, frame.data, frame.size, m_decodeOptions))
    {
        Serial.printf("Failed to decode animation frame %d\n", m_frameIndex);
        return false;
    }

    return true;
}

bool JpegAnimation::isActive() const
{
    return m_active;
}

void JpegAnimation::setFrameDelay(uint32_t delayMs)
{
    m_frameDelayMs = delayMs;
}

void JpegAnimation::setDecodeOptions(int options)
{
    m_decodeOptions = options;
}
