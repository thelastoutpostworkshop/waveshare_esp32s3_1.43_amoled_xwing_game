// animationsDefintions.h
// Animation frame declarations shared across the project.
#pragma once

#include "JpegAnimation.h"

constexpr int kExplosionFrameCount = 10;
constexpr int kIntroFrameCount = 30;
constexpr int kGameOverFrameCount = 24;
constexpr int kYouWinFrameCount = 17;
constexpr int kBlinkFrameCount = 15;
constexpr int kMedalFrameCount = 17;

extern const JpegAnimationFrame g_explosionFrames[kExplosionFrameCount];
extern const JpegAnimationFrame g_introFrames[kIntroFrameCount];
extern const JpegAnimationFrame g_gameOverFrames[kGameOverFrameCount];
extern const JpegAnimationFrame g_youWinFrames[kYouWinFrameCount];
extern const JpegAnimationFrame g_blinkFrames[kBlinkFrameCount];
extern const JpegAnimationFrame g_medalFrames[kMedalFrameCount];
