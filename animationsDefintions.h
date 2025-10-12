// animationsDefintions.h
// Animation frame declarations shared across the project.
#pragma once

#include "JpegAnimation.h"

constexpr int kExplosionFrameCount = 30;
constexpr int kIntroFrameCount = 60;
constexpr int kGameOverFrameCount = 60;
constexpr int kYouWinFrameCount = 60;
constexpr int kBlinkFrameCount = 30;

extern const JpegAnimationFrame g_explosionFrames[kExplosionFrameCount];
extern const JpegAnimationFrame g_introFrames[kIntroFrameCount];
extern const JpegAnimationFrame g_gameOverFrames[kGameOverFrameCount];
extern const JpegAnimationFrame g_youWinFrames[kYouWinFrameCount];
extern const JpegAnimationFrame g_blinkFrames[kBlinkFrameCount];
