// animationsDefintions.h
// Animation frame declarations shared across the project.
#pragma once

#include "JpegAnimation.h"

constexpr int kExplosionFrameCount = 30;
constexpr int kIntroFrameCount = 60;

extern const JpegAnimationFrame g_explosionFrames[kExplosionFrameCount];
extern const JpegAnimationFrame g_introFrames[kIntroFrameCount];
