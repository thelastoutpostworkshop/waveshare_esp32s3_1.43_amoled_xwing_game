# X-Wing Pursuit Game
## For the Waveshare ESP32-S3 1.43inch AMOLED Touch Display
<a href="https://www.buymeacoffee.com/thelastoutpostworkshop" target="_blank">
<img src="https://www.buymeacoffee.com/assets/img/custom_images/orange_img.png" alt="Buy Me A Coffee">
</a>

## Youtube Tutorial
[<img src="https://github.com/thelastoutpostworkshop/images/blob/main/X-Wing%20Game-1.png" width="500">](https://youtu.be/eDDZ7O_mwxU)

## Project Description
`X-Wing Pursuit` is an arcade-style reflex game built for the Waveshare ESP32-S3 1.43" AMOLED Touch Display.  
You pilot an X-Wing using the onboard IMU (tilt + twist), line up over the center target, and tap the screen to score hits before time runs out.  
The game includes animated intro/win/lose sequences and stores your best completion time in ESP32 NVS, so your high score survives power cycles.

## Features
- IMU-based ship movement with smooth inertia and damping.
- Touch-based hit detection when the ship is aligned with the target zone.
- Timed rounds with configurable win condition (hits required before countdown ends).
- Visual game states: intro, explosion effect, game over, victory, and medal/new-best feedback.
- Persistent best-time tracking saved in onboard non-volatile storage (NVS).
- High score screen with BOOT button support to clear saved best score.
- Double-buffered rendering and JPEG animation playback optimized for the AMOLED display.

## What You Can Customize
- **Game rules and difficulty** (`waveshare_esp32s3_1.43_amoled_xwing_game.ino`):
  - `ROUND_TARGET_HITS`, `ROUND_DURATION_SECONDS`
  - `XWING_TARGET_AREA`
- **Control feel** (`waveshare_esp32s3_1.43_amoled_xwing_game.ino`):
  - `ACCEL_SCALE`, `GYRO_SCALE`, `DAMPING`
  - `XWING_DIRECTION_MODE` (normal/inverted control modes)
- **HUD and layout positions** (`waveshare_esp32s3_1.43_amoled_xwing_game.ino`):
  - score/timer/sensor/target text position defines (for UI placement tuning)
- **Visual style and assets**:
  - X-Wing sprites and UI images in `images/`
  - animation frame sets in `images/gameintro`, `images/explosion`, `images/gameover`, `images/youwin`, `images/medal`, and `images/blinks`
  - fonts in `fonts/`


