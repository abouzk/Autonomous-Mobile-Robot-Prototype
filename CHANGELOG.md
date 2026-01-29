# Changelog

## Portfolio Cleanup & Fixes
Resurrected this project to document the architecture and clean up the code for my portfolio.

### Fixes
* **Turn Logic:** Fixed a bug in `turn()` where the robot would stop moving if just *one* wheel reached its target, instead of waiting for *both*. Changed the loop condition from `OR` to `AND`.
* **Safety Priority:** Rewrote `checkObstacle()` so it doesn't conflict with the main drive loop. Now it strictly handles the emergency stop without trying to drive the motors itself.

### Cleanup
* **Comments:** Added headers to functions to explain the math (ticks-per-meter, etc.).
* **Refactor:** Removed unused variables (`button`, `readA1`) that were left over from testing.
* **Params:** Replaced hardcoded PWM numbers (e.g., 255) with variables like `TURN_SPEED` so they are easier to tune.
* **Formatting:** General text refinement and spacing corrections.
