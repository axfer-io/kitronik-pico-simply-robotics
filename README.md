# kitronik-simply-robotics

C++ library for controlling the Kitronik Simply Robotics Board using the
Raspberry Pi Pico SDK.

Provides control over:

-   Servos (PIO-based PWM)
-   DC motors (hardware PWM)
-   Stepper motors

using direct GPIO and microcontroller peripherals.

------------------------------------------------------------------------

## Features

-   Native RP2040 / RP2350 implementation
-   Servo control via PIO
-   Motor control via hardware PWM
-   No external controller required
-   Deterministic timing
-   Real-time capable

------------------------------------------------------------------------

## Requirements

-   Raspberry Pi Pico SDK
-   hardware_pwm
-   hardware_pio

Supported targets:

-   RP2040
-   RP2350

------------------------------------------------------------------------

## Installation

Add to your project:

    lib/
    └── kitronik_simply_robotics/

In your CMakeLists.txt:

``` cmake
add_subdirectory(lib/kitronik_simply_robotics)

target_link_libraries(app
    kitronik_simply_robotics
)
```

------------------------------------------------------------------------

## Example

``` cpp
#include "kitronik_simply_robotics.hpp"

KitronikSimplyRobotics board(true);

board.servos[0].goToPosition(90);
board.motors[0].on('f', 50);
board.steppers[0].step('f');
```

------------------------------------------------------------------------

## API

### Constructor

``` cpp
KitronikSimplyRobotics(bool centreServos = true);
```

### Servo

``` cpp
servos[index].goToPosition(degrees);
servos[index].goToRadians(radians);
servos[index].goToPeriod(period_us);
```

Range: `0–7`

### Motor

``` cpp
motors[index].on(direction, speed);
motors[index].off();
```

Range: `0–3`

### Stepper

``` cpp
steppers[index].step(direction);
steppers[index].halfStep(direction);
```

Range: `0–1`

------------------------------------------------------------------------

## Hardware

Uses:

-   PIO for servo timing
-   Hardware PWM for motor control
-   Direct GPIO

------------------------------------------------------------------------

## License

MIT
