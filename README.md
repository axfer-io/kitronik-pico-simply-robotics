# kitronik-simply-robotics

C++ library for controlling the Kitronik Simply Robotics Board using the
Raspberry Pi Pico SDK.

Provides deterministic, low-level control over:

-   Servos (PIO-based PWM)
-   DC motors (hardware PWM)
-   Stepper motors

using direct GPIO and native RP2040 / RP2350 peripherals.

No external PWM controller required.

------------------------------------------------------------------------

## Features

-   Native RP2040 / RP2350 implementation\
-   Servo control using PIO state machines\
-   Hardware PWM motor control\
-   Deterministic timing and real-time safe\
-   No dynamic allocation\
-   No interrupts required\
-   Individual servo calibration support (pulse range and offset)
-   Full control over pulse timing in microseconds

------------------------------------------------------------------------

## Requirements

-   Raspberry Pi Pico SDK\
-   hardware_pwm\
-   hardware_pio\
-   hardware_clocks

Supported targets:

-   RP2040 (Pico / Pico W)
-   RP2350 (Pico 2 / Pico 2 W)

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

// Servo control
board.servos[0].goToPosition(90);

// Motor control
board.motors[0].on('f', 50);

// Stepper control
board.steppers[0].step('f');
```

------------------------------------------------------------------------

## Servo Calibration (NEW)

Different servos often have slightly different electrical pulse
requirements.

This library allows per-servo calibration without affecting others.

### Default pulse range

    500 µs → 0°
    2500 µs → 180°

These defaults work for most servos.

------------------------------------------------------------------------

### Adjust pulse range

``` cpp
board.servos[3].setPulseRangeUs(500, 2200);
```

------------------------------------------------------------------------

### Apply offset correction

``` cpp
board.servos[3].setPulseOffsetUs(-15);
```

------------------------------------------------------------------------

### Direct pulse control

``` cpp
board.servos[3].goToPeriod(1500);
```

------------------------------------------------------------------------

Calibration is applied per-servo and does not affect other channels.

------------------------------------------------------------------------

## API

### Constructor

``` cpp
KitronikSimplyRobotics(bool centreServos = true);
```

------------------------------------------------------------------------

### Servo

``` cpp
servos[index].goToPosition(degrees);
servos[index].goToRadians(radians);
servos[index].goToPeriod(period_us);

servos[index].setPulseRangeUs(minUs, maxUs);
servos[index].setPulseOffsetUs(offsetUs);
```

Range: `0–7`

------------------------------------------------------------------------

### Motor

``` cpp
motors[index].on(direction, speed_percent);
motors[index].off();
```

Range: `0–3`

------------------------------------------------------------------------

### Stepper

``` cpp
steppers[index].step(direction);
steppers[index].halfStep(direction);
```

Range: `0–1`

------------------------------------------------------------------------

## Hardware Architecture

Uses:

-   PIO state machines for servo PWM generation\
-   Hardware PWM slices for motor control\
-   Direct GPIO control\
-   hardware_clocks for precise timing

Fully deterministic.

------------------------------------------------------------------------

## License

MIT
