# TERRACOTTA

TERRACOTTA is a handheld embedded system inspired by classic virtual pets such as Tamagotchi. The project explores low-level embedded development by combining a microcontroller, persistent memory, graphics rendering, sensors, and audio into an interactive device.

The goal of the project is to build a fully custom embedded platform capable of running a persistent virtual pet environment while demonstrating core embedded systems concepts including hardware integration, state machine design, memory management, and peripheral communication.

---

## Features

- **Persistent Virtual Pet System**  
  The device maintains game state across power cycles using external non-volatile memory.

- **Graphical Display Interface**  
  A TFT SPI display renders sprites, menus, and pet animations.

- **Sprite Rendering Engine**  
  Graphics are stored in external memory and streamed to the display for rendering.

- **Audio Feedback**  
  A piezo provides sound effects and alerts.

- **User Input Controls**  
  Buttons allow interaction with the virtual pet.

---

## System Architecture

The TERRACOTTA device integrates several hardware subsystems around a central microcontroller.

Main components:

Microcontroller  (Atmega328p)

- Task scheduling and system control
- EEPROM graphics storage

SPI Bus  
- TFT Display  

GPIO  
- Buttons

PWM
- Piezo

---

## Firmware Architecture

The firmware is written in C/C++ and runs on bare-metal AVR hardware.

Key modules include:

- **Hardware Initialization**  
  Configures timers, communication interfaces, and peripherals.

- **Scheduler System**  
  Timer interrupts manage periodic tasks such as sensor reads and animation updates.

- **Graphics Engine**  
  Handles sprite loading, buffering, and rendering to the TFT display.

- **Input Handling**  
  Reads button or joystick states and translates them into game actions.

- **Audio Control**  
  Drives PWM output to generate sound effects.

## Development Environment

Firmware is developed using:

- PlatformIO  
- AVR-GCC toolchain  
- Embedded C / C++

Testing is performed on:

- ATmega328P hardware  
- Breadboard prototype hardware  

## Future Improvements

Planned enhancements include:

- Custom PCB design
- More advanced sprite animation system, current system is limited to only four colors!
- Expanded virtual pet behavior

---

## Educational Goals

The TERRACOTTA project serves as a platform for exploring core embedded systems topics:

- Peripheral communication (SPI / I2C)
- Interrupt-driven scheduling
- Memory management
- Embedded graphics rendering
- Hardware integration

---

## Acknowledgments

TERRACOTTA draws inspiration from classic handheld virtual pets while exploring modern embedded development techniques. The project is intended as both a learning platform and a demonstration of full-stack embedded hardware and firmware design.
