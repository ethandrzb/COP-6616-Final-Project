# COP-6616 Final Project
- This repo contains STM32CubeIDE projects for each of the scenarios tested. Import each into STM32CubeIDE, build, and flash to an STM32 Nucleo H755ZI-Q board to reproduce results.

## Folder Structure
Each project contains two subprojects (one for each core) with separate source and include directories. Both subprojects share the "Common" folder at the root of each project using a symlink
### Files of Interest
- <Scenario x root>/CM4 or CM7/Core/Src/main.c: Main source file for each core
- <Scenario x root>/Common/
  - Inc
    - common.h: Memory layout for ring buffer
    - ring_buffer.h: Header for ring buffer
  - Src
    - ring_buffer.c: Source for ring buffer
  - **Scenario 2 only**: ssd1306 (OLED display driver)