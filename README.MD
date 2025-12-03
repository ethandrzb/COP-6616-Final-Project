# COP-6616 Final Project
This repo contains STM32CubeIDE projects for each of the scenarios tested. **Each project implements both the algorithms and experiments to test them.** Import each into STM32CubeIDE, build, and flash to an STM32 Nucleo H755ZI-Q board to reproduce results.

# Video Demonstration: https://www.youtube.com/watch?v=14W7-2xvVIM

## Scenario 1: Unidirectional Communication Benchmark
The CM7 core periodically sends data of a predetermined size and a counter to the CM4 core. The CM4 core verifies that it received the data correctly by comparing the counter with the expected value. If the counter doesn't match the expected value, the test is over and the timer period is printed to the serial terminal. The timer period is decremented every 10 successful transfers until failure.
## Scenario 2: Compute Offload using Bidirectional Communication
This scenario generates a vector, x, computes y=sin(x) on each value in the vector, and renders it to an SSD1306 OLED display connected via I2C. The work is split between the two cores based on role, I/O and compute. The following experiments were run with this scenario:
    1. Baseline: CM4 core does all the work
    2. Ring buffer: CM4 sends x to CM7 to compute y=sin(x). CM7 sends result vector, y, back to CM4. CM4 displays results on OLED.
       1. Ring buffer with memcpy
       2. Ring buffer with DMA

## Folder Structure
Each project contains two subprojects (one for each core) with separate source and include directories. Both subprojects share the "Common" folder at the root of each project using a symlink
### Files of Interest
- `<Scenario x root>/CM4 or CM7/Core/Src/main.c`: Main source file for each core
- `<Scenario x root>/Common/`
  - `Inc`
    - `common.h`: Memory layout for ring buffer
    - `ring_buffer.h`: Header for ring buffer
  - `Src`
    - `ring_buffer.c`: Source for ring buffer
  - **Scenario 2 only**: `ssd1306` (OLED display driver)