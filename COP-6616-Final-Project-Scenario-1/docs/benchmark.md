# Benchmark Goals
1. Measure performance of ring buffer
    - What are we measuring?
      - Throughput?
        - Similar to maximum transfer unit (MTU) in Ethernet?
          - "Packets" aren't really a thing in the ring buffer. The overhead of making and parsing "packets" might be helpful for more complex data transfer scenarios.
          - Requiring "packets" to be created would require at least some effort by the CPU
          - More small buffers or one large buffer?
      - Request latency?
        - Use DMA transfer complete callback?
        - Send event (SEV) instruction?
        - Find max rate that data can be written to the buffer before causing an overrun
          - Benchmark memcpy vs DMA
2. Isolate ring buffer as bottleneck
    - Non-ring buffer operations should be as light/minimal as possible
3. Benchmark should be about the hardware, not request handling
    - Use current approach as baseline and timer-automated DMA as experimental run

# Actual Benchmarking Ideas
## Counter Hot-Potato
### Description
- High rate small data benchmark
- CON: Requires both ring buffers

### Steps
1. Core 0 initializes counter to 0
2. Core 0 writes counter to buffer
3. Core 1 reads counter from buffer
4. Core 1 increments counter
5. Core 1 writes counter back to buffer
6. Core 0 reads counter from buffer
7. Core 0 increments counter
8. Core 0 writes counter back to buffer
9. Go to Step 3

### Measurements
- Time elapsed until either core reads a specific value from the counter

## Count but Verify
### Description
- Unidirectional data transfer
- Both cores increment counter. One core sends counter, other core receives counter and compares with expected value.

### Steps
1. Both cores initialize private counters to 0
2. Core 0 uses timer interrupts to increment counter, send data to Core 1, and increase interrupt frequency by some increment.
3. Core 1 continuously polls the buffer. If it has new data, it increments its own counter and compares it with the received data.

Test continues until Core 1 reports failure condition.
### Measurements
- Maximum request rate for small data (4 byte unsigned integer in this case). Make sure to run each core as both the sender and receiver b/c they run at different frequencies. Need to identify impact of CPU speed on request processing rate.

### Results (lowest ARR value before failure)
- Preliminary baseline with CM4 logging via UART (TIM6 ARR of ~3400 was found using manual binary search): 29411.7647058824 requests handled/second
- Baseline: 304 (328947.368421053 requests handled/second)
- BDMA enabled, but not used in ring buffer: 298
    - I'm not sure why this has an impact

### Hardware Configuration Notes
- Using TIM6, input clock 200 MHz, with prescalar 2 (100 MHz actual frequency)
- Counter period (ARR) register starts at 5000 and decreases by 1 every 100 iterations if the consumers's local counter stays in sync with the received value from the producing core
- Shared buffer is 1KB. Failure will not occur until the producing core overwrites data being read by the consuming core, so buffer size controls the delay between failure occurence and detection. Smaller buffers will fail sooner (e.g., min ARR with 16B buffer is 369 vs 304 with 1KB buffer)
- Wanted to use BDMA used for DMA tests b/c it's in the same domain as SRAM4 (D3), but it does not have access to resources outside the D3 domain
- Using DMA1 for DMA tests
- No compiler optimization used (-O0)

### Notes
- UART and string functions are REALLY expensive (removing them allowed ARR to decrease from 1010 to 304 before failure)