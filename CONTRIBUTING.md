# Contributing

Thanks for improving AirQualitySensors. Keep changes small enough to review and explain the hardware assumptions behind them.

## Development workflow

1. Create a branch from `main`.
2. Keep credentials and local addresses in `Core/Inc/app_config.local.h`; never commit that file.
3. Build both firmware presets.
4. Run the host tests.
5. Update the README, protocol document, CubeMX file, and tests when behavior changes.
6. Open a pull request describing the change, target hardware, and validation performed.

Required local checks:

```text
cmake --preset debug
cmake --build --preset debug --parallel
cmake --preset release
cmake --build --preset release --parallel
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host-tests --parallel
ctest --test-dir build/host-tests --output-on-failure
```

## Code expectations

- Use bounded buffer operations and validate HAL/driver return values.
- Keep interrupt handlers short and use ISR-safe synchronization APIs.
- Do not block while interrupts are disabled.
- Preserve the previous interrupt state when entering a local critical section.
- Put shared measurements through `measurement_store` instead of new global variables.
- Keep secrets, proprietary binary libraries, generated build output, and machine-specific IDE state out of commits.
- Avoid editing generated STM32 files unless the change is also represented in `AirQualitySensors.ioc` or clearly documented as a protected user-code change.
- Add platform-independent parsing and formatting behavior to the host test suite.

Hardware changes should state the exact MCU/board revision, sensor modules, ESP32 AT firmware, wiring, and test duration. CI proves compilation and host logic only; it does not prove electrical or real-time behavior.
