# Sun Sensor Test

This repo contains rust code to run on the IC of my Sun Sensor Test board, which I'm using to test a sensor for my $1K Cubesat. Hardware design files for this board can be found [in the 1KCubeSat Hardware repo](https://github.com/rgw3d/1KCubeSat_Hardware/tree/master/sun_sensor_test)

The IC used is the STM32L051R8

## Why Rust?

C and C++ based development for STM32 platforms is very well established, and is something I have done before. Rust provided the perfect opportunity to try something new.

## Tooling setup with VSCode (Highlevel)

- The Cortex-Debug extension in VSCode provides the wrapper around GDB for debugging and stepping through the code.
- See [`.vscode/launch.json`](.vscode/launch.json) for Cortex-Debug launch configurations
- See [`../svd/`](../svd/README.md) for SVD register description files for STM32 devices.
- OpenOCD is the Debugger, and OpenOCD uses STLink to flash the STM32.
- OpenOCD opens a port for GDB to connect
- See [`../openocd_cfg/`](../openocd_cfg/README.md) for OpenOCD configuration files.

### Actually getting the above tooling to work

> See [`../eps/README.md`](../eps/README.md) for more specifics.

## Run lib Unit tests

```bash
cargo test --lib --target=x86_64-unknown-linux-gnu --features=test
```

## Manually install on target MCU

If all else fails, here's a manual way to flash your micro

```bash
cargo build --release
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/sun_sensor_test out.bin
st-flash --debug --reset --freq=100K write out.bin 0x8000000
```

## [`cortex-m-quickstart`](https://github.com/rust-embedded/cortex-m-quickstart)

> See the cortex-m-quickstart repository for a template used to generate this repo
