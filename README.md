# wisblock-lorawan
Demonstrate multicore capability on a RAK11310 WisBlock Core module. This program acquires and displays accelerometer data on core 1 while simultaneously reading the RP2040 internal temperature and displaying it using core 0.

## Hardware

 * RAK11310 WisBlock Core module (https://docs.rakwireless.com/Product-Categories/WisBlock/RAK11310/Datasheet/#overview)
 * RAK19001 WisBlock Dual IO Base Board (https://docs.rakwireless.com/Product-Categories/WisBlock/RAK19001/Datasheet/#wisblock-dual-io-base-board-overview)

### Default Pinout

| RAK11310 |
| ----------------- |
| GPIO 10 | SCK |
| GPIO 11 | MOSI |
| GPIO 12 | MISO |
| GPIO 13 | NSS / CS |
| GPIO 14 | RESET |
| GPIO 15 | BUSY |
| GPIO 29 | DIO1 |

GPIO pins are configured in src/include/rak11310/board-config.h.

## Examples

See [examples](examples/) folder.

There is a `config.h` file to your ABP or OTAA node configuration for each example.

## Cloning

```sh
git clone --recurse-submodules https://github.com/m-nahirny/wisblock-lorawan.git 
```

## Building

1. [Set up the Pico C/C++ SDK](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
2. Set `PICO_SDK_PATH`
```sh
export PICO_SDK_PATH=/path/to/pico-sdk
```
3. Create `build` dir, run `cmake` and `make`:
```
mkdir build
cd build
cmake .. -DPICO_BOARD=pico
make
```
4. Copy example `.uf2` to RAK11310 when in BOOT mode.
