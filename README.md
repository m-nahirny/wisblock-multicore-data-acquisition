# wisblock-multicore-data-acquisition
Demonstrate multicore capability on a RAK11310 WisBlock Core module. This program acquires and displays accelerometer data on core 1 while simultaneously reading the RP2040 internal temperature and displaying it using core 0.

This was adapted from the lis3dh_ic2 example in the Raspberry Pi Pico SDK.

## Hardware

 * RAK11310 WisBlock Core module (https://docs.rakwireless.com/Product-Categories/WisBlock/RAK11310/Datasheet/#overview)
 * RAK19001 WisBlock Dual IO Base Board (https://docs.rakwireless.com/Product-Categories/WisBlock/RAK19001/Datasheet/#wisblock-dual-io-base-board-overview)
 * RAK1904 WisBlock 3-axis Acceleration Sensor Module (https://docs.rakwireless.com/Product-Categories/WisBlock/RAK1904/Datasheet/)

### Default Pinout

| RAK11310 |
| ----------------- |
| GPIO 2 | I2C1SDA |
| GPIO 3 | I2C1SCL |
| GPIO 20 | I2C0SDA |
| GPIO 21 | I2C0SCL |


## Cloning

```sh
git clone --recurse-submodules https://github.com/m-nahirny/wisblock-multicore-data-acquisition.git 
```

## Building

1. [Set up the Pico C/C++ SDK](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
2. Set `PICO_SDK_PATH`
```sh
export PICO_SDK_PATH=/path/to/pico-sdk
```
3. Create `build` dir, run `cmake` and `make`:
```
```
4. Copy example `.uf2` to RAK11310 when in BOOT mode.

