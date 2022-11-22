```rtl_asio``` - fast and simple I/Q spectrum single-client server for RTL2832 based DVB-T receivers.

## Features

 * Rewritten on modern C++20.
 * Uses incredible standalone [asio](https://think-async.com/Asio/) library for communcation and signals handling.
 * Fully compatible with existing ```rtl_tcp``` clients, also command line options (except ```-n```) same as for original ```rtl_tcp```. 
 * Print EEPROM info at run.
 * Dongle starts only after client connects and completely stops when client disconnects.
 * No slow stuff (memory allocation/mutexes/linked lists/ring buffers etc.) during I/Q data sending.
 * Depends only on [libusb](https://github.com/libusb/libusb) library.
 * Tested on [Orange Pi Zero](http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-Zero.html), [Armbian](https://www.armbian.com/) and with original [RTL-SDR V3 Dongle](https://www.rtl-sdr.com/rtl-sdr-blog-v-3-dongles-user-guide/).
 
## Build

To build the project execute the following commands:

```
apt-get install libusb-1.0-0-dev
git clone https://github.com/vasilenkoalexey/rtl_asio.git
cd rtl_asio
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
```
