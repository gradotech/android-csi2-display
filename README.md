# Android CSI2 Display #

The purpose of this App is to:
- Read RAW RGB frames direclty from **/dev/video0** and render them using GStreamer and openGL;
- Transmit user's touch coordinates over UART;
- Trigger a GPIO upon transmission.

It is optimized to use the GPU and hardware encoders/decoders without overloading the CPU.

Tested on Raspberry Pi 5 and Rockchip Rock 5C.

Based on the [Android tutorial 3: Video](https://gstreamer.freedesktop.org/documentation/tutorials/android/video.html?gi-language=c).

Developed by [Grado Technologies](https://gradotech.eu/) (customers@gradotech.eu)
