# PocketAI-XIAO_ESP32

⚠️ **Development Status**: This project is not yet fully developed. For a working implementation that can already be
used, please refer to [PocketAI-ESP32Cam](https://github.com/WavJaby/PocketAI-ESP32Cam).

A compact AI-powered camera system built on XIAO ESP32S3 with touch LCD display, image capture, Wi-Fi connectivity, and
interactive control functionality.

## Features

### Current Development Focus

- 🔄 **Smooth Preview Screen**: Currently studying methods to achieve fluid real-time camera preview
- 👆 **Touch Support**: Implementing touch functionality for intuitive user interaction
- 📷 **Camera Integration**: AI-powered image capture and processing
- 📶 **Wi-Fi Connectivity**: Wireless communication capabilities
- 🎛️ **Control Interface**: Interactive button and touch controls

### Planned Features

- Real-time AI image processing
- Wireless image transmission
- Touch-based user interface
- Battery-powered operation
- Compact form factor design

## Hardware

### Display

- **Model**: 1.69inch Touch LCD Module
- **Specifications**: Touch-enabled color display
- **Documentation**: [Waveshare 1.69inch Touch LCD Module](https://www.waveshare.com/wiki/1.69inch_Touch_LCD_Module)

### Main Controller

- **Model**: XIAO ESP32S3 Sense
- **Features**:
    - ESP32-S3 dual-core processor
    - Built-in camera support
    - Wi-Fi and Bluetooth connectivity
    - Compact form factor
- **Documentation**: [Seeed Studio XIAO ESP32S3 Sense](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/)

## Related Projects

- [PocketAI Backend Server](https://github.com/WavJaby/PocketAI-Backend) - Backend server for AI image processing
- [PocketAI ESP32 Cam](https://github.com/WavJaby/PocketAI-ESP32Cam) - ESP32 Cam version (fully functional)

**Note:** This repository is specifically designed for XIAO ESP32S3. For ESP32-CAM support, please
visit [PocketAI-ESP32Cam](https://github.com/WavJaby/PocketAI-ESP32Cam).

## Development Environment

### Prerequisites

- ESP-IDF

### Building the Project

1. **Clone the repository**:
   ```bash
   git clone https://github.com/WavJaby/PocketAI-XIAO_ESP32.git
   cd PocketAI-XIAO_ESP32
   ```

2. **Build the project**:
   ```bash
   idf.py build
   ```

3. **Flash to device**:
   ```bash
   idf.py -p PORT flash monitor
   ```

## Current Development Challenges

1. **Display Performance**: Optimizing frame rate for smooth camera preview
2. **Touch Integration**: Implementing reliable touch detection and gesture recognition
3. **Memory Management**: Efficient handling of image data and display buffers
4. **Power Optimization**: Balancing performance with battery life
