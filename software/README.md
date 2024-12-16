# Scampi Firmware

Pre-built binaries for the firmware can be found in the [releases](https://github.com/mackieks/scampi/releases) section.

## Firmware

The Scampi firmware is written in AVR C with no external dependencies. It uses [PlatformIO](https://platformio.org/) to build and flash the firmware, providing a consistent build environment and an easy way to manage toolchains.

### Prerequisites

- [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) for VS Code
- UPDI programmer (e.g., [ATMEL-ICE](https://www.microchip.com/en-us/development-tool/atatmel-ice), [Adafruit UPDI Friend](https://www.adafruit.com/product/5879), [SerialUPDI](https://www.tindie.com/products/mcudude/serialupdi-programmer/)).

### Building

To build the firmware in VS Code, open the `firmware` folder in VS Code, and let the PlatformIO extension configure the project. Then click the Build button at the bottom (check mark icon).

### Flashing

Flashing the firmware requires a UPDI programmer. You can use the official [ATMEL-ICE](https://www.microchip.com/en-us/development-tool/atatmel-ice), or a cheaper programmer such as the [Adafruit UPDI Friend](https://www.adafruit.com/product/5879) or MCUdude's [SerialUPDI](https://www.tindie.com/products/mcudude/serialupdi-programmer/).

You need to change line 8 of `platformio.ini` to match the programmer you're using.

- `atmelice_updi` for ATMEL-ICE
- `serialupdi` for SerialUPDI or UPDI Friend

See [this page](https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/PlatformIO.md#upload_protocol) for a full list of config options.

The Scampi PCB must be powered from the 3.3V and GND pads for programming to succeed! 

In VS Code, click the Upload button (arrow icon) to flash the firmware. Ensure your UPDI programmer is connected before clicking the button!
