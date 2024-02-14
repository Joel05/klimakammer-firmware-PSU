# Klimakammer Firmware Template


## Description

This is a template for creating firmware for the various klimakammer modules.

## Getting Started

### Dependencies

* VS Code
* PlatformIO for VS Code

### Installing

* Download VS Code from Microsoft at https://code.visualstudio.com/
* Install PlatformIO from the VS Code Extension Store
* Click "use this template" on this GitHub Page, and create your new repo for the firmware
* Clone the repo
* Open the repo with platformio

For reference, refer to https://docs.platformio.org/en/latest/integration/ide/vscode.html#quick-start

### Building program

* Click the "build" button on the bottom bar (looks like a checkmark) to build the program


### Preparing to upload the program
When using PlatformIO for the first time you have to follow all of these steps. When using a new pico you only need to do steps 3 and 7
1. Download [Zadig](https://zadig.akeo.ie/)(necessary to change the default driver for the pico)
2. Open Zadig (grant admin privileges)
3. Connect the pico while holding the BOOTSEL button (a new drive should appear in windows)
4. Select the pico from the dropdown menu in zadig (is called "RP2 Boot" or something similar)
5. Select the WINUSB driver in the dropdown menu below
6. Click install driver, and wait for it to finish
7. Copy the "firmware.uf2" file from "/.pio/build/pico" inside of your project to the "RPI-RP2" drive

Now the pico is flashed with your code, as well as the firmware for the PlatformIO uploader. 
### Uploading the program
If you want to upload a new program, you can just:
* Click the "Upload" button on the bottom bar (looks like an arrow) to build and flash the program


## Help

Advise for known issues will stand here, when issues arise.

## Authors

[Joel Rupp](https://github.com/Joel05) 

## Version History

* 0.1
    * Initial Release

## License

This project is licensed under the [GPL-3.0 license] License - see the LICENSE file for details

## Acknowledgments

Inspiration, code snippets, etc.
* [Readme template]([https://github.com/matiassingers/awesome-readme](https://gist.github.com/DomPizzie/7a5ff55ffa9081f2de27c315f5018afc#file-readme-template-md))
