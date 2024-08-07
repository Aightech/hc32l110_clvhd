

# Prerequisites

* Board using HC32L110 serial MCU
  * 16K Flash / 2K RAM: HC32L110C4UA, HC32L110C4PA, HC32L110B4PA;
  * 32K Flash / 4k RAM: HC32L110C6UA, HC32L110C6PA, HC32L110B6PA, HC32L110B6YA;
* Programmer
  * J-Link: J-Link OB programmer
  * PyOCD: DAPLink or J-Link 
* SEGGER J-Link Software and Documentation pack [https://www.segger.com/downloads/jlink/](https://www.segger.com/downloads/jlink/)
* Or PyOCD [https://pyocd.io/](https://pyocd.io/)
* GNU Arm Embedded Toolchain

## 1. Install GNU Arm Embedded Toolchain

Download the toolchain from [Arm GNU Toolchain Downloads](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) according to your pc architecture, extract the files

```bash
tar xvf gcc-arm-11.2-2022.02-x86_64-arm-none-eabi.tar.xz
cd /opt/gcc-arm/
sudo mv ~/Backup/linux/gcc-arm-11.2-2022.02-x86_64-arm-none-eabi/ .
sudo chown -R root:root gcc-arm-11.2-2022.02-x86_64-arm-none-eabi/
```
## 2. Install JLink Or PyOCD

### Option #1: Install SEGGER J-Link

Download and install JLink from [J-Link / J-Trace Downloads](https://www.segger.com/downloads/jlink/).

```bash
# installation command for .deb
sudo dpkg -i JLink_Linux_V770a_x86_64.deb
```
The default installation directory is */opt/SEGGER*

**Add HC32L110 Deivce Support**

JLink (currently 7.70e) doesn't provide out-of-box support for HC32L110, which need to be added manually. 

Create a folder `HDSC` under /opt/SEGGER/JLink/Devices, and copy the flash algorithm files to it.
```
Devices
├── Altera
├── AnalogDevices
├── ATMEL
├── Broadcom
├── ClouderSemi
├── HDSC
│   ├── HC32L110B4_C4.FLM
│   └── HC32L110B6_C6.FLM
├── Infineon
├── Samsung
├── ST
└── Zilog
```
Edit /opt/SEGGER/JLink/JLinkDevices.xml, add the following lines before `</DataBase>`
```xml
  <!--                 -->
  <!-- Huada (HDSC)    -->
  <!--                 -->
  <Device>
    <ChipInfo Vendor="HDSC" Name="HC32L110x4"  WorkRAMAddr="0x20000000" WorkRAMSize="0x800" Core="JLINK_CORE_CORTEX_M0"/>
    <FlashBankInfo Name="Flash_16K" BaseAddr="0x0" MaxSize="0x4000" Loader="Devices/HDSC/HC32L110B4_C4.FLM" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
  </Device>
  <Device>
    <ChipInfo Vendor="HDSC" Name="HC32L110x6"  WorkRAMAddr="0x20000000" WorkRAMSize="0x1000" Core="JLINK_CORE_CORTEX_M0"/>
    <FlashBankInfo Name="Flash_32K" BaseAddr="0x0" MaxSize="0x8000" Loader="Devices/HDSC/HC32L110B6_C6.FLM" LoaderType="FLASH_ALGO_TYPE_OPEN" AlwaysPresent="1"/>
  </Device>
```

### Option #2: Install PyOCD

Install from pip instead of apt repository because the version is 0.13.1+dfsg-1, which is too low to recognize J-Link probe

```bash
pip uninstall pyocd
```
This will install PyOCD into:
```
/home/[user]/.local/bin/pyocd
/home/[user]/.local/bin/pyocd-gdbserver
/home/[user]/.local/lib/python3.10/site-packages/pyocd-0.34.2.dist-info/*
/home/[user]/.local/lib/python3.10/site-packages/pyocd/*
```
.profile will take care of the PATH, run `source ~/.profile` to make pyocd command available

## 3. Clone This Repository

Clone this repository to local workspace
```bash
git clone https://github.com/IOsetting/hc32l110-template.git
```

# Building

## 1. Edit Makefile

* make sure ARM_TOOCHAIN points to the correct path
* If you use J-Link, FLASH_PROGRM can be jlink or pyocd
* If you use DAPLink, set FLASH_PROGRM to pyocd

```makefile
##### Project #####

PROJECT 		?= app
# The path for generated files
BUILD_DIR 		= Build


##### Options #####

# Programmer, jlink or pyocd
FLASH_PROGRM	?= pyocd

##### Toolchains #######

# path to gcc arm
ARM_TOOCHAIN 	?= /opt/gcc-arm/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin
# path to JLinkExe
JLINKEXE		?= /opt/SEGGER/JLink/JLinkExe
# JLink devices: HC32L110x4 or HC32L110x6
JLINK_DEVICE	?= HC32L110x4
# path to PyOCD
PYOCD_EXE		?= pyocd
# PyOCD device type: hc32l110 hc32l110b4pa hc32l110c4pa hc32l110c4ua hc32l110b6pa hc32l110c6pa hc32l110c6ua
PYOCD_DEVICE	?= hc32l110c4ua

##### Paths ############

# Link descript file, hc32l110x4.ld or hc32l110x6.ld
LDSCRIPT		= Libraries/LDScripts/hc32l110x4.ld
```

## 2. Compiling And Flashing

```bash
# clean source code
make clean
# build
make
# or make with verbose output
V=1 make
# flash
make flash
```

# Debugging In VSCode

Install Cortex-Debug extension in VSCode, and setup the debug settings

## .vscode/launch.json

```
    "configurations": [
        {
            "name": "Cortex Debug",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/Build/app.elf",
            "request": "launch",
            "type": "cortex-debug",
            "runToEntryPoint": "main",
            "servertype": "jlink",
            "device": "HC32L110X4", // HC32L110X6 for 32KB type
            "interface": "swd",
            "runToMain": true,
            "preLaunchTask": "build download", // task name configured in tasks.json
            // "preLaunchCommands": ["Build all"], // or cli command instead of task
            "svdFile": "${workspaceFolder}/Misc/HC32L110.svd", // Include svd to watch device peripherals
            "showDevDebugOutput": "parsed", // parsed, raw, vscode
            "swoConfig":
            {
                "enabled": true,
                "cpuFrequency": 24000000,
                "swoFrequency":  4000000,
                "source": "probe",
                "decoders":
                [
                    {
                        "label": "ITM port 0 output",
                        "type": "console",
                        "port": 0,
                        "showOnStartup": true,
                        "encoding": "ascii"
                    }
                ]
            }
        }
    ]
```
## .vscode/settings.json
```json
{
    "cortex-debug.gdbPath": "/opt/gcc-arm/gcc-arm-11.2-2022.02-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb",
    "cortex-debug.JLinkGDBServerPath": "/opt/SEGGER/JLink/JLinkGDBServerCLExe",
}
```

## rules.mk

Add extra compile options in rules.mk,
```makefile
# produce debugging information in DWARF format version 2
CFLAGS    += -g -gdwarf-2
```
And
```makefile
# Without optimization
OPT       ?= -O0
```

# Try Other Examples

Replace the source files of *User* folder with the source files from other example folder.

# About The HC32L110 Driver

Part of the driver has been **heavily modified** from its original version(HC32L110_DDL_Rev1.1.4). The interrupt callbacks are associated will
vector handler table directly and part of the peripheral functions were replaced with macros for efficiency.

# Reference

* Template (forked from): [https://github.com/IOsetting/hc32l110-template](https://github.com/IOsetting/hc32l110-template)
* HDSC Product Page: [https://www.hdsc.com.cn/Category82](https://www.hdsc.com.cn/Category82)
* Jeroen Domburg's HC32L110 SDK working with GCC/GDB/OpenOCD: [https://github.com/Spritetm/hc32l110-gcc-sdk](https://github.com/Spritetm/hc32l110-gcc-sdk)
* Jeffreyabecker's hc32l110 lib (with translated user manuals): [https://github.com/jeffreyabecker/hc32l110_lib](https://github.com/jeffreyabecker/hc32l110_lib)