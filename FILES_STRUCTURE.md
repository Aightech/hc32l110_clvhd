# File Structure

```
├── Build                       # Build results
├── Examples                    # Example code
├── Libraries
│   ├── CMSIS
│   │   ├── base_types.h
│   │   ├── cmsis_gcc.h
│   │   ├── core_cm0plus.h
│   │   ├── core_cmFunc.h
│   │   ├── core_cmInstr.h
│   │   ├── hc32l110.h          # HC32L110 hardware register definitions
│   │   ├── startup_hc32l110.c  # Startup code
│   │   ├── system_hc32l110.c
│   │   └── system_hc32l110.h
│   ├── Debug                   # Printf support
│   ├── HC32L110_Driver         # MCU peripheral driver
│   │   ├── inc
│   │   └── src
│   └── LDScripts
│       ├── hc32l110x4.ld       # Link description script for 16K types
│       └── hc32l110x6.ld       # Link description script for 32K types
├── Misc
│   ├── Flash_Algorithms
│   │   ├── HC32L110B4_C4.FLM   # Flash algorithm file for 16K types
│   │   └── HC32L110B6_C6.FLM   # Flash algorithm file for 32k types
│   ├── flash.jlink             # JLink download commander script
│   ├── HC32L110.svd            # CMSIS System View Description file for debug
│   ├── HDSC.HC32L110.1.0.3.pack 
│   ├── JLinkDevicesAddon.xml   # Device addon for JLinkDevices.xml
│   └── pyocd.yaml              # PyOCD configuration file
├── LICENSE
├── Makefile                    # Make config
├── README.md
├── rules.mk                    # Pre-defined rules include in Makefile 
└── User                        # User application code
```
