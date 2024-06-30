# Aurora V Avionics Firmware
![HIVE Aurora V footer](./img/banner.png)


## Table of Contents
<!-- mtoc-start -->

* [Requirements](#requirements)
  * [Project Includes](#project-includes)
* [Getting Started](#getting-started)
  * [Using GitHub](#using-github)
  * [Coding Standard](#coding-standard)
    * [Formatting](#formatting)
    * [Library Naming](#library-naming)
    * [Library Functions](#library-functions)
    * [Struct Encapsulation](#struct-encapsulation)

<!-- mtoc-end -->

## Requirements

To successfully install and contribute to the project, ensure you have the following prerequisites installed:

- Git
- Keil uVision IDE
- GCC for arm (arm-none-eabi-gcc)
- Latest version of Aurora V Avionics libraries found [here](https://github.com/RMIT-AURC-Team/AuroraV-Avionics-lib/releases)

<!-- TODO: add in lib submodule as dependancy -->

##### Project Includes

In order to build the project you must ensure the correct include paths are added for the compiler to recognise the headers. These are provided as follows:

```shell
/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Core/Inc
/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Middlewares/Third_Party/FreeRTOS/Source/include
/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Drivers/CMSIS/Include
/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Drivers/CMSIS/Device/ST/STM32F4xx/Include
/path/to/AuroraV-Avionics-lib/inc
/path/to/AuroraV-Avionics-lib/inc/DSP/Include
/path/to/AuroraV-Avionics-lib/inc/DSP/PrivateInclude
/path/to/AuroraV-Avionics-lib/inc/CORE/Include
```

For clangd LSP configuration you can use the following:

```yaml
CompileFlags:
  Add: [
    "-I/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Core/Inc",
    "-I/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Middlewares/Third_Party/FreeRTOS/Source/include",
    "-I/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
    "-I/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS",
    "-I/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Drivers/CMSIS/Include",
    "-I/path/to/AuroraV-Avionics-firmware/AuroraV-Avionics/Drivers/CMSIS/Device/ST/STM32F4xx/Include",
    "-I/path/to/AuroraV-Avionics-lib/inc",
    "-I/path/to/AuroraV-Avionics-lib/inc/DSP/Include/",
    "-I/path/to/AuroraV-Avionics-lib/inc/DSP/PrivateInclude",
    "-I/path/to/AuroraV-Avionics-lib/inc/CORE/Include/"
]
```

## Getting Started
This document provides an introduction to using GitHub for development of Aurora V firmware libraries, as well as their coding standard and conventions. 

### Using GitHub
_Detailed instructions on using GitHub for this project will be provided here._ (Content to be added)

---

### Coding Standard
The specification of this standard exists to document and define the organisation and naming convention of code to create a well structured and cohesive project. 

These standards aren't to be strict and annoying, but to minimise points of failure and ensure a functional codebase with as little resistance as possible. Justification for these standards is provided to help understand their necessity.

To get started as simply as possible, copy the files located in ```/src``` within any library inside the [lib]() repository and follow the conventions you see there.

#### Formatting

For formatting the project it is recommended that ```clang-format``` is used with the following settings configured in the ```.clangd``` project file:

```yaml
AlignAfterOpenBracket: BlockIndent
AlignOperands: AlignAfterOperator
AlignTrailingComments:
  Kind: Always
  OverEmptyLines: 4
AllowAllParametersOfDeclarationOnNextLine: false
AllowShortLoopsOnASingleLine: true
BinPackParameters: false
BraceWrapping:
  AfterFunction: false
BreakBeforeBraces: Custom
BreakBeforeBinaryOperators: NonAssignment
ColumnLimit: 0
ReflowComments: true
```

#### Library Naming
> **Convention:**   
> Library names should be all lower case without spaces, with the naming convention ```lib<name>```. These names should be kept concise,  e.g. ```libcan```, ```libspi```.

The ```gcc``` toolchain by default expects libraries to be preceeded by ```lib```, so this simplifies the compilation step.

#### Library Functions
> **Convention:**   
> Functions should be proceeded by the library name separated by a single underscore. The library name in this case should follow proper capitilisation, particularly in the case of initialisms and acronyms; e.g. ```MemBuff_append()```, ```LoRa_send()```, ```CAN_init()```.

C does not provide functionality for namespacing or mangling, meaning it is possible for name conflicts that would cause failure to compile in the case that two libraries share functions or defintions of the same name.

Take, for example, a situation where ```CAN``` and ```SPI``` libraries both implement a function ```sendData()```. Attempting to compile the project will error out when linking these libraries due to name conflict.

#### Struct Encapsulation
> **Convention:**   
> Global variables should **not** be used within libraries. If a resource is required to be shared across functions, place them within a library struct with appropriate intitialisation. These structs should follow the same naming convention as library functions. It is not necessary, though it is preferred, to include library functions as pointers within these structs.

As mentioned, C does not support namespaces. Global variables can and will cause logic errors when multiple libraries implement globals sharing common names. The compiler may issue a warning however multiple definitions are legal in C and will not throw an error at compile time, this can be quite difficult to debug.

For an example see the [MemBuff](https://github.com/RMIT-AURC-Team/AuroraV-Avionics-lib/tree/master/membuff/src) implementation. An example skeleton of a library struct implementation can be found [here](https://github.com/RMIT-AURC-Team/AuroraV-Avionics-lib/tree/master/example/src); You may find it easier to copy the example directory entirely and simply rename the files and code elements to align with the library you are working on.

![HIVE Aurora V footer](./img/footer.png)
---
