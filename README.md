# Aurora V Avionics Firmware
This repository contains the project code implementing firmware to run on board the Aurora V avionics system. 

## Table of Contents
<!-- mtoc-start -->

* [Getting Started](#getting-started)
  * [Using GitHub](#using-github)
  * [Coding Standard](#coding-standard)
    * [Library Naming](#library-naming)
    * [Library Functions](#library-functions)
    * [Struct Encapsulation](#struct-encapsulation)

<!-- mtoc-end -->

## Getting Started
This document provides an introduction to using GitHub for development of Aurora V firmware libraries, as well as their coding standard and conventions. 

### Using GitHub
---

### Coding Standard
---
The specification of this standard exists to document and define the organisation and naming convention of code to create a well structured and cohesive project. 

These standards aren't to be strict and annoying, but to minimise points of failure and ensure a functional codebase with as little resistance as possible. Justification for these standards is provided to help understand their necessity.

<!-- TODO: add in link URLs -->
To get started as simply as possible, copy the files located in ```/src``` within any library inside the [lib]() repository and follow the conventions you see there.

#### Library Naming
---
> Library names should be all lower case without spaces, with the naming convention ```lib<name>```. These names should be kept concise,  e.g. ```libcan```, ```libspi```.

Justification
: The ```gcc``` toolchain by default expects libraries to be preceeded by ```lib```, so this simplifies the compilation step.

#### Library Functions
---
> Functions should be proceeded by the library name separated by a single underscore. The library name in this case should follow proper capitilisation, particularly in the case of initialisms and acronyms; e.g. ```MemBuff_append()```, ```LoRa_send()```, ```CAN_init()```.

Justification
: C does not provide functionality for namespacing or name mangling, meaning it is possible for name conflicts that would cause failure to compile in the case that two libraries share functions or defintions of the same name.

Take, for example, a situation where ```CAN``` and ```SPI``` libraries both implement a function ```sendData()```. Attempting to compile the project will error out when linking these libraries due to name conflict.

#### Struct Encapsulation
> Global variables should **not** be used within libraries. If a resource is required to be shared across functions, place them within a library struct with appropriate intitialisation. It is not necessary, though it is preferred, to include library functions as pointers within these structs.

<!-- TODO: add in link URLs -->
For an example see the [MemBuff]() implementation. An example skeleton of a library struct implementation can be found [here]().

Justification
: As mentioned, C does not support namespaces. Global variables can and will cause logic errors when multiple libraries implement globals sharing common names. The compiler may issue a warning however multiple definitions are legal in C and will not throw an error at compile time, this can be quite difficult to debug.
