# joltc

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/NBT22/joltc/blob/main/LICENSE)

[JoltPhysics](https://github.com/jrouwe/JoltPhysics) C interface.

## About this fork
I made this fork to add in some missing bindings as well as fix a few inconsistencies with the bindings which were really bothering me. My version is not tested on any compiler other than GCC using C11 with GNU extensions actively in use throughout the header. I have made this version to work with a [specific project](https://github.com/droc101/c-game-engine), and since that project only officially supports x86_64 with GCC, that's the only compatibility I am concerned about. Additionally, I may add additional bindings, structs, or other project-specific code to the `joltc.h` header. If you are interested in using this fork, feel free to go ahead, but I will not be resolving any issues that you have with it, be it because of my changes or because of something upstream.
