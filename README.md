![1408Hyper 2024-25 repo banner](https://raw.githubusercontent.com/helloworld3200/1408Hyper-2024VRC-Code/main/readme-assets/banner1.png)

<center><h1> ⚡<em><strong>1408H</em></strong>yper @ VRC 2024-25</center></h1>

[![Github Actions Makefile CI](https://github.com/helloworld3200/1408Hyper-2024VRC-Code/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/helloworld3200/1408Hyper-2024VRC-Code/actions/workflows/c-cpp.yml)

> 1408H's source code for the 2024-25 "High Stakes" season of VRC.

### Sponsored by [Teehee Dental Works](https://teehee.sg/)

**1408Hyper-2024VRC-Code** was designed with portability in mind - we understand that situations can quickly change at any competition.  

Our code is hot-swappable, making heavy use of **abstract classes** and **templates**
to allow for us to rapidly change and test different pieces of code when time is of the essence.

## Included Libraries
- Created with [`PROS` API](https://github.com/purduesigbots/pros)
as this has better documentation
- [**fmt**](https://fmt.dev/11.0/) for text formatting. Polyfill of C++20's `std::format` as  PROS currently doesn't support it.

#### Main file at `src/main.cpp`. To run Make, first install [ARM G++](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads).