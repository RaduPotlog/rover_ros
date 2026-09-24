<h1>Modbus library for modern C++</h1>

[![CMake on multiple platforms](https://github.com/Mazurel/Modbus/actions/workflows/cmake-multi-platform.yml/badge.svg?branch=master)](https://github.com/Mazurel/Modbus/actions/workflows/cmake-multi-platform.yml)
[![Clang formatting checker](https://github.com/Mazurel/Modbus/actions/workflows/clang-format.yml/badge.svg?branch=master)](https://github.com/Mazurel/Modbus/actions/workflows/clang-format.yml)


Library for high level Modbus frame/packet manipulation, including encoding and decoding, all written in modern C++17.

Additionally, the library contains a reference TCP implementation (Linux only).

> **This fork is TCP only.** Fork commit `c5d53cb` removed the Serial/RTU implementation,
> the examples and the test suite. See "Local changes to this fork" below.


# Table of content
- [Why](#why)
- [Important Concept](#important-concept)
- [Possibilities](#possibilities)
- [Dependencies](#dependencies)
- [Status](#status)
- [Building it](#building-it)
- [Api](#api)

# Why

When I was working on my last project and tried to find a good C++ Modbus library (other than Qt) I was unable to find it.
That is why I have decided to share my own implementation of Modbus.

# Important Concept

This library is **mainly** for providing Modbus logic.
It doesnt aim to have best communication implementation, as it is usually HW-specific.
It gives user ability to create Modbus frames using high level api and convert them to raw bytes or show them as string.
That is why *Modbus Core* is OS independent and can be easily used with other communication frameworks,
assuming that you compiler supports at least C++17.

The codec and the communication code are separate CMake targets in this fork - link
`Modbus_Core` alone if you supply your own transport.

# Possibilities

Quick example of what Modbus Core can do:

Code:

```c++
#include "MB/modbusException.hpp"
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"

// Create simple request
MB::ModbusRequest request(1, MB::utils::ReadDiscreteOutputCoils, 100, 10);

std::cout << "Stringed Request: " << request.toString() << std::endl;

std::cout << "Raw request:" << std::endl;

// Get raw represenatation for request
std::vector<uint8_t> rawed = request.toRaw();

// Method for showing byte
auto showByte = [](const uint8_t &byte) {
    std::cout << " 0x" << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(byte);
};

// Show all bytes
std::for_each(rawed.begin(), rawed.end(), showByte);
std::cout << std::endl;

// Create CRC and pointer to its bytes
uint16_t CRC = MB::utils::calculateCRC(rawed);
auto CRCptr  = reinterpret_cast<uint8_t *>(&CRC);

// Show byted CRC for request
std::cout << "CRC for the above code: ";
std::for_each(CRCptr, CRCptr + 2, showByte);
std::cout << std::endl;

auto request1 = MB::ModbusRequest::fromRaw(rawed);
std::cout << "Stringed Request 1 after rawed: " << request1.toString() << std::endl;

// Add CRC to the end of raw request so that it can be loaded with CRC check
rawed.insert(rawed.end(), CRCptr, CRCptr + 2);
auto request2 = MB::ModbusRequest::fromRawCRC(rawed); // Throws on invalid CRC
std::cout << "Stringed Request 2 after rawed: " << request2.toString() << std::endl;
```

Output:
```bash
Stringed Request: Read from output coils, from slave 1, starting from address 100, on 10 registers
Raw request:
 0x01 0x01 0x00 0x64 0x00 0x0a
CRC for the above code:  0xfd 0xd2
Stringed Request 1 after rawed: Read from output coils, from slave 1, starting from address 100, on 10 registers
Stringed Request 2 after rawed: Read from output coils, from slave 1, starting from address 100, on 10 registers
```

# Dependencies

None beyond the C++17 standard library. (Upstream listed libnet for the TCP module; it was
never actually linked and the include has been removed - see below.)

# STATUS

The codec is functional. The TCP communication code works on Linux only. Several real
defects in it were fixed in this fork - an fd leak, a frozen transaction id, a wrong MBAP
length encoding and a `std::terminate()` on accept failure - all listed below.

# How to learn Modbus ?

Just use [Simply modbus](http://www.simplymodbus.ca/FAQ.htm).

# Building it

This is an **ament package**, so colcon builds it in dependency order from a consumer's
`<depend>rover_modbus</depend>` - there is no manual install step:

```bash
colcon build --packages-select rover_modbus
```

Consumers then link the target they actually need:

```cmake
find_package(rover_modbus REQUIRED)
target_link_libraries(<your target> rover_modbus::Modbus_Core)  # frame codec only
target_link_libraries(<your target> rover_modbus::Modbus_Tcp)   # codec + sockets
```

Headers install to `<prefix>/include/MB/`, so includes read `#include <MB/modbusRequest.hpp>`.

## Local changes to this fork

This tree is a hard fork; upstream is no longer merged. Beyond fork commit `c5d53cb`
(which removed `example/`, `tests/` and all Serial/RTU support, and flattened
`include/MB/TCP/*` into `include/MB/*`), the following changed here. Each site carries a
`// Modified 2026 by Mechatronics Academy:` comment.

* **Packaging.** Added `package.xml` and `ament_package()`. Previously this was a
  plain-CMake project that colcon skipped entirely, so it had to be
  `sudo make install`ed to `/usr/local` before the workspace would build - a step that
  lived in `rover_ros/README.md` and in the platform Dockerfile. Both are gone.
* **Two targets instead of one.** `Modbus_Core` is now the OS-independent frame codec and
  `Modbus_Tcp` the POSIX sockets, so a consumer that only builds frames can link the codec
  alone and have that enforced by the linker.
* **Headers install namespaced** into `include/MB/` rather than flat into `include/`.
  Generic names like `connection.hpp` and `server.hpp` at the root of an include directory
  that is on every dependent's search path are a shadowing hazard.
* **Fixed a re-introduced fd leak** in `Connection::with()`. Upstream `397a169` closed the
  socket before throwing on a failed `connect()`; `c5d53cb` dropped that fix while moving
  the file. With the rover's shipped URDF (retry forever, 1 s apart) an unreachable device
  leaked one fd per second inside the `controller_manager` process until `EMFILE`.
* **Fixed the transaction id**, which was never incremented - every frame went out with
  id 0, making `awaitResponse()`'s id check degenerate, so after one timeout the next call
  would accept the previous transaction's buffered payload as a fresh reading.
* **Fixed the MBAP length encoding**, which pushed the high half of a `uint32_t` (always 0)
  followed by a truncated low half. Correct by accident under 256 bytes, endian-dependent,
  wrong above.
* **Whole-frame reads and complete writes.** `::send()`'s return was ignored (partial write
  -> truncated frame) and a single `::recv()` into a 1024-byte buffer was assumed to be a
  whole frame.
* **`Server::awaitConnection()` no longer calls `std::terminate()`.** It did a bare
  `throw;` outside any catch block; it now returns `std::nullopt`, which its signature
  already allowed.
* **`Server` no longer leaks.** Two `new int(1)` passed to `setsockopt` and never freed,
  return values unchecked, and `_serverfd` leaked on a failed `bind()` (the destructor
  cannot run when the constructor throws).
* **Added `setTimeout()`/`setRequestTimeout()`.** `_timeout` was private with no setter, so
  the response timeout was frozen at 500 ms for every consumer; both timeouts are now also
  carried across moves, which they were not.
* **Dropped `<libnet.h>`.** No `libnet_*` symbol is used and the target never linked
  `-lnet`, so `libnet1-dev` came out of the Dockerfile and README too. It had been
  supplying `::close()` and `::inet_addr()` transitively, so `<unistd.h>` and
  `<arpa/inet.h>` are now included explicitly.
* **Removed dead build config.** The `MODBUS_EXAMPLE` / `MODBUS_TESTS` /
  `MODBUS_SERIAL_COMMUNICATION` options pointed at directories `c5d53cb` deleted
  (`-DMODBUS_TESTS=ON` failed configure), and `.gitmodules` still declared a `tests/googletest`
  submodule that no longer exists.

The rover-facing client that uses this library lives in
`rover_ros/rover_transport/rover_modbus_driver`.

# API

API documentation is generated using [Doxygen](https://www.doxygen.nl) and it is available online under this [link](https://mazurel.github.io/docs/modbus/index.html).
If you want, you can also generate it yourself !
