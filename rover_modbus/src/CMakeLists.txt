# Modified 2026 by Mechatronics Academy: split the single Modbus_Core target in two, and
# changed the header install from flat to include/MB/.
#
# The split: the request/response/exception/CRC codec is pure std C++17 with no syscalls,
# while connection.cpp / server.cpp are POSIX sockets and poll(). Keeping them in one
# library forced every consumer of the codec to link the socket code too. Consumers that
# only build and parse frames (e.g. rover_modbus_driver's _core library, which must stay
# OS-free) now link Modbus_Core alone and have that enforced by the linker rather than by
# convention.
#
# The install path: headers used to go flat into <prefix>/include via install(FILES ...),
# so consumers wrote #include <modbusRequest.hpp>. Under ament that puts generic names
# like connection.hpp, server.hpp and crc.hpp at the root of an include directory that is
# on every dependent's search path - a shadowing hazard. They now install to
# <prefix>/include/MB/ and consumers write #include <MB/modbusRequest.hpp>.

set(MODBUS_INCLUDE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../include)

# ---------------------------------------------------------------------------------------
# Modbus_Core - the OS-independent frame codec. No sockets, no syscalls.
# ---------------------------------------------------------------------------------------
add_library(Modbus_Core SHARED
    modbusException.cpp
    modbusRequest.cpp
    modbusResponse.cpp
    crc.cpp
)

set_target_properties(Modbus_Core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(Modbus_Core
    PUBLIC
        $<BUILD_INTERFACE:${MODBUS_INCLUDE_ROOT}>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        # Internal sources are inconsistent: most do #include "modbusFoo.hpp" (needs
        # include/MB on the path), crc.cpp does #include "MB/crc.hpp" (needs the include/
        # parent). Both are kept PRIVATE so neither leaks into the consumer-facing
        # interface above, which is include/ only - i.e. <MB/modbusFoo.hpp>.
        ${MODBUS_INCLUDE_ROOT}
        ${MODBUS_INCLUDE_ROOT}/MB
)

# ---------------------------------------------------------------------------------------
# Modbus_Tcp - the POSIX socket client and server. Linux only.
# ---------------------------------------------------------------------------------------
add_library(Modbus_Tcp SHARED
    connection.cpp
    server.cpp
)

set_target_properties(Modbus_Tcp PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(Modbus_Tcp PUBLIC Modbus_Core)

target_include_directories(Modbus_Tcp
    PUBLIC
        $<BUILD_INTERFACE:${MODBUS_INCLUDE_ROOT}>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${MODBUS_INCLUDE_ROOT}
        ${MODBUS_INCLUDE_ROOT}/MB
)

# ---------------------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------------------
install(DIRECTORY ${MODBUS_INCLUDE_ROOT}/MB DESTINATION include)

install(TARGETS Modbus_Core Modbus_Tcp
    EXPORT export_rover_modbus
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
