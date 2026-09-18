^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rover_modbus_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-09-17)
------------------
* Extracted the Modbus TCP client from ``rover_hardware_interface`` into a reusable
  ament package. It was previously linked ``PRIVATE`` against a plain-CMake
  ``Modbus_Core`` that colcon could not see, so no other package could depend on it.
* Library only - no bridge node. Modbus TCP is request/response rather than a byte
  stream, and the ros2_control safety controller needs in-process calls rather than a
  topic round-trip on the e-stop path.
* Split into ``rover_modbus_driver_core`` (domain + application, links only the
  OS-independent frame codec) and ``rover_modbus_driver_ros`` (sockets + rclcpp).
* Replaced the client's ``rclcpp::Logger`` member with an injected ``LoggerPort``, so the
  core library is ROS-free and unit tests need no ROS context. Connection-retry warnings
  now reach ``/rosout`` instead of ``std::cerr``.
* Injected the transport through a factory, which made the wire encoding testable.
* Added a virtual destructor to the transport port. It was held by ``unique_ptr`` through
  the base without one, so deleting it was undefined behaviour and
  ``~ModbusTcpConnection`` never ran - leaking the socket fd on every teardown.
  ``close()`` was a documented no-op, so nothing else closed it either.
* ``close()`` now actually releases the socket.
* Removed a no-op catch-and-rethrow in the TCP transport.
* ``kModbusDeviceId`` is ``static constexpr`` rather than a per-instance ``const``
  member, which had been suppressing the implicit copy-assignment operator.
* Dropped the ``const`` qualifiers on ``CoilInfo``'s members, which made the struct
  non-assignable.
* New tests: the wire encoding against a fake transport (including the read-only-coil
  guard, previously uncovered on a safety-relevant path), and a full round trip against a
  real ``MB::TCP::Server``.
