# Changelog

## [Unreleased]

### Added
- `ModbusPLCConfig` for ROS2-independent PLC serial and address settings.
- `ModbusPLCClient` for direct pymodbus coil/register reads, writes, and pulse control.
- `PLCError` and `PLCConfigError` for explicit PLC failure handling.
- Semantic system-state output mapping via `ModbusPLCClient.set_system_state()`.
- `pymodbus`-backed XBC-DR14E Modbus RTU implementation and lazy `plc_core`
  public exports for dependency-light enum/client imports.
- `plc_error` system event support for PLC actuator/read failure logging.

### Changed
- Hardware target and documentation from XBC-DR10E assumptions to XBC-DR14E.
- `PLCClient.set_state()`, `set_error()`, and `set_estop()` now return
  `PLCStatus`, matching `ModbusPLCClient`.
