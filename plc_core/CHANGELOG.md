# Changelog

## [Unreleased]

### Added
- `ModbusPLCConfig` for ROS2-independent PLC serial and address settings.
- `ModbusPLCClient` for direct pymodbus coil/register reads, writes, and pulse control.
- `PLCError` and `PLCConfigError` for explicit PLC failure handling.
- Semantic system-state output mapping via `ModbusPLCClient.set_system_state()`.
