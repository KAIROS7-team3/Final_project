# Keep the shared non-ROS plc_core package importable from the source workspace.
_plc_project_root="$(builtin cd "${AMENT_CURRENT_PREFIX}/../.." && pwd)"
ament_prepend_unique_value PYTHONPATH "${_plc_project_root}"
unset _plc_project_root
