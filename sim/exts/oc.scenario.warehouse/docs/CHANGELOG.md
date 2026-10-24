# Changelog

## [0.3.0] - 2026-10-24
### Added
- `autoplay` setting.

### Changed
- `autostart` no longer plays the Scenario unless `autoplay` is on.

## [0.2.0] - 2026-10-24
### Added
- `autostart`, `ros` and `obstacles` settings.
- `ObjectManager` (`oc.scenario.warehouse.objects`): `show()`, `hide()`, `visible()` and `OBSTACLES` for the obstacle props, edited in the session layer.
- The `ros` setting also activates or deactivates the `/clock` graph (`/World/Graph/ROS2_Clock`).
- 200 Hz physics rate; `isaacsim.robot_setup.virtual_gantry` is disabled.

## [0.1.0] - 2026-10-24
### Added
- Initial version.
