@echo off
set "PIXI_DDS_CHOICE=fast"
if exist "%PIXI_PROJECT_ROOT%\.pixi\dds-selection" (
    set "PIXI_DDS_CHOICE="
    set /p PIXI_DDS_CHOICE=<"%PIXI_PROJECT_ROOT%\.pixi\dds-selection"
)
if "%PIXI_DDS_CHOICE%"=="fast" (
    set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
) else if "%PIXI_DDS_CHOICE%"=="cyclone" (
    set "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
) else if "%PIXI_DDS_CHOICE%"=="zenoh" (
    set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"
) else (
    echo Invalid DDS selection in %PIXI_PROJECT_ROOT%\.pixi\dds-selection: %PIXI_DDS_CHOICE% 1>&2
    exit 1
)
set "PIXI_DDS_CHOICE="
