_pixi_dds_choice=fast
_pixi_dds_file="${PIXI_PROJECT_ROOT}/.pixi/dds-selection"
if [ -f "$_pixi_dds_file" ]; then
    _pixi_dds_choice="$(cat "$_pixi_dds_file")"
fi

case "$_pixi_dds_choice" in
    fast) export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ;;
    cyclone) export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ;;
    zenoh) export RMW_IMPLEMENTATION=rmw_zenoh_cpp ;;
    *)
        echo "Invalid DDS selection in $_pixi_dds_file: $_pixi_dds_choice" >&2
        exit 1
        ;;
esac
unset _pixi_dds_choice _pixi_dds_file
