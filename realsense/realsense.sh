#!/bin/bash

shell_type=${SHELL##*/}


shell_config="source ./install/setup.$shell_type"
shell_exec="exec $shell_type"

declare -A CAMS=(
  [camera_h]="230322271365"
  [camera_l]="230422271416"
  [camera_r]="230322274234"
)

COLOR_PROFILE="640x480x90"
DEPTH_PROFILE="640x480x90"

terminal_cmd() {
  local title="$1"
  local body="$2"

  gnome-terminal --tab --title="$title" -- "$shell_type" -ic "$body"
}

normalize_serial() 
{
  local s="${1//[[:space:]]/}"

  while [[ "$s" == _* ]]; do s="${s#_}"; done
  printf '%s' "$s"
}

serial_is_set() 
{
  local s="$1"
  [[ -n "${s}" && "${s}" != "_" ]]
}

serial_is_online() 
{
  local s="$1"
  [[ ${#ONLINE[@]} -eq 0 ]] && return 0
  [[ -n "${ONLINE[$s]:-}" ]]
}

launch_cam() 
{
  local cam_name="$1" raw_serial="$2"

  if ! serial_is_set "$raw_serial"; then
    return 0
  fi

  digits="$(normalize_serial "$raw_serial")"
  local sn="_${digits}"

  echo "$cam_name -> serial=$sn"
  terminal_cmd "$cam_name" "$shell_config
  ros2 launch realsense2_camera rs_launch.py \
  camera_name:=${cam_name} \
  depth_module.color_profile:=${COLOR_PROFILE} \
  depth_module.depth_profile:=${DEPTH_PROFILE} \
  serial_no:=${sn} \
  enable_color:=true \
  enable_depth:=true \
  enable_compressed:=true \
  enable_ir:=false
  $shell_exec"

  sleep 1
}

for cam in "${!CAMS[@]}"; do
  launch_cam "$cam" "${CAMS[$cam]}"
done