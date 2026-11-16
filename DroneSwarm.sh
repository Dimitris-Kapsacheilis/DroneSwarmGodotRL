#!/bin/sh
echo -ne '\033c\033]0;DroneSwarm\a'
base_path="$(dirname "$(realpath "$0")")"
"$base_path/DroneSwarm.x86_64" "$@"
