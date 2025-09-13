#!/bin/bash

workspace=$(pwd)

shell_type=${SHELL##*/}
shell_exec="exec $shell_type"

# Trein
gnome-terminal --title="train" -x $shell_type -i -c "cd ${workspace}; cd ../act; conda activate act; python train.py --num_episodes -1; $shell_exec"   
