#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIR="$(readlink -f $DIR)"
WS_ROOT="$(readlink -f $DIR/../../..)"
SETUP_FILE="$WS_ROOT/devel/setup.bash"
source $SETUP_FILE
rospack profile
