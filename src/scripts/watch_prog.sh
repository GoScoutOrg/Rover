#!/bin/bash
process=main.py
makerun="/home/pi/rover/src/scripts/run_prog.sh"

if ps ax | grep -v grep | grep $process > /dev/null
then
    exit
else
    $makerun
fi

exit
