#!/bin/sh

for file in *.crt *.key; do
    sudo chowner root:root $file
    sudo chmod 600 $file
done
