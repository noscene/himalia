#!/bin/bash

cd /PRJ/samples
for i in $( ls s*.wav ); do
    echo $i
    sox $i --bits 16 --encoding unsigned-integer --endian little -c 1 $(echo $i | cut -d'.' -f1).raw
    xxd -i $(echo $i | cut -d'.' -f1).raw > /PRJ/IOCore/HimaliaSketch/$(echo $i | cut -d'.' -f1).h
    sed -i -r 's/unsigned/const unsigned/g' /PRJ/IOCore/HimaliaSketch/$(echo $i | cut -d'.' -f1).h
done


