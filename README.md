# Himalia - IO Instruments

## RunDocker


```
cd ~/Documents/GitHub/Himalia
docker build -t iocore IOCore/


export MOUNTPOINT=`pwd`
docker run -it -v $MOUNTPOINT:/PRJ -v /Volumes:/USB  -v $MOUNTPOINT/IOCore/variant_IOCore:/root/.arduino15/packages/dadamachines_doppler/hardware/samd/1.2.9/variants/IOCore  iocore bash

# all in One
. /PRJ/build.sh 

# compile sketch
cd && arduino-cli compile -v  --fqbn  dadamachines_doppler:samd:Himalia -o myBin /PRJ/IOCore/HimaliaSketch/
# Convert to UF2
chmod 755 /root/uf2/utils/uf2conv.py
uf2/utils/uf2conv.py -b 0x4000 -c -o mybin.uf2 myBin.bin 
# UPLOAD to Board
cp mybin.uf2 /USB/theMCore/
```

