# Himalia - IO Instruments

Himalia is a eurorack module with an samd51 cortex m4 mcu. The firmeware can build complete
in a docker enviroment, so it has no library conflicts on build machine.
The firmware is written in arduino style so it is easy to use and modify.

## Docker build chain based on arduino cli
the commands are used on macos in terminal. if using windows or linux may need
some changes in syntax or directorys.

### build docker image
just clone repo and create the docker images. it contains all arduino stuff, board package and convert tool for own samples
just start the terminal and:
```
cd ~/Documents/GitHub
git clone https://github.com/noscene/himalia
cd ~/Documents/GitHub/Himalia
docker build -t iocore IOCore/
```

### start docker instance and mount USB,Arduino BoardPackage and himalia src files
when docker image is created, it can run to compile it.
if using mac you can mount /Volumes dir into docker container to copy image after build direct by usb mass storage
you need connect the himalia module by micro usb and double tap the reset button to enter bootloader mode.
```
cd ~/Documents/GitHub/Himalia
export MOUNTPOINT=`pwd`
docker run -it -v $MOUNTPOINT:/PRJ -v /Volumes:/USB  -v $MOUNTPOINT/IOCore/variant_IOCore:/root/.arduino15/packages/dadamachines_doppler/hardware/samd/1.2.9/variants/IOCore  iocore bash
```

### inside docker: build all in one
double tap reset on module to go into bootloader mode. Then it works like a simple usb stick
and you only need the xxx.uf2 file into root for update firmware. Call the build.sh script
to compile all and copy firmware into module.
```
. /PRJ/build.sh 
```

### how to convert sample files into header
samples will linked into src code by convert it in the right format and make a c-header file
from binary to use it in compiler.
```
sox /PRJ/samples/test1.aif  --bits 16 --encoding unsigned-integer --endian little -c 1 s10.raw
xxd -i s10.raw > /PRJ/IOCore/HimaliaSketch/s10.h
sed -i -r 's/unsigned/const unsigned/g' /PRJ/IOCore/HimaliaSketch/s10.h
```

### here is the uf2 bootloader for the samd51
https://github.com/noscene/uf2-samdx1
