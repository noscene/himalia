# Himalia - IO Instruments

## Docker build chain based on arduino cli


### build docker image
```
cd ~/Documents/GitHub/Himalia
docker build -t iocore IOCore/
```

### start docker instance and mount USB,Arduino BoardPackage and himalia src files
```
export MOUNTPOINT=`pwd`
docker run -it -v $MOUNTPOINT:/PRJ -v /Volumes:/USB  -v $MOUNTPOINT/IOCore/variant_IOCore:/root/.arduino15/packages/dadamachines_doppler/hardware/samd/1.2.9/variants/IOCore  iocore bash
```

### inside docker: build all in one
```
. /PRJ/build.sh 
```


### how to convert sample files into header
```
sox /PRJ/samples/test1.aif  --bits 16 --encoding unsigned-integer --endian little -c 1 s10.raw
xxd -i s10.raw > /PRJ/IOCore/HimaliaSketch/s10.h
sed -i -r 's/unsigned/const unsigned/g' /PRJ/IOCore/HimaliaSketch/s10.h
```


