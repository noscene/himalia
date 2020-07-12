# icestorm Installation
FROM ubuntu:bionic
MAINTAINER Sven Braun <s.braun@mobileonly.com>

RUN export DEBIAN_FRONTEND="noninteractive" && apt-get update  && apt-get -y install wget vim git curl python3 sox && \
cd && \
git clone https://github.com/microsoft/uf2 && \
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sh && \
arduino-cli core update-index && \
arduino-cli core install arduino:samd && \
cd && \ 
echo "# arduino-cli.yaml" > arduino-cli.yaml && \
echo "board_manager:" >> arduino-cli.yaml && \
echo "  additional_urls:" >> arduino-cli.yaml && \
echo "    - https://adafruit.github.io/arduino-board-index/package_adafruit_index.json" >> arduino-cli.yaml && \
echo "    - https://noscene.github.io/doppler4arduino/package_doppler_index.json" >> arduino-cli.yaml && \
arduino-cli core update-index && \
arduino-cli core install arduino:samd dadamachines_doppler:samd && \
arduino-cli board listall && \
cd /root/.arduino15/packages/dadamachines_doppler/hardware/samd/1.2.9 && \
echo "Himalia.name= Himalia (SAMD51)" >> boards.txt && \
echo "Himalia.vid.0=0x16D0" >> boards.txt && \
echo "Himalia.pid.0=0x0E4F" >> boards.txt && \
echo "Himalia.vid.1=0x16D0" >> boards.txt && \
echo "Himalia.pid.1=0x0E40" >> boards.txt && \
echo "Himalia.upload.tool=bossac18" >> boards.txt && \
echo "Himalia.upload.protocol=sam-ba" >> boards.txt && \
echo "Himalia.upload.maximum_size=1032192" >> boards.txt && \
echo "Himalia.upload.offset=0x4000" >> boards.txt && \
echo "Himalia.upload.use_1200bps_touch=true" >> boards.txt && \
echo "Himalia.upload.wait_for_upload_port=true" >> boards.txt && \
echo "Himalia.upload.native_usb=true" >> boards.txt && \
echo "Himalia.build.mcu=cortex-m4" >> boards.txt && \
echo "Himalia.build.f_cpu=120000000L" >> boards.txt && \
echo "Himalia.build.usb_product=\"Himalia\"" >> boards.txt && \
echo "Himalia.build.usb_manufacturer=\"IO Instruments\"" >> boards.txt && \
echo "Himalia.build.board=Himalia     " >> boards.txt && \
echo "Himalia.build.core=arduino" >> boards.txt && \
echo "Himalia.build.extra_flags=-D__SAMD51J20A__ -DSNDCore -D__SAMD51__ {build.usb_flags} -D__FPU_PRESENT  -DARM_MATH_CM4 -mfloat-abi=hard -mfpu=fpv4-sp-d16" >> boards.txt && \
echo "Himalia.build.ldscript=linker_scripts/gcc/flash_with_bootloader.ld" >> boards.txt && \
echo "Himalia.build.openocdscript=openocd_scripts/arduino_zero.cfg" >> boards.txt && \
echo "Himalia.build.variant=IOCore" >> boards.txt && \
echo "Himalia.build.variant_system_lib=" >> boards.txt && \
echo "Himalia.build.vid=0x16D0" >> boards.txt && \
echo "Himalia.build.pid=0x0E4F" >> boards.txt && \
echo "Himalia.bootloader.tool=openocd" >> boards.txt && \
echo "#Himalia.bootloader.file=doppler/bootloader-doppler-v2.0.0-33-ge603fc3.bin" >> boards.txt && \
echo "Himalia.compiler.arm.cmsis.ldflags="-L{build.variant.path}" -larm_cortexM4lf_math -mfloat-abi=hard -mfpu=fpv4-sp-d16" >> boards.txt && \
echo "Himalia.menu.cache.on=Enabled" >> boards.txt && \
echo "Himalia.menu.cache.on.build.cache_flags=-DENABLE_CACHE" >> boards.txt && \
echo "Himalia.menu.cache.off=Disabled" >> boards.txt && \
echo "Himalia.menu.cache.off.build.cache_flags=" >> boards.txt && \
cd /root/Arduino/libraries && git clone https://github.com/Dennis-van-Gils/SAMD51_InterruptTimer && \
cd && \
chmod 755 /root/uf2/utils/uf2conv.py && \
arduino-cli lib install "Adafruit ZeroTimer Library" && \
arduino-cli lib install "SdFat - Adafruit Fork" && \
cd /root/Arduino/libraries && git clone https://github.com/noscene/Adafruit_SPIFlash && cd && \
echo "arduino-cli compile -v  --fqbn  dadamachines_doppler:samd:Himalia -o myBin /PRJ/Himalia/HimaliaSketch/" > build.sh && \
echo "uf2/utils/uf2conv.py -b 0x4000 -c -o mybin.uf2 myBin.bin" >> build.sh && \
echo "cp mybin.uf2 /USB/Himalia/" >> build.sh
#COPY variant_SNDCore /root/.arduino15/packages/dadamachines_doppler/hardware/samd/1.2.9/variants/SNDCore


