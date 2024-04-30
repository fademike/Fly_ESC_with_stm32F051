# Fly_ESC_with_stm32F051
ESC for brushless motors.

git clone https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware
git current project
cd AM32-MultiRotor-ESC-firmware
git apply am32ref_f051.patch (from current project)
May be to change make/tools: GCC_REQUIRED_VERSION ?= 10.3.1
make f051
STM32_Programmer_CLI -c port=swd -w ./obj/AM32_AM32REF_F051_2.00.bin 0x08000000 –verify -hardRst

![Image alt](https://github.com/fademike/Fly_ESC_with_stm32F051/raw/master//ESC_f051_top.png)
![Image alt](https://github.com/fademike/Fly_ESC_with_stm32F051/raw/master//ESC_f051_bottom.png)



