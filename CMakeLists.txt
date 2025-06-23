cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ipotcas2)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/ethernet.c
    Core/Src/lan8742.c
    Core/Src/msc_disk.c
    Core/Src/network.c
    Core/Src/receive.c
    Core/Src/transmit.c
    Core/Src/usb_descriptors.c
    Core/Src/codec.c
    tinyusb/src/host/hub.c
    tinyusb/src/host/usbh.c
    tinyusb/src/portable/nordic/nrf5x/dcd_nrf5x.c
    tinyusb/src/portable/synopsys/dwc2/hcd_dwc2.c
    tinyusb/src/portable/synopsys/dwc2/dwc2_common.c
    tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c
    tinyusb/src/portable/analog/max3421/hcd_max3421.c
    tinyusb/src/portable/valentyusb/eptri/dcd_eptri.c
    tinyusb/src/portable/ehci/ehci.c
    tinyusb/src/portable/bridgetek/ft9xx/dcd_ft9xx.c
    tinyusb/src/portable/nxp/lpc_ip3511/dcd_lpc_ip3511.c
    tinyusb/src/portable/nxp/khci/dcd_khci.c
    tinyusb/src/portable/nxp/khci/hcd_khci.c
    tinyusb/src/portable/nxp/lpc17_40/dcd_lpc17_40.c
    tinyusb/src/portable/nxp/lpc17_40/hcd_lpc17_40.c
    tinyusb/src/portable/ohci/ohci.c
    tinyusb/src/portable/wch/dcd_ch32_usbhs.c
    tinyusb/src/portable/wch/dcd_ch32_usbfs.c
    tinyusb/src/portable/template/dcd_template.c
    tinyusb/src/portable/template/hcd_template.c
    tinyusb/src/portable/mentor/musb/hcd_musb.c
    tinyusb/src/portable/mentor/musb/dcd_musb.c
    tinyusb/src/portable/st/typec/typec_stm32.c
    tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    tinyusb/src/portable/dialog/da146xx/dcd_da146xx.c
    tinyusb/src/portable/chipidea/ci_fs/dcd_ci_fs.c
    tinyusb/src/portable/chipidea/ci_hs/dcd_ci_hs.c
    tinyusb/src/portable/chipidea/ci_hs/hcd_ci_hs.c
    tinyusb/src/portable/sunxi/dcd_sunxi_musb.c
    tinyusb/src/portable/microchip/samx7x/dcd_samx7x.c
    tinyusb/src/portable/microchip/samg/dcd_samg.c
    tinyusb/src/portable/microchip/pic/dcd_pic.c
    tinyusb/src/portable/microchip/pic32mz/dcd_pic32mz.c
    tinyusb/src/portable/microchip/samd/dcd_samd.c
    tinyusb/src/portable/nuvoton/nuc121/dcd_nuc121.c
    tinyusb/src/portable/nuvoton/nuc120/dcd_nuc120.c
    tinyusb/src/portable/nuvoton/nuc505/dcd_nuc505.c
    tinyusb/src/portable/renesas/rusb2/rusb2_common.c
    tinyusb/src/portable/renesas/rusb2/hcd_rusb2.c
    tinyusb/src/portable/renesas/rusb2/dcd_rusb2.c
    tinyusb/src/portable/espressif/esp32sx/dcd_esp32sx.c
    tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c
    tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c
    tinyusb/src/portable/raspberrypi/rp2040/hcd_rp2040.c
    tinyusb/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
    tinyusb/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
    tinyusb/src/portable/sony/cxd56/dcd_cxd56.c
    tinyusb/src/portable/mindmotion/mm32/dcd_mm32f327x_otg.c
    tinyusb/src/portable/ti/msp430x5xx/dcd_msp430x5xx.c
    tinyusb/src/common/tusb_fifo.c
    tinyusb/src/typec/usbc.c
    tinyusb/src/class/cdc/cdc_rndis_host.c
    tinyusb/src/class/cdc/cdc_host.c
    tinyusb/src/class/cdc/cdc_device.c
    tinyusb/src/class/video/video_device.c
    tinyusb/src/class/net/ncm_device.c
    tinyusb/src/class/net/ecm_rndis_device.c
    tinyusb/src/class/dfu/dfu_rt_device.c
    tinyusb/src/class/dfu/dfu_device.c
    tinyusb/src/class/midi/midi_host.c
    tinyusb/src/class/midi/midi_device.c
    tinyusb/src/class/usbtmc/usbtmc_device.c
    tinyusb/src/class/bth/bth_device.c
    tinyusb/src/class/audio/audio_device.c
    tinyusb/src/class/msc/msc_host.c
    tinyusb/src/class/msc/msc_device.c
    tinyusb/src/class/hid/hid_device.c
    tinyusb/src/class/hid/hid_host.c
    tinyusb/src/class/vendor/vendor_host.c
    tinyusb/src/class/vendor/vendor_device.c
    tinyusb/src/device/usbd_control.c
    tinyusb/src/device/usbd.c
    tinyusb/src/tusb.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    tinyusb/src tinyusb/hw
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
