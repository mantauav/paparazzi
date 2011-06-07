

#serial USB (e.g. /dev/ttyACM0)


ap.CFLAGS +=  -DUSE_USB_SERIAL -DUSE_USB_HIGH_PCLK

ap.srcs += $(SRC_ARCH)/usb_ser_hw.c $(SRC_ARCH)/lpcusb/usbhw_lpc.c $(SRC_ARCH)/lpcusb/usbcontrol.c
ap.srcs += $(SRC_ARCH)/lpcusb/usbstdreq.c $(SRC_ARCH)/lpcusb/usbinit.c $(SRC_ARCH)/lpcusb/examples/printf.c

