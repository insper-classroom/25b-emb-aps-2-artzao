#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Required by TinyUSB
#define CFG_TUSB_MCU                 OPT_MCU_RP2040
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_FREERTOS
#endif


#define CFG_TUSB_RHPORT0_MODE       (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

// Memory for transfer buffers
#define CFG_TUD_ENDPOINT0_SIZE        64

// ------------- DEVICE CONFIGURATION ------------- //

// Enable HID
#define CFG_TUD_HID                  1

// Report sizes
#define CFG_TUD_HID_EP_BUFSIZE       64

// No CDC, MSC, MIDI, etc for now
#define CFG_TUD_MSC                  0
#define CFG_TUD_CDC                  0
#define CFG_TUD_MIDI                 0
#define CFG_TUD_VENDOR               0

#ifdef __cplusplus
}
#endif
