#ifndef FUSB300_H
#define FUSB300_H

/*
 * Register Mapping
 */
#define REG_DEVICEID (0x01)
#define REG_SWITCH_0 (0x02)
#define REG_SWITCH_1 (0x03)
#define REG_MEASURE  (0x04)
#define REG_SLICE    (0x05)
#define REG_CTRL_0   (0x06)
#define REG_CTRL_1   (0x07)
#define REG_MASK_0   (0x0A)
#define REG_POWER    (0x0B)
#define REG_SWRESET  (0x0C)
#define REG_STATUS_0 (0x40)
#define REG_STATUS_1 (0x41)
#define REG_INT      (0x42)
#define REG_FIFO     (0x43)

#define ID_FUSB300   (0x50)

#endif
