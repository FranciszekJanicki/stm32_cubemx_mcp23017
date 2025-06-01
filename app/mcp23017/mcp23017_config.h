#ifndef MCP23017_MCP23017_CONFIG_H
#define MCP23017_MCP23017_CONFIG_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    MCP23017_ERR_OK = 0,
    MCP23017_ERR_FAIL = 1 << 0,
    MCP23017_ERR_NULL = 1 << 1,
} mcp23017_err_t;

typedef enum {
    MCP23017_DEV_ADDRESS_A2H_A1H_A0H = 0b0100111,
    MCP23017_DEV_ADDRESS_A2H_A1H_A0L = 0b0100110,
    MCP23017_DEV_ADDRESS_A2H_A1L_A0H = 0b0100101,
    MCP23017_DEV_ADDRESS_A2L_A1H_A0H = 0b0100011,
    MCP23017_DEV_ADDRESS_A2H_A1L_A0L = 0b0100100,
    MCP23017_DEV_ADDRESS_A2L_A1H_A0L = 0b0100010,
    MCP23017_DEV_ADDRESS_A2L_A1L_A0H = 0b0100001,
    MCP23017_DEV_ADDRESS_A2L_A1L_A0L = 0b0100000,
} mcp23017_dev_address_t;

typedef enum {
    MCP23017_REG_ADDRESS_IODIR = 0x00,
    MCP23017_REG_ADDRESS_IPOL = 0x01,
    MCP23017_REG_ADDRESS_GPINTEN = 0x02,
    MCP23017_REG_ADDRESS_DEFVAL = 0x03,
    MCP23017_REG_ADDRESS_INTCON = 0x04,
    MCP23017_REG_ADDRESS_IOCON = 0x05,
    MCP23017_REG_ADDRESS_GPPU = 0x06,
    MCP23017_REG_ADDRESS_INTF = 0x07,
    MCP23017_REG_ADDRESS_INTCAP = 0x08,
    MCP23017_REG_ADDRESS_GPIO = 0x09,
    MCP23017_REG_ADDRESS_OLAT = 0x0A,
} mcp23017_reg_address_t;

typedef enum {
    MCP23017_PORT_A = 0x00,
    MCP23017_PORT_B = 0x01,
} mcp23017_port_t;

typedef enum {
    MCP23017_PIN_NUM_IO_7 = 0x07,
    MCP23017_PIN_NUM_IO_6 = 0x06,
    MCP23017_PIN_NUM_IO_5 = 0x05,
    MCP23017_PIN_NUM_IO_4 = 0x04,
    MCP23017_PIN_NUM_IO_3 = 0x03,
    MCP23017_PIN_NUM_IO_2 = 0x02,
    MCP23017_PIN_NUM_IO_1 = 0x01,
    MCP23017_PIN_NUM_IO_0 = 0x00,
} mcp23017_pin_num_t;

typedef enum {
    MCP23017_PIN_DIRECTION_INPUT = 0x01,
    MCP23017_PIN_DIRECTION_OUTPUT = 0x00,
} mcp23017_pin_direction_t;

typedef enum {
    MCP23017_PIN_POLARITY_INVERSE = 0x01,
    MCP23017_PIN_POLARITY_NORMAL = 0x00,
} mcp23017_pin_polarity_t;

typedef enum {
    MCP23017_BANK_SEPARATE = 0x01,
    MCP23017_BANK_COMMON = 0x00,
} mcp23017_bank_t;

typedef enum {
    MCP23017_MIRROR_INT_PINS_CONNECTED = 0x01,
    MCP23017_MIRROR_INT_PINS_DISCONNECTED = 0x00,
} mcp23017_mirror_t;

typedef enum {
    MCP23017_SEQUENTIAL_OP_DISABLED = 0x01,
    MCP23017_SEQUENTIAL_OP_ENABLED = 0x00,
} mcp23017_sequential_op_t;

typedef enum {
    MCP23017_SLEW_RATE_DISABLED = 0x01,
    MCP23017_SLEW_RATE_ENABLED = 0x00,
} mcp23017_slew_rate_t;

typedef enum {
    MCP23017_HARDWARE_ADDRES_EN_ENABLE = 0x01,
    MCP23017_HARDWARE_ADDRES_EN_DISABLE = 0x00,
} mcp23017_harware_address_en_t;

typedef enum {
    MCP23017_OUTPUT_DRIVE_OPEN_DRAIN = 0x01,
    MCP23017_OUTPUT_DRIVE_PUSH_PULL = 0x00,
} mcp23017_output_drive_t;

typedef enum {
    MCP23017_PIN_STATE_LOGIC_HIGH = 0x01,
    MCP23017_PIN_STATE_LOGIC_LOW = 0x00,
} mcp23017_pin_state_t;

typedef enum {
    MCP23017_INT_ENABLE_ENABLED = 0x01,
    MCP23017_INT_ENABLE_DISABLED = 0x00,
} mcp23017_int_enable_t;

typedef enum {
    MCP23017_INT_MODE_FALLING = 0x01,
    MCP23017_INT_MODE_RISING = 0x00,
} mcp23017_int_mode_t;

typedef enum {
    MCP23017_INT_CONTROL_COMPARE_DEFAULT = 0x01,
    MCP23017_INT_CONTROL_COMPARE_PREV = 0x00,
} mcp23017_int_control_t;

typedef enum {
    MCP23017_INT_CAPTURE_HIGH_ACTIVE = 0x01,
    MCP23017_INT_CAPTURE_LOW_ACTIVE = 0x00,
} mcp23017_int_capture_t;

typedef enum {
    MCP23017_INPUT_DRIVE_PULL_UP = 0x01,
    MCP23017_INPUT_DRIVE_PULL_DOWN = 0x00,
} mcp23017_input_drive_t;

typedef enum {
    MCP23017_INT_FLAG_OCCURED = 0x01,
    MCP23017_INT_FLAG_PENDING = 0x00,
} mcp23017_int_flag_t;

typedef struct {
    mcp23017_bank_t bank;
} mcp23017_config_t;

typedef struct {
    void* bus_user;
    mcp23017_err_t (*bus_init)(void*);
    mcp23017_err_t (*bus_deinit)(void*);
    mcp23017_err_t (*bus_write)(void*, uint8_t, uint8_t const*, size_t);
    mcp23017_err_t (*bus_read)(void*, uint8_t, uint8_t*, size_t);
} mcp23017_interface_t;

#endif // MCP23017_MCP23017_CONFIG_H