#ifndef MCP23017_CONFIG_HPP
#define MCP23017_CONFIG_HPP

#include <cstdint>

namespace MCP23017 {

    enum struct DevAddress : std::uint16_t {
        A2H_A1H_A0H = 0b0100111,
        A2H_A1H_A0L = 0b0100110,
        A2H_A1L_A0H = 0b0100101,
        A2L_A1H_A0H = 0b0100011,
        A2H_A1L_A0L = 0b0100100,
        A2L_A1H_A0L = 0b0100010,
        A2L_A1L_A0H = 0b0100001,
        A2L_A1L_A0L = 0b0100000,
    };

    enum struct RA : std::uint8_t {
        IODIR = 0x00,
        IPOL = 0x01,
        GPINTEN = 0x02,
        DEFVAL = 0x03,
        INTCON = 0x04,
        IOCON = 0x05,
        GPPU = 0x06,
        INTF = 0x07,
        INTCAP = 0x08,
        GPIO = 0x09,
        OLAT = 0x0A,
    };

    enum struct Port : std::uint8_t {
        PORT_A = 0x00,
        PORT_B = 0x01,
    };

    enum struct PinNum : std::uint8_t {
        IO_7 = 0x07,
        IO_6 = 0x06,
        IO_5 = 0x05,
        IO_4 = 0x04,
        IO_3 = 0x03,
        IO_2 = 0x02,
        IO_1 = 0x01,
        IO_0 = 0x00,
    };

    enum struct Direction : std::uint8_t {
        INPUT = 0x01,
        OUTPUT = 0x00,
    };

    enum struct PinPolarity : std::uint8_t {
        INVERSE = 0x01,
        NORMAL = 0x00,
    };

    enum struct Bank : std::uint8_t {
        SEPARATE = 0x01,
        COMMON = 0x00,
    };

    enum struct Mirror : std::uint8_t {
        INT_PINS_CONNECTED = 0x01,
        INT_PINS_DISCONNECTED = 0x00,
    };

    enum struct SequentialOp : std::uint8_t {
        DISABLED = 0x01,
        ENABLED = 0x00,
    };

    enum struct SlewRate : std::uint8_t {
        DISABLED = 0x01,
        ENABLED = 0x00,
    };

    enum struct HAEnable : std::uint8_t {
        ENABLE = 0x01,
        DISABLE = 0x00,
    };

    enum struct OutputDrive : std::uint8_t {
        OPEN_DRAIN = 0x01,
        PUSH_PULL = 0x00,
    };

    enum struct PinState : std::uint8_t {
        LOGIC_HIGH = 0x01,
        LOGIC_LOW = 0x00,
    };

    enum struct IntEnable : std::uint8_t {
        ENABLED = 0x01,
        DISABLED = 0x00,
    };

    enum struct IntMode : std::uint8_t {
        FALLING = 0x01,
        RISING = 0x00,
    };

    enum struct IntControl : std::uint8_t {
        COMPARE_DEFAULT = 0x01,
        COMPARE_PREV = 0x00,
    };

    enum struct IntCapture : std::uint8_t {
        HIGH_ACTIVE = 0x01,
        LOW_ACTIVE = 0x00,
    };

    enum struct InputDrive : std::uint8_t {
        PULL_UP = 0x01,
        PULL_DOWN = 0x00,
    };

    enum struct IntFlag : std::uint8_t {
        OCCURED = 0x01,
        PENDING = 0x00,
    };

    inline std::uint8_t pin_num_to_mask(PinNum const pin_num) noexcept
    {
        return 1U << std::to_underlying(pin_num);
    }

    inline std::uint8_t separate_bank_port_to_reg_address(Port const port, RA const reg_address) noexcept
    {
        switch (port) {
            case Port::PORT_A:
                return std::to_underlying(reg_address);
            case Port::PORT_B:
                return std::to_underlying(reg_address) + 10U;
            default:
                return 0U;
        }
    }

    inline std::uint8_t common_bank_port_to_reg_address(Port const port, RA const reg_address) noexcept
    {
        switch (port) {
            case Port::PORT_A:
                return std::to_underlying(reg_address);
            case Port::PORT_B:
                return std::to_underlying(reg_address) + 1U;
            default:
                return 0U;
        }
    }

    inline std::uint8_t port_bank_to_reg_address(Port const port, Bank const bank, RA const reg_address) noexcept
    {
        switch (bank) {
            case Bank::SEPARATE:
                return separate_bank_port_to_reg_address(port, reg_address);
            case Bank::COMMON:
                return common_bank_port_to_reg_address(port, reg_address);
            default:
                return 0U;
        }
    }

}; // namespace MCP23017

#endif // MCP23017_CONFIG_HPP