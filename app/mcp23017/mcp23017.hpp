#ifndef MCP23017_HPP
#define MCP23017_HPP

#include "i2c_device.hpp"
#include "mcp23017_config.hpp"
#include "mcp23017_registers.hpp"

namespace MCP23017 {

    struct MCP23017 {
    public:
        using I2CDevice = Utility::I2CDevice;

        MCP23017() noexcept = default;
        MCP23017(I2CDevice&& i2c_device, PortConfig const& port_a_config, PortConfig const& port_b_config) noexcept;

        MCP23017(MCP23017 const& other) = delete;
        MCP23017(MCP23017&& other) noexcept = default;

        MCP23017& operator=(MCP23017 const& other) = delete;
        MCP23017& operator=(MCP23017&& other) noexcept = default;

        ~MCP23017() noexcept;

        PinState get_pin_state(Port const port, PinNum const pin_num) const noexcept;
        void set_pin_state(Port const port, PinNum const pin_num, PinState const pin_state) const noexcept;

        bool get_pin(Port const port, PinNum const pin_num) const noexcept;

        void toggle_pin(Port const port, PinNum const pin_num) const noexcept;
        void toggle_pins(Port const port) const noexcept;

        void set_pin(Port const port, PinNum const pin_num) const noexcept;
        void set_pins(Port const port) const noexcept;

        void reset_pin(Port const port, PinNum const pin_num) const noexcept;
        void reset_pins(Port const port) const noexcept;

    private:
        std::uint8_t read_byte(std::uint8_t const reg_address) const noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> read_bytes(std::uint8_t const reg_address) const noexcept;

        void write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept;

        template <std::size_t SIZE>
        void write_bytes(std::uint8_t const reg_address, std::array<std::uint8_t, SIZE> const& bytes) const noexcept;

        void initialize(PortConfig const& port_a_config, PortConfig const& port_b_config) noexcept;
        void initialize_port(Port const port, PortConfig const& port_config) const noexcept;

        void deinitialize() noexcept;

        IODIR get_iodir_register(Port const port, Bank const bank) const noexcept;
        void set_iodir_register(Port const port, Bank const bank, IODIR const iodir) const noexcept;

        IPOL get_ipol_register(Port const port, Bank const bank) const noexcept;
        void set_ipol_register(Port const port, Bank const bank, IPOL const ipol) const noexcept;

        GPINTEN get_gpinten_register(Port const port, Bank const bank) const noexcept;
        void set_gpinten_register(Port const port, Bank const bank, GPINTEN const gpinten) const noexcept;

        DEFVAL get_defval_register(Port const port, Bank const bank) const noexcept;
        void set_defval_register(Port const port, Bank const bank, DEFVAL const defval) const noexcept;

        INTCON get_intcon_register(Port const port, Bank const bank) const noexcept;
        void set_intcon_register(Port const port, Bank const bank, INTCON const intcon) const noexcept;

        IOCON get_iocon_register(Port const port, Bank const bank) const noexcept;
        void set_iocon_register(Port const port, Bank const bank, IOCON const iocon) const noexcept;

        GPPU get_gppu_register(Port const port, Bank const bank) const noexcept;
        void set_gppu_register(Port const port, Bank const bank, GPPU const gppu) const noexcept;

        INTF get_intf_register(Port const port, Bank const bank) const noexcept;

        INTCAP get_intcap_register(Port const port, Bank const bank) const noexcept;

        GPIO get_gpio_register(Port const port, Bank const bank) const noexcept;
        void set_gpio_register(Port const port, Bank const bank, GPIO const gpio) const noexcept;

        bool initialized_{false};

        Bank bank_{};

        I2CDevice i2c_device_{};
    };

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> MCP23017::read_bytes(std::uint8_t const reg_address) const noexcept
    {
        return this->i2c_device_.read_bytes<SIZE>(reg_address);
    }

    template <std::size_t SIZE>
    inline void MCP23017::write_bytes(std::uint8_t const reg_address,
                                      std::array<std::uint8_t, SIZE> const& bytes) const noexcept
    {
        this->i2c_device_.write_bytes(reg_address, bytes);
    }

}; // namespace MCP23017

#endif // MCP23017_HPP