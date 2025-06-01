#include "mcp23017.h"
#include "mcp23017_config.h"
#include "mcp23017_utility.h"
#include <assert.h>
#include <string.h>

static mcp23017_err_t mcp23017_bus_init(mcp23017_t const* mcp23017)
{
    assert(mcp23017);

    if (mcp23017->interface.bus_init) {
        return mcp23017->interface.bus_init(mcp23017->interface.bus_user);
    }

    return MCP23017_ERR_NULL;
}

static mcp23017_err_t mcp23017_bus_deinit(mcp23017_t const* mcp23017)
{
    assert(mcp23017);

    if (mcp23017->interface.bus_deinit) {
        return mcp23017->interface.bus_deinit(mcp23017->interface.bus_user);
    }

    return MCP23017_ERR_NULL;
}

static mcp23017_err_t
mcp23017_bus_write(mcp23017_t const* mcp23017, uint8_t write_address, uint8_t const* write_data, size_t write_size)
{
    assert(mcp23017 && write_data);

    if (mcp23017->interface.bus_write) {
        return mcp23017->interface.bus_write(mcp23017->interface.bus_user, write_address, write_data, write_size);
    }

    return MCP23017_ERR_NULL;
}

static mcp23017_err_t
mcp23017_bus_read(mcp23017_t const* mcp23017, uint8_t read_address, uint8_t* read_data, size_t read_size)
{
    assert(mcp23017 && read_data);

    if (mcp23017->interface.bus_read) {
        return mcp23017->interface.bus_read(mcp23017->interface.bus_user, read_address, read_data, read_size);
    }

    return MCP23017_ERR_NULL;
}

static mcp23017_err_t mcp23017_bus_port_write(mcp23017_t const* mcp23017,
                                              mcp23017_port_t port,
                                              uint8_t write_address,
                                              uint8_t const* write_data,
                                              size_t write_size)
{
    assert(mcp23017 && write_data);

    return mcp23017_bus_write(mcp23017,
                              mcp23017_port_bank_to_reg_address(port, mcp23017->config.bank, write_address),
                              write_data,
                              write_size);
}

static mcp23017_err_t mcp23017_bus_port_read(mcp23017_t const* mcp23017,
                                             mcp23017_port_t port,
                                             uint8_t read_address,
                                             uint8_t* read_data,
                                             size_t read_size)
{
    assert(mcp23017 && read_data);

    return mcp23017_bus_read(mcp23017,
                             mcp23017_port_bank_to_reg_address(port, mcp23017->config.bank, read_address),
                             read_data,
                             read_size);
}

mcp23017_err_t
mcp23017_initialize(mcp23017_t* mcp23017, mcp23017_config_t const* config, mcp23017_interface_t const* interface)
{
    assert(mcp23017 && config && interface);

    memset(mcp23017, 0, sizeof(*mcp23017));
    memcpy(&mcp23017->config, config, sizeof(*config));
    memcpy(&mcp23017->interface, interface, sizeof(*interface));

    return mcp23017_bus_init(mcp23017);
}

mcp23017_err_t mcp23017_deinitialize(mcp23017_t* mcp23017)
{
    assert(mcp23017);

    mcp23017_err_t err = mcp23017_bus_deinit(mcp23017);

    memset(mcp23017, 0, sizeof(*mcp23017));

    return err;
}

mcp23017_err_t mcp23017_get_pin_state(mcp23017_t const* mcp23017,
                                      mcp23017_port_t port,
                                      mcp23017_pin_num_t pin_num,
                                      mcp23017_pin_state_t* pin_state)
{
    assert(mcp23017 && pin_state);

    mcp23017_gpio_reg_t reg = {};

    mcp23017_err_t err = mcp23017_get_gpio_reg(mcp23017, port, &reg);

    *pin_state = (reg.gp & (1U << pin_num)) ? MCP23017_PIN_STATE_LOGIC_HIGH : MCP23017_PIN_STATE_LOGIC_LOW;

    return err;
}

mcp23017_err_t mcp23017_set_pin_state(mcp23017_t const* mcp23017,
                                      mcp23017_port_t port,
                                      mcp23017_pin_num_t pin_num,
                                      mcp23017_pin_state_t pin_state)
{
    assert(mcp23017);

    mcp23017_gpio_reg_t reg = {};

    mcp23017_err_t err = mcp23017_get_gpio_reg(mcp23017, port, &reg);

    reg.gp &= (uint8_t)~(1U << pin_num);
    reg.gp |= (uint8_t)(pin_state << pin_num);

    err |= mcp23017_set_gpio_reg(mcp23017, port, &reg);

    return err;
}

mcp23017_err_t mcp23017_toggle_pin_state(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_pin_num_t pin_num)
{
    assert(mcp23017);

    mcp23017_gpio_reg_t reg = {};

    mcp23017_err_t err = mcp23017_get_gpio_reg(mcp23017, port, &reg);

    reg.gp ^= (uint8_t)(1U << pin_num);

    err |= mcp23017_set_gpio_reg(mcp23017, port, &reg);

    return err;
}

mcp23017_err_t mcp23017_get_iodir_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_iodir_reg_t* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    mcp23017_err_t err = mcp23017_bus_port_read(mcp23017, port, MCP23017_REG_ADDRESS_IODIR, data, sizeof(data));

    reg->io7 = (data[0] >> 7U) & 0x01U;
    reg->io6 = (data[0] >> 6U) & 0x01U;
    reg->io5 = (data[0] >> 5U) & 0x01U;
    reg->io4 = (data[0] >> 4U) & 0x01U;
    reg->io3 = (data[0] >> 3U) & 0x01U;
    reg->io2 = (data[0] >> 2U) & 0x01U;
    reg->io1 = (data[0] >> 1U) & 0x01U;
    reg->io0 = (data[0] >> 0U) & 0x01U;

    return err;
}

mcp23017_err_t mcp23017_set_iodir_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_iodir_reg_t const* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    data[0] |= (reg->io7 & 0x01U) << 7U;
    data[0] |= (reg->io6 & 0x01U) << 6U;
    data[0] |= (reg->io5 & 0x01U) << 5U;
    data[0] |= (reg->io4 & 0x01U) << 4U;
    data[0] |= (reg->io3 & 0x01U) << 3U;
    data[0] |= (reg->io2 & 0x01U) << 2U;
    data[0] |= (reg->io1 & 0x01U) << 1U;
    data[0] |= (reg->io0 & 0x01U) << 0U;

    return mcp23017_bus_port_write(mcp23017, port, MCP23017_REG_ADDRESS_IODIR, data, sizeof(data));
}

mcp23017_err_t mcp23017_get_ipol_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_ipol_reg_t* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    mcp23017_err_t err = mcp23017_bus_port_read(mcp23017, port, MCP23017_REG_ADDRESS_IPOL, data, sizeof(data));

    reg->ip7 = (data[0] >> 7U) & 0x01U;
    reg->ip6 = (data[0] >> 6U) & 0x01U;
    reg->ip5 = (data[0] >> 5U) & 0x01U;
    reg->ip4 = (data[0] >> 4U) & 0x01U;
    reg->ip3 = (data[0] >> 3U) & 0x01U;
    reg->ip2 = (data[0] >> 2U) & 0x01U;
    reg->ip1 = (data[0] >> 1U) & 0x01U;
    reg->ip0 = (data[0] >> 0U) & 0x01U;

    return err;
}

mcp23017_err_t mcp23017_set_ipol_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_ipol_reg_t const* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    data[0] |= (reg->ip7 & 0x01U) << 7U;
    data[0] |= (reg->ip6 & 0x01U) << 6U;
    data[0] |= (reg->ip5 & 0x01U) << 5U;
    data[0] |= (reg->ip4 & 0x01U) << 4U;
    data[0] |= (reg->ip3 & 0x01U) << 3U;
    data[0] |= (reg->ip2 & 0x01U) << 2U;
    data[0] |= (reg->ip1 & 0x01U) << 1U;
    data[0] |= (reg->ip0 & 0x01U) << 0U;

    return mcp23017_bus_port_write(mcp23017, port, MCP23017_REG_ADDRESS_IPOL, data, sizeof(data));
}

mcp23017_err_t mcp23017_get_gpinten_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gpinten_reg_t* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    mcp23017_err_t err = mcp23017_bus_port_read(mcp23017, port, MCP23017_REG_ADDRESS_GPINTEN, data, sizeof(data));

    reg->gpint7 = (data[0] >> 7U) & 0x01U;
    reg->gpint6 = (data[0] >> 6U) & 0x01U;
    reg->gpint5 = (data[0] >> 5U) & 0x01U;
    reg->gpint4 = (data[0] >> 4U) & 0x01U;
    reg->gpint3 = (data[0] >> 3U) & 0x01U;
    reg->gpint2 = (data[0] >> 2U) & 0x01U;
    reg->gpint1 = (data[0] >> 1U) & 0x01U;
    reg->gpint0 = (data[0] >> 0U) & 0x01U;

    return err;
}

mcp23017_err_t
mcp23017_set_gpinten_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gpinten_reg_t const* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    data[0] |= (reg->gpint7 & 0x01U) << 7U;
    data[0] |= (reg->gpint6 & 0x01U) << 6U;
    data[0] |= (reg->gpint5 & 0x01U) << 5U;
    data[0] |= (reg->gpint4 & 0x01U) << 4U;
    data[0] |= (reg->gpint3 & 0x01U) << 3U;
    data[0] |= (reg->gpint2 & 0x01U) << 2U;
    data[0] |= (reg->gpint1 & 0x01U) << 1U;
    data[0] |= (reg->gpint0 & 0x01U) << 0U;

    return mcp23017_bus_port_write(mcp23017, port, MCP23017_REG_ADDRESS_GPINTEN, data, sizeof(data));
}

mcp23017_err_t mcp23017_get_defval_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_defval_reg_t* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    mcp23017_err_t err = mcp23017_bus_port_read(mcp23017, port, MCP23017_REG_ADDRESS_DEFVAL, data, sizeof(data));

    reg->def7 = (data[0] >> 7U) & 0x01U;
    reg->def6 = (data[0] >> 6U) & 0x01U;
    reg->def5 = (data[0] >> 5U) & 0x01U;
    reg->def4 = (data[0] >> 4U) & 0x01U;
    reg->def3 = (data[0] >> 3U) & 0x01U;
    reg->def2 = (data[0] >> 2U) & 0x01U;
    reg->def1 = (data[0] >> 1U) & 0x01U;
    reg->def0 = (data[0] >> 0U) & 0x01U;

    return err;
}

mcp23017_err_t
mcp23017_set_defval_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_defval_reg_t const* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    data[0] |= (reg->def7 & 0x01U) << 7U;
    data[0] |= (reg->def6 & 0x01U) << 6U;
    data[0] |= (reg->def5 & 0x01U) << 5U;
    data[0] |= (reg->def4 & 0x01U) << 4U;
    data[0] |= (reg->def3 & 0x01U) << 3U;
    data[0] |= (reg->def2 & 0x01U) << 2U;
    data[0] |= (reg->def1 & 0x01U) << 1U;
    data[0] |= (reg->def0 & 0x01U) << 0U;

    return mcp23017_bus_port_write(mcp23017, port, MCP23017_REG_ADDRESS_DEFVAL, data, sizeof(data));
}

mcp23017_err_t mcp23017_get_intcon_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_intcon_reg_t* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    mcp23017_err_t err = mcp23017_bus_port_read(mcp23017, port, MCP23017_REG_ADDRESS_INTCON, data, sizeof(data));

    reg->ioc7 = (data[0] >> 7U) & 0x01U;
    reg->ioc6 = (data[0] >> 6U) & 0x01U;
    reg->ioc5 = (data[0] >> 5U) & 0x01U;
    reg->ioc4 = (data[0] >> 4U) & 0x01U;
    reg->ioc3 = (data[0] >> 3U) & 0x01U;
    reg->ioc2 = (data[0] >> 2U) & 0x01U;
    reg->ioc1 = (data[0] >> 1U) & 0x01U;
    reg->ioc0 = (data[0] >> 0U) & 0x01U;

    return err;
}

mcp23017_err_t
mcp23017_set_intcon_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_intcon_reg_t const* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    data[0] |= (reg->ioc7 & 0x01U) << 7U;
    data[0] |= (reg->ioc6 & 0x01U) << 6U;
    data[0] |= (reg->ioc5 & 0x01U) << 5U;
    data[0] |= (reg->ioc4 & 0x01U) << 4U;
    data[0] |= (reg->ioc3 & 0x01U) << 3U;
    data[0] |= (reg->ioc2 & 0x01U) << 2U;
    data[0] |= (reg->ioc1 & 0x01U) << 1U;
    data[0] |= (reg->ioc0 & 0x01U) << 0U;

    return mcp23017_bus_port_write(mcp23017, port, MCP23017_REG_ADDRESS_INTCON, data, sizeof(data));
}

mcp23017_err_t mcp23017_get_iocon_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_iocon_reg_t* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    mcp23017_err_t err = mcp23017_bus_port_read(mcp23017, port, MCP23017_REG_ADDRESS_IOCON, data, sizeof(data));

    reg->bank = (data[0] >> 7U) & 0x01U;
    reg->mirror = (data[0] >> 6U) & 0x01U;
    reg->seqop = (data[0] >> 5U) & 0x01U;
    reg->disslw = (data[0] >> 4U) & 0x01U;
    reg->haen = (data[0] >> 3U) & 0x01U;
    reg->odr = (data[0] >> 2U) & 0x01U;
    reg->intpol = (data[0] >> 1U) & 0x01U;

    return err;
}

mcp23017_err_t mcp23017_set_iocon_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_iocon_reg_t const* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    data[0] |= (reg->bank & 0x01U) << 7U;
    data[0] |= (reg->mirror & 0x01U) << 6U;
    data[0] |= (reg->seqop & 0x01U) << 5U;
    data[0] |= (reg->disslw & 0x01U) << 4U;
    data[0] |= (reg->haen & 0x01U) << 3U;
    data[0] |= (reg->odr & 0x01U) << 2U;
    data[0] |= (reg->intpol & 0x01U) << 1U;

    return mcp23017_bus_port_write(mcp23017, port, MCP23017_REG_ADDRESS_IOCON, data, sizeof(data));
}

mcp23017_err_t mcp23017_get_gppu_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gppu_reg_t* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    mcp23017_err_t err = mcp23017_bus_port_read(mcp23017, port, MCP23017_REG_ADDRESS_GPPU, data, sizeof(data));

    reg->pu7 = (data[0] >> 7U) & 0x01U;
    reg->pu6 = (data[0] >> 6U) & 0x01U;
    reg->pu5 = (data[0] >> 5U) & 0x01U;
    reg->pu4 = (data[0] >> 4U) & 0x01U;
    reg->pu3 = (data[0] >> 3U) & 0x01U;
    reg->pu2 = (data[0] >> 2U) & 0x01U;
    reg->pu1 = (data[0] >> 1U) & 0x01U;
    reg->pu0 = (data[0] >> 0U) & 0x01U;

    return err;
}

mcp23017_err_t mcp23017_set_gppu_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gppu_reg_t const* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    data[0] |= (reg->pu7 & 0x01U) << 7U;
    data[0] |= (reg->pu6 & 0x01U) << 6U;
    data[0] |= (reg->pu5 & 0x01U) << 5U;
    data[0] |= (reg->pu4 & 0x01U) << 4U;
    data[0] |= (reg->pu3 & 0x01U) << 3U;
    data[0] |= (reg->pu2 & 0x01U) << 2U;
    data[0] |= (reg->pu1 & 0x01U) << 1U;
    data[0] |= (reg->pu0 & 0x01U) << 0U;

    return mcp23017_bus_port_write(mcp23017, port, MCP23017_REG_ADDRESS_GPPU, data, sizeof(data));
}

mcp23017_err_t mcp23017_get_intf_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_intf_reg_t* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    mcp23017_err_t err = mcp23017_bus_port_read(mcp23017, port, MCP23017_REG_ADDRESS_INTF, data, sizeof(data));

    reg->int7 = (data[0] >> 7U) & 0x01U;
    reg->int6 = (data[0] >> 6U) & 0x01U;
    reg->int5 = (data[0] >> 5U) & 0x01U;
    reg->int4 = (data[0] >> 4U) & 0x01U;
    reg->int3 = (data[0] >> 3U) & 0x01U;
    reg->int2 = (data[0] >> 2U) & 0x01U;
    reg->int1 = (data[0] >> 1U) & 0x01U;
    reg->int0 = (data[0] >> 0U) & 0x01U;

    return err;
}

mcp23017_err_t mcp23017_get_intcap_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_intcap_reg_t* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    mcp23017_err_t err = mcp23017_bus_port_read(mcp23017, port, MCP23017_REG_ADDRESS_INTCAP, data, sizeof(data));

    reg->icp7 = (data[0] >> 7U) & 0x01U;
    reg->icp6 = (data[0] >> 6U) & 0x01U;
    reg->icp5 = (data[0] >> 5U) & 0x01U;
    reg->icp4 = (data[0] >> 4U) & 0x01U;
    reg->icp3 = (data[0] >> 3U) & 0x01U;
    reg->icp2 = (data[0] >> 2U) & 0x01U;
    reg->icp1 = (data[0] >> 1U) & 0x01U;
    reg->icp0 = (data[0] >> 0U) & 0x01U;

    return err;
}

mcp23017_err_t mcp23017_get_gpio_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gpio_reg_t* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    mcp23017_err_t err = mcp23017_bus_port_read(mcp23017, port, MCP23017_REG_ADDRESS_GPIO, data, sizeof(data));

    reg->gp = data[0] & 0xFFU;

    return err;
}

mcp23017_err_t mcp23017_set_gpio_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gpio_reg_t const* reg)
{
    assert(mcp23017 && reg);

    uint8_t data[1] = {};

    data[0] = reg->gp & 0xFFU;

    return mcp23017_bus_port_write(mcp23017, port, MCP23017_REG_ADDRESS_GPIO, data, sizeof(data));
}
