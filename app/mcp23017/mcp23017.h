#ifndef MCP23017_MCP23017_H
#define MCP23017_MCP23017_H

#include "mcp23017_config.h"
#include "mcp23017_registers.h"

typedef struct {
    mcp23017_config_t config;
    mcp23017_interface_t interface;
} mcp23017_t;

mcp23017_err_t
mcp23017_initialize(mcp23017_t* mcp23017, mcp23017_config_t const* config, mcp23017_interface_t const* interface);
mcp23017_err_t mcp23017_deinitialize(mcp23017_t* mcp23017);

mcp23017_err_t mcp23017_get_pin_state(mcp23017_t const* mcp23017,
                                      mcp23017_port_t port,
                                      mcp23017_pin_num_t pin_num,
                                      mcp23017_pin_state_t* pin_state);
mcp23017_err_t mcp23017_set_pin_state(mcp23017_t const* mcp23017,
                                      mcp23017_port_t port,
                                      mcp23017_pin_num_t pin_num,
                                      mcp23017_pin_state_t pin_state);
mcp23017_err_t mcp23017_toggle_pin_state(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_pin_num_t pin_num);

mcp23017_err_t mcp23017_get_iodir_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_iodir_reg_t* reg);
mcp23017_err_t
mcp23017_set_iodir_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_iodir_reg_t const* reg);

mcp23017_err_t mcp23017_get_ipol_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_ipol_reg_t* reg);
mcp23017_err_t mcp23017_set_ipol_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_ipol_reg_t const* reg);

mcp23017_err_t mcp23017_get_gpinten_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gpinten_reg_t* reg);
mcp23017_err_t
mcp23017_set_gpinten_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gpinten_reg_t const* reg);

mcp23017_err_t mcp23017_get_defval_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_defval_reg_t* reg);
mcp23017_err_t
mcp23017_set_defval_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_defval_reg_t const* reg);

mcp23017_err_t mcp23017_get_intcon_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_intcon_reg_t* reg);
mcp23017_err_t
mcp23017_set_intcon_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_intcon_reg_t const* reg);

mcp23017_err_t mcp23017_get_iocon_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_iocon_reg_t* reg);
mcp23017_err_t
mcp23017_set_iocon_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_iocon_reg_t const* reg);

mcp23017_err_t mcp23017_get_gppu_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gppu_reg_t* reg);
mcp23017_err_t mcp23017_set_gppu_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gppu_reg_t const* reg);

mcp23017_err_t mcp23017_get_intf_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_intf_reg_t* reg);

mcp23017_err_t mcp23017_get_intcap_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_intcap_reg_t* reg);

mcp23017_err_t mcp23017_get_gpio_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gpio_reg_t* reg);
mcp23017_err_t mcp23017_set_gpio_reg(mcp23017_t const* mcp23017, mcp23017_port_t port, mcp23017_gpio_reg_t const* reg);

#endif // MCP23017_MCP23017_H