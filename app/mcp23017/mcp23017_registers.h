#ifndef MCP23017_MCP23017_REGISTERS_H
#define MCP23017_MCP23017_REGISTERS_H

#include "mcp23017_config.h"
#include <stdint.h>

typedef union {
    uint8_t io7 : 1;
    uint8_t io6 : 1;
    uint8_t io5 : 1;
    uint8_t io4 : 1;
    uint8_t io3 : 1;
    uint8_t io2 : 1;
    uint8_t io1 : 1;
    uint8_t io0 : 1;
} mcp23017_iodir_reg_t;

typedef struct {
    uint8_t ip7 : 1;
    uint8_t ip6 : 1;
    uint8_t ip5 : 1;
    uint8_t ip4 : 1;
    uint8_t ip3 : 1;
    uint8_t ip2 : 1;
    uint8_t ip1 : 1;
    uint8_t ip0 : 1;
} mcp23017_ipol_reg_t;

typedef struct {
    uint8_t gpint7 : 1;
    uint8_t gpint6 : 1;
    uint8_t gpint5 : 1;
    uint8_t gpint4 : 1;
    uint8_t gpint3 : 1;
    uint8_t gpint2 : 1;
    uint8_t gpint1 : 1;
    uint8_t gpint0 : 1;
} mcp23017_gpinten_reg_t;

typedef struct {
    uint8_t def7 : 1;
    uint8_t def6 : 1;
    uint8_t def5 : 1;
    uint8_t def4 : 1;
    uint8_t def3 : 1;
    uint8_t def2 : 1;
    uint8_t def1 : 1;
    uint8_t def0 : 1;
} mcp23017_defval_reg_t;

typedef struct {
    uint8_t ioc7 : 1;
    uint8_t ioc6 : 1;
    uint8_t ioc5 : 1;
    uint8_t ioc4 : 1;
    uint8_t ioc3 : 1;
    uint8_t ioc2 : 1;
    uint8_t ioc1 : 1;
    uint8_t ioc0 : 1;
} mcp23017_intcon_reg_t;

typedef struct {
    uint8_t bank : 1;
    uint8_t mirror : 1;
    uint8_t seqop : 1;
    uint8_t disslw : 1;
    uint8_t haen : 1;
    uint8_t odr : 1;
    uint8_t intpol : 1;
} mcp23017_iocon_reg_t;

typedef struct {
    uint8_t pu7 : 1;
    uint8_t pu6 : 1;
    uint8_t pu5 : 1;
    uint8_t pu4 : 1;
    uint8_t pu3 : 1;
    uint8_t pu2 : 1;
    uint8_t pu1 : 1;
    uint8_t pu0 : 1;
} mcp23017_gppu_reg_t;

typedef struct {
    uint8_t int7 : 1;
    uint8_t int6 : 1;
    uint8_t int5 : 1;
    uint8_t int4 : 1;
    uint8_t int3 : 1;
    uint8_t int2 : 1;
    uint8_t int1 : 1;
    uint8_t int0 : 1;
} mcp23017_intf_reg_t;

typedef struct {
    uint8_t icp7 : 1;
    uint8_t icp6 : 1;
    uint8_t icp5 : 1;
    uint8_t icp4 : 1;
    uint8_t icp3 : 1;
    uint8_t icp2 : 1;
    uint8_t icp1 : 1;
    uint8_t icp0 : 1;
} mcp23017_intcap_reg_t;

typedef struct {
    uint8_t gp7 : 1;
    uint8_t gp6 : 1;
    uint8_t gp5 : 1;
    uint8_t gp4 : 1;
    uint8_t gp3 : 1;
    uint8_t gp2 : 1;
    uint8_t gp1 : 1;
    uint8_t gp0 : 1;
} mcp23017_gpio_reg_t;

#endif // MCP23017_MCP23017_REGISTERS_H