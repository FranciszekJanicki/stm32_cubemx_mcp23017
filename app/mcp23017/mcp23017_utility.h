#ifndef MCP23017_MCP23017_UTILITY_H
#define MCP23017_MCP23017_UTILITY_H

#include "mcp23017_config.h"

inline uint8_t mcp23017_separate_bank_port_to_address(mcp23017_port_t port,
                                                      mcp23017_reg_address_t address)
{
    switch (port) {
        case MCP23017_PORT_A:
            return address;
        case MCP23017_PORT_B:
            return address + 10U;
        default:
            return 0U;
    }
}

inline uint8_t mcp23017_common_bank_port_to_address(mcp23017_port_t port,
                                                    mcp23017_reg_address_t address)
{
    switch (port) {
        case MCP23017_PORT_A:
            return address;
        case MCP23017_PORT_B:
            return address + 1U;
        default:
            return 0U;
    }
}

inline uint8_t mcp23017_port_to_address(mcp23017_port_t port,
                                        mcp23017_bank_t bank,
                                        mcp23017_reg_address_t address)
{
    switch (bank) {
        case MCP23017_BANK_SEPARATE:
            return mcp23017_separate_bank_port_to_address(port, address);
        case MCP23017_BANK_COMMON:
            return mcp23017_common_bank_port_to_address(port, address);
        default:
            return 0U;
    }
}

#endif // MCP23017_MCP23017_UTILITY_H