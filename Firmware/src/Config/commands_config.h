#pragma once

#include <stdint.h>
#include <unordered_map>
#include <functional>
#include <initializer_list>

#include <libriccore/commands/commandhandler.h>
#include <librnp/rnp_packet.h>

#include "Config/forward_decl.h"
#include "Commands/commands.h"


namespace Commands
{
    enum class ID : uint8_t
    {
        NoCommand = 0,
        Telemetry = 8,
        BuckRestart = 44,
        Free_Ram = 250
    };

    inline std::initializer_list<ID> defaultEnabledCommands = {ID::Free_Ram, ID::Telemetry};

    inline std::unordered_map<ID, std::function<void(ForwardDecl_SystemClass &, const RnpPacketSerialized &)>> command_map{
        {ID::Free_Ram, FreeRamCommand}, {ID::Telemetry, GregTelemCommand}, {ID::BuckRestart, BuckRestartCommand}};

};