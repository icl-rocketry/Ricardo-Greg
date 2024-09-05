#include "Controlled.h"

#include <memory>

#include <libriccore/fsm/state.h>
#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/systemflags_config.h"
#include "Config/types.h"
#include <librrc/Local/remoteactuatoradapter.h>

#include "system.h"


Controlled::Controlled(Greg::DefaultStateInit& DefaultInitParams, NRCGreg& Greg):
State(GREG_FLAGS::STATE_CONTROLLED,DefaultInitParams.gregstatus),
m_regAdapter(DefaultInitParams.regAdapter),
m_Greg(Greg)
{};

void Controlled::initialize()
{
    Types::EREGTypes::State_t::initialize(); // call parent initialize first!
    m_regAdapter.arm(0);
};

Types::EREGTypes::State_ptr_t Controlled::update()
{

    m_regAdapter.execute(static_cast<uint32_t>(m_Greg.nextAngle()));

    return nullptr;
};

void Controlled::exit()
{
    Types::EREGTypes::State_t::exit(); // call parent exit last!
};