#pragma once
#include <cstdint>
#include <memory>

#include <libriccore/systemstatus/systemstatus.h>
#include <libriccore/fsm/state.h>
#include <libriccore/fsm/statemachine.h>

/**
 * @brief Templated struct with type aliases inside to provide convient type access. Some of the template paramters might require
 * forward declaration to prevent cylic dependancies.
 *
 * @tparam GREG_FLAGS_T Enum of system flags
 */
enum class GREG_FLAGS : uint32_t
{
    // state flags
    STATE_DEFAULT = (1 << 0),
    STATE_PRESSURISE = (1 << 1),
    STATE_CONTROLLED = (1 << 2),
    STATE_SHUTDOWN = (1 << 3),
    STATE_HALFABORT = (1 << 4),
    // flags
    STATE_DEBUG = (1 << 5),
    // critical messages
    ERROR_CRITICALOVP = (1 << 6),   // Critical overpressure - abort triggered
    ERROR_FUELTANKP = (1 << 7),     // Abort triggered by fueltank error
    ERROR_OXTANKP_LOCAL = (1 << 8), // Abort triggered by local oxidiser tank error
    ERROR_OXTANKP_REMOTE = (1 << 9) // Abort triggered by remote oxidiser tank error
};

template <typename GREG_FLAGS_T>
struct GregTypes
{
    using SystemStatus_t = SystemStatus<GREG_FLAGS_T>;
    using State_t = State<GREG_FLAGS_T>;
    using State_ptr_t = std::unique_ptr<State_t>;
    using StateMachine_t = StateMachine<GREG_FLAGS_T>;
};

namespace Types
{
    using EREGTypes = GregTypes<GREG_FLAGS>;
    using Servo_t = NRCRemoteServo<LocalPWM>;
    using ServoAdapter_t = RemoteActuatorAdapter<Types::Servo_t>;
    using ServoADPMap_t = std::array<ServoAdapter_t *, 1>;
};

namespace Greg
{
    struct DefaultStateInit
    {
        Types::EREGTypes::SystemStatus_t &gregstatus;
        Types::ServoAdapter_t &regAdapter;
        const uint32_t regClosedAngle;
    };
    
    struct PressuriseParams
    {
        float OxTankP;
        uint32_t PressAngle;
        float P_Setpoint;
        float P_extra;
    };
}