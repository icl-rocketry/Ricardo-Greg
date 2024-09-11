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
    STATE_DEBUG = (1 << 5),

    // critical messages
    ERROR_CRITICALOVP = (1 << 6),   // Critical overpressure - abort triggered
    // Local fuel tank codes
    ERROR_FUELTANKP_LOCAL = (1 << 7),     // Abort triggered by some local fuel tank error
    ERROR_FTP_LOCAL_DC = (1 << 8),        // Trigerred by disconnect
    ERROR_FTP_LOCAL_NORESPONSE = (1 << 9),// No response
    ERROR_FTP_LOCAL_OVP = (1 << 10),      // Overpressure
    // Remote fuel tank codes
    ERROR_FUELTANKP_REMOTE = (1 << 11),    // Abort triggered by some remote fuel tank error
    ERROR_FTP_REMOTE_DC = (1 << 12),        // Disconnect
    ERROR_FTP_REMOTE_NORESPONSE = (1 << 13),// No response
    ERROR_FTP_REMOTE_OVP = (1 << 14),       // Overpressure
    // Ox tank error codes
    ERROR_OXTANKP_REMOTE = (1 << 15),      // Abort triggered by some ox tank error
    ERROR_OXP_REMOTE_DC = (1 << 16),        // Disconnect
    ERROR_OXP_REMOTE_NORESPONSE = (1 << 17),// No response
    ERROR_OXP_REMOTE_OVP = (1 << 18),       // Overpressure
    // Nitrogen tank error code
    ERROR_N2P_REMOTE = (1 << 19),          // Abort triggered by some NITROGEN tank error
    ERROR_N2P_REMOTE_DC = (1 << 20),        // Disconnect
    ERROR_N2P_REMOTE_NORESPONSE = (1 << 21),// No response
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