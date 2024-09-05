#include "nrcgreg.h"
#include <math.h>
#include <Arduino.h>

#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/services_config.h"
#include "default.h"
#include "pressurise.h"
#include "shutdown.h"
#include "controlled.h"

// Setup for the E-Reg Controller
void NRCGreg::setup()
{
    m_regServo.setup();
    m_GregMachine.initalize(std::make_unique<Default>(m_DefaultStateParams));
}

float NRCGreg::feedforward()
{
    float FF = m_FF_0 + m_FF_Alpha / m_PressTankPoller.getVal();
    return std::max(std::min(FF, m_FF_max), m_FF_min); // Set bounds on FF angle before returning.
}

float NRCGreg::Kp()
{
    float Kp = m_Kp_0 + m_Kp_Beta / m_PressTankPoller.getVal();

    return std::max(std::min(Kp, m_Kp_max), m_Kp_min); // Set bounds on Kp before returning.
}

uint32_t NRCGreg::nextAngle()
{

    float error = m_P_setpoint - getOxTankP(); // Calculate error in tank pressure

    m_P_angle = (float)Kp() * (float)error;

    uint32_t reg_angle = static_cast<uint32_t>(m_P_angle + feedforward());

    return std::max(std::min(reg_angle, m_regMaxOpenAngle), m_regMinOpenAngle); // Set bounds on angle during operation.
}

void NRCGreg::update()
{   
    _value = m_GregStatus.getStatus();
    m_GregMachine.update();

    if (this->_state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED) && !m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT))
    {
        m_GregMachine.changeState(std::make_unique<Default>(m_DefaultStateParams)); // Return to defualt if the engine is disarmed
    }

    if (m_GregStatus.flagSetOr(GREG_FLAGS::STATE_CONTROLLED, GREG_FLAGS::STATE_PRESSURISE)) //Only poll in the relevant states
    {
        try{m_PressTankPoller.update();}
        catch (const std::exception &e)
        {
            
            return;
        }

        try{m_FuelTankPoller.update();}
        catch (const std::exception &e)
        {
            return;
        }
    }
}

void NRCGreg::execute_impl(packetptr_t packetptr)
{
    SimpleCommandPacket execute_command(*packetptr);

    switch (execute_command.arg)
    {
    case 1: // Controlled command
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT)) // Can only go to the controlled state from default.
        {
            break;
        }
        m_GregMachine.changeState(std::make_unique<Controlled>(m_DefaultStateParams, *this)); // Can always shut down
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Test Start");
        break;
    }
    case 2: // Shutdown command
    {
        m_GregMachine.changeState(std::make_unique<Shutdown>(m_DefaultStateParams)); // Can always shut down
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("ShutDown");
        break;
    }
    case 3: // Debug command
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT)) // Can only pressurise from default.
        {
            break;
        }
        // DEBUG COMMAND
        //  currentEngineState = EngineState::Debug;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered debug");
        break;
    }
    case 4: // Pressurise command
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT)) // Can only pressurise from default.
        {
            break;
        }

        m_GregMachine.changeState(std::make_unique<Pressurise>(m_DefaultStateParams, *this));
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Pressurisation Start");
        break;
    }
    }
}

// Extra states for use during system debugging
void NRCGreg::extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr)
{
    SimpleCommandPacket command_packet(*packetptr);
    switch (static_cast<uint8_t>(commandID))
    {
    case 6:
    {
        // if (currentEngineState == EngineState::Debug)
        // {
        //     m_regAdapter.goto_Angle(command_packet.arg);
        //     m_regAngleHiRes = regServo.getAngle();
        // }
        // else
        // {
        //     break;
        // }
    }
    default:
    {
        NRCRemoteActuatorBase::extendedCommandHandler_impl(commandID, std::move(packetptr));
        break;
    }
    }
}