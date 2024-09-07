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
#include "debug.h"

// Setup for the E-Reg Controller
void NRCGreg::setup()
{
    m_regServo.setup();
    m_regServo.setAngleLims(0, 850);
    m_GregMachine.initalize(std::make_unique<Default>(m_DefaultStateParams));
}

float NRCGreg::getFuelTankP()
{
    if (m_GregStatus.flagSet(GREG_FLAGS::ERROR_FUELTANKP_LOCAL))
    {
        if (m_P_source == 0)
        {
            RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Controller pressure source switched to remote fuel ptap!");
            m_P_source = 1;
        }
        return m_FuelTankPoller.getVal();
    }

    if (m_P_source == 1)
    {
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Controller pressure source returned to local fuel ptap!");
    }
    return m_FuelPT.getPressure();
}

float NRCGreg::feedforward()
{
    float FF = m_FF_0 + m_FF_Alpha / m_HPN;
    return std::max(std::min(FF, m_FF_max), m_FF_min); // Set bounds on FF angle before returning.
}

float NRCGreg::Kp()
{
    float Kp = m_Kp_0 + m_Kp_Beta / m_HPN;

    return std::max(std::min(Kp, m_Kp_max), m_Kp_min); // Set bounds on Kp before returning.
}

uint32_t NRCGreg::nextAngle()
{

    float error = m_P_setpoint - getFuelTankP(); // Calculate error in tank pressure

    m_P_angle = (float)Kp() * (float)error;

    uint32_t reg_angle = static_cast<uint32_t>((m_P_angle + feedforward()) * 10.0f);

    return std::max(std::min(reg_angle, m_regMaxOpenAngle), m_regMinOpenAngle); // Set bounds on angle during operation.
}

void NRCGreg::update()
{
    _value = m_GregStatus.getStatus();

    if (this->_state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED) && !m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT))
    {
        m_GregMachine.changeState(std::make_unique<Default>(m_DefaultStateParams)); // Return to defualt if the engine is disarmed
    }

    m_GregMachine.update();

    updateRemoteP();

    checkPressures();
}

void NRCGreg::shutdown()
{
    m_GregMachine.changeState(std::make_unique<Shutdown>(m_DefaultStateParams));
}

void NRCGreg::checkPressures()
{
    if (m_FuelPT.getPressure() < m_P_disconnect) //
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_FUELTANKP_LOCAL))
        {
            m_GregStatus.newFlag(GREG_FLAGS::ERROR_FUELTANKP_LOCAL, "Local fuel tank PT disconnected!");
        }
    }

    if (m_FuelPT.getPressure() > m_P_full_abort)
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_CRITICALOVP))
        {
            m_GregStatus.newFlag(GREG_FLAGS::ERROR_CRITICALOVP, "Local fuel tank PT exceeded maximum pressure!");
        }
        if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_FUELTANKP_LOCAL))
        {
            m_GregStatus.newFlag(GREG_FLAGS::ERROR_FUELTANKP_LOCAL, "Local fuel tank PT exceeded maximum pressure!");
        }
    }

    checkRemoteP();

    if (m_GregStatus.flagSet(GREG_FLAGS::ERROR_CRITICALOVP))
    {
        shutdown(); // Abort in the case of a critical overpressure event.
        return;
    }
}

void NRCGreg::updateRemoteP()
{

    if (m_GregStatus.flagSetOr(GREG_FLAGS::STATE_CONTROLLED, GREG_FLAGS::STATE_PRESSURISE)) // Only poll in the relevant states
    {
        try
        {
            m_PressTankPoller.update();
        }
        catch (const std::exception &e)
        {
            if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_NITROGENTANKP))
            {
                m_GregStatus.newFlag(GREG_FLAGS::ERROR_NITROGENTANKP, "No response received from nitrogen PT service within the 1s timeout period!");
            }
        }

        try
        {
            m_FuelTankPoller.update();
        }
        catch (const std::exception &e)
        {
            if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_FUELTANKP_REMOTE))
            {
                m_GregStatus.newFlag(GREG_FLAGS::ERROR_FUELTANKP_REMOTE, "No response received from remote fuel tank PT service within the 1s timeout period!");
            }
        }

        try
        {
            m_OxTankPoller.update();
        }
        catch (const std::exception &e)
        {
            if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_OXTANKP))
            {
                m_GregStatus.newFlag(GREG_FLAGS::ERROR_OXTANKP, "No response received from oxidiser tank PT service within the 1s timeout period!");
            }
        }
    }
}

void NRCGreg::checkRemoteP()
{
    if (m_FuelTankPoller.getVal() < m_P_disconnect)
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_FUELTANKP_REMOTE))
        {
            m_GregStatus.newFlag(GREG_FLAGS::ERROR_FUELTANKP_REMOTE, "Remote fuel tank PT disconnected!");
        }
    }

    if (m_OxTankPoller.getVal() < m_P_disconnect)
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_OXTANKP))
        {
            m_GregStatus.newFlag(GREG_FLAGS::ERROR_OXTANKP, "Remote fuel tank PT disconnected!");
        }
    }

    if (m_FuelTankPoller.getVal() > m_P_full_abort)
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_CRITICALOVP))
        {
            m_GregStatus.newFlag(GREG_FLAGS::ERROR_CRITICALOVP, "Remote fuel tank PT exceeded maximum pressure!");
        }
        if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_FUELTANKP_REMOTE))
        {
            m_GregStatus.newFlag(GREG_FLAGS::ERROR_FUELTANKP_REMOTE, "Remote fuel tank PT exceeded maximum pressure!");
        }
    }

    if (m_OxTankPoller.getVal() > m_P_full_abort)
    {
        if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_CRITICALOVP))
        {
            m_GregStatus.newFlag(GREG_FLAGS::ERROR_CRITICALOVP, "Remote ox tank PT exceeded maximum pressure!");
        }
        if (!m_GregStatus.flagSet(GREG_FLAGS::ERROR_OXTANKP))
        {
            m_GregStatus.newFlag(GREG_FLAGS::ERROR_OXTANKP, "Remote ox tank PT exceeded maximum pressure!");
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
        if (!m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT)) // Can only debug from default.
        {
            break;
        }
        // DEBUG COMMAND
        m_GregMachine.changeState(std::make_unique<Debug>(m_DefaultStateParams));
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
        if (m_GregStatus.flagSet(GREG_FLAGS::STATE_DEBUG))
        {
            m_regAdapter.execute(command_packet.arg);
        }
        else
        {
            break;
        }
    }
    case 7: // command to set pressurise angle
    {
        if (command_packet.arg > (m_regPressuriseAngle + 50))
        {
            break;
        }
        else
        {
            m_regPressuriseAngle = command_packet.arg;
        }
        break;
    }

    default:
    {
        NRCRemoteActuatorBase::extendedCommandHandler_impl(commandID, std::move(packetptr));
        break;
    }
    }
}