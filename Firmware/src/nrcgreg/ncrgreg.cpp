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
    return m_FuelTankAvg.getAvg();
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

uint32_t lastlog;

void NRCGreg::update()
{
    _value = m_GregStatus.getStatus();

    if (this->_state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED) && !m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT))
    {
        m_GregMachine.changeState(std::make_unique<Default>(m_DefaultStateParams)); // Return to defualt if the engine is disarmed
    }

    m_FuelTankAvg.update(m_FuelPT.getPressure());

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
    //Check if any sensors are below the disconnect threshold
    checkDisconnect(m_FuelPT.getPressure(), GREG_FLAGS::ERROR_FTP_LOCAL_DC, "Local fuel tank PT");
    checkDisconnect(m_PressTankPoller.getVal(), GREG_FLAGS::ERROR_N2P_REMOTE_DC, "Remote nitrogen PT");
    checkDisconnect(m_FuelTankPoller.getVal(), GREG_FLAGS::ERROR_FTP_REMOTE_NORESPONSE, "Remote fuel tank PT");
    checkDisconnect(m_OxTankPoller.getVal(), GREG_FLAGS::ERROR_OXP_REMOTE_NORESPONSE, "Remote ox tank PT");

    //Check if any sensors are above the overpressure 
    checkOverPressure(m_FuelPT.getPressure(), GREG_FLAGS::ERROR_FTP_LOCAL_OVP, "Local fuel tank PT");
    checkOverPressure(m_FuelTankPoller.getVal(), GREG_FLAGS::ERROR_FTP_REMOTE_OVP, "Remote fuel tank PT");
    checkOverPressure(m_OxTankPoller.getVal(), GREG_FLAGS::ERROR_OXP_REMOTE_OVP, "Remote ox tank PT");

    if(m_GregStatus.flagSetOr(GREG_FLAGS::ERROR_FTP_LOCAL_OVP,GREG_FLAGS::ERROR_FTP_REMOTE_OVP,GREG_FLAGS::ERROR_OXP_REMOTE_OVP) && !m_GregStatus.flagSet(GREG_FLAGS::ERROR_CRITICALOVP)){
        m_GregStatus.newFlag(GREG_FLAGS::ERROR_CRITICALOVP,"One or more pressures above the critical threshold!");
    }

    if (m_GregStatus.flagSet(GREG_FLAGS::ERROR_CRITICALOVP))
    {
        if (m_GregStatus.flagSet(GREG_FLAGS::STATE_DEFAULT))
        {
            return;
        }
        shutdown(); // Abort in the case of a critical overpressure event.
        return;
    }
}

void NRCGreg::updateRemoteP()
{
    if (m_GregStatus.flagSetOr(GREG_FLAGS::STATE_CONTROLLED, GREG_FLAGS::STATE_PRESSURISE)) // Only poll in the relevant states
    {
        checkNoResponse(m_PressTankPoller, GREG_FLAGS::ERROR_N2P_REMOTE_NORESPONSE, "nitrogen P");
        checkNoResponse(m_FuelTankPoller, GREG_FLAGS::ERROR_FTP_REMOTE_NORESPONSE, "fuel tank P");
        checkNoResponse(m_OxTankPoller, GREG_FLAGS::ERROR_OXP_REMOTE_NORESPONSE, "ox tank P");
    }
}

void NRCGreg::checkNoResponse(SensorPoller &poller_obj, GREG_FLAGS err_flag, std::string err_name)
{
    try
    {
        poller_obj.update();
        if (m_GregStatus.flagSet(err_flag))
        {
            m_GregStatus.deleteFlag(err_flag, std::string("Response received from ") + err_name + std::string(" service!"));
        }
    }
    catch (const std::exception &e)
    {
        if (!m_GregStatus.flagSet(err_flag))
        {
            m_GregStatus.newFlag(err_flag, std::string("No response received from ") + err_name + std::string(" service within the 1s timeout period!"));
        }
    }
}

void NRCGreg::checkDisconnect(float value, GREG_FLAGS err_flag, std::string err_name)
{
    if (value < m_P_disconnect)
    {
        if (!m_GregStatus.flagSet(err_flag))
        {
            m_GregStatus.newFlag(err_flag, err_name + std::string(" disconnected!"));
        }
    }

    else if (value > m_P_disconnect && m_GregStatus.flagSet(err_flag))
    {
        m_GregStatus.deleteFlag(err_flag, err_name + std::string(" reading back in expected range!"));
    }
}

void NRCGreg::checkOverPressure(float value, GREG_FLAGS err_flag, std::string err_name)
{
    if (value > m_P_full_abort)
    {
        if (!m_GregStatus.flagSet(err_flag))
        {
            m_GregStatus.newFlag(err_flag, err_name + std::string(" exceeded critical pressure!"));
        }
    }

    else if (value < m_P_full_abort && m_GregStatus.flagSet(err_flag))
    {
        m_GregStatus.deleteFlag(err_flag, err_name + std::string(" reading back below critical pressure!"));
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