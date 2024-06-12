#include "nrcgreg.h"
#include <math.h>
#include <Arduino.h>

#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/services_config.h"

// Setup for the E-Reg Controller
void NRCGreg::setup()
{
    regServo.setup();

    adc.setup();

    regServo.setAngleLims(regClosedAngle, regMaxOpenAngle);

    pinMode(_overrideGPIO, INPUT_PULLUP); // May still keep manual override, not sure
}

void NRCGreg::update()
{
    // Close valves if component disarmed
    if (this->_state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::DISARMED))
    {
        currentEngineState = EngineState::Default;
    }

    /*
    if (digitalRead(_overrideGPIO) == 1)
    {
        currentEngineState = EngineState::ShutDown;
    }
    */

    // // Close valves after test duration
    // if ((millis() - startTime > testDuration) && _ignitionCalls > 0)
    // {
    //     currentEngineState = EngineState::ShutDown;
    //     _ignitionCalls = 0; // Flag used to ensure that system can exit the shutdown state
    // }

    adc.update();
    _value = static_cast<uint32_t>(currentEngineState);

    switch (currentEngineState)
    {

    // Default (turn on) state
    case EngineState::Default:
    {
        regServo.goto_Angle(regClosedAngle);
        m_polling = false;

        if (this->_state.flagSet(LIBRRC::COMPONENT_STATUS_FLAGS::NOMINAL))
        {
        }
        break;
    }

    // Tank pressurisation state
    case EngineState::Filling:

    {
        if (!nominalRegOp())
        {
            currentEngineState = EngineState::ShutDown;
            break;
        }
        // Open reg valve to filling angle. Hold open until _lptankP reaches P_set and then close
        regServo.goto_Angle(regTankFillAngle);

        if (get_lptankP() >= (P_set + P_fill_add))
        {
            regServo.goto_Angle(regClosedAngle);
            currentEngineState = EngineState::Default;
            // resetVars();
            break;
        }

        break;
    }

    // Closed loop control (nominal) state
    case EngineState::Controlled:
    {

        // Start closed loop control of regulator valve
        regServo.goto_AngleHighRes(closedLoopAngle());

        if (!nominalRegOp())
        {
            currentEngineState = EngineState::ShutDown;
            break;
        }

        break;
    }

    // System shutdown state (close both valves)
    case EngineState::ShutDown:
    {
        regServo.goto_Angle(regClosedAngle);
        m_I_angle = 0;
        m_P_angle = 0;
        m_regAngleHiRes = regClosedAngle;
        m_polling = false;

        break;
    }

    default:
    {
        break;
    }
    }
}

// Function to perform state machine transitions for the main operational system states
void NRCGreg::execute_impl(packetptr_t packetptr)
{
    SimpleCommandPacket execute_command(*packetptr);

    switch (execute_command.arg)
    {
    case 1:
    {
        if (currentEngineState != EngineState::Default)
        {
            break;
        }
        currentEngineState = EngineState::Controlled; // Change 'Controlled' to 'Openloop' if doing an open loop test
        startTime = millis();
        _ignitionCalls = 1;
        m_polling = true;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Test Start");
        break;
    }
    case 2:
    {
        currentEngineState = EngineState::ShutDown;
        _ignitionCalls = 0;
        resetVars();
        m_polling = false;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("ShutDown");
        break;
    }
    case 3:
    {
        if (currentEngineState != EngineState::Default)
        {
            break;
        }
        m_polling = false;
        currentEngineState = EngineState::Debug;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Entered debug");
        break;
    }
    case 4:
    {
        if (currentEngineState != EngineState::Default)
        {
            break;
        }
        m_polling = true;
        currentEngineState = EngineState::Filling;
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
        if (currentEngineState == EngineState::Debug)
        {
            regServo.goto_Angle(command_packet.arg);
        }
        else
        {
            break;
        }
    }
    default:
    {
        NRCRemoteActuatorBase::extendedCommandHandler_impl(commandID, std::move(packetptr));
        break;
    }
    }
}

// Function to apply the closed loop PI controller with feed forward. FIGURE OUT HOW PRESSURES ARE MEASURED AND WHICH FUNCTION TO DO THAT IN
float NRCGreg::closedLoopAngle()
{
    // float _HPtankP = _HPtankP; //Replace with actual measurement of the upstream tank pressure
    //  float P_HP_TANK = 200;

    float error = P_set - get_lptankP();   // Calculate error in tank pressure
    float dt = (millis() - t_prev) / 1000; // Calculate the time since the last
    t_prev = millis();

    if (error / prev_error < 0) // Detect for error zero crossings (error changing from +ve to -ve or vice versa)
    {
        I_error = 0; // Reset the integrator to zero
    }

    prev_error = error;

    I_error = I_error + error * dt; // Increment the integrator

    // Set upper and lower bounds to the integral term to prevent windup
    if (I_error > I_lim)
    {
        I_error = I_lim;
    }
    else if (I_error < -I_lim)
    {
        I_error = -I_lim;
    }

    float I_term = K_i * I_error;
    m_I_angle = I_term;

    float K_p = K_p_0 + K_p_alpha * Angle_integrator;
    m_P_angle = K_p * error;

    float reg_angle = (float)(K_p * error + I_term + feedforward());
    m_regAngleHiRes = reg_angle;

    // Set upper and lower bounds for the regualtor valve angle (figure out where these values should be defined from calibration)
    if (reg_angle > regMaxOpenAngle)
    {
        reg_angle = regMaxOpenAngle; // Prevent reg opening too much
    }
    else if (reg_angle < regMinOpenAngle)
    {
        reg_angle = regMinOpenAngle; // Prevent reg closing
    }

    Angle_integrator += (reg_angle - regMinOpenAngle) * dt; // Increment the angle integrator (proxy for ullage volume change over time)

    // Constrain the angle integrator and prevent it becoming negative (shouldn't be possible)
    if (Angle_integrator > 150)
    {
        Angle_integrator = 150;
    }
    else if (Angle_integrator < 0)
    {
        Angle_integrator = 0;
    }

    return reg_angle;
}

// Function to calculate the feedforward angle based on upstream pressure, set pressure and expected flowrate
float NRCGreg::feedforward()
{

    float FF_angle = ((C_2 * Q_water * P_set) / m_HPtankP) + C_1;
    if (FF_angle > 55)
    {
        FF_angle = 55;
    }
    else if (FF_angle < C_1)
    {
        FF_angle = C_1;
    }

    return FF_angle;
}

// Function to check if the reg system is running nominally or if an abort should be triggered
bool NRCGreg::nominalRegOp()
{
    if (get_lptankP() > lowAbortP && get_lptankP() < highAbortP)
    {
        return true;
    }

    if (m_fueltankP > lowAbortP && m_fueltankP < highAbortP)
    {
        return true;
    }

    return false;
}

void NRCGreg::updateHPtankP(float HPtankP)
{
    lastTimeHPtankPUpdate = millis();
    m_HPtankP = HPtankP;
}

void NRCGreg::updateFuelTankP(float fueltankP)
{
    lastTimeFuelPUpdate = millis();
    m_fueltankP = fueltankP;
}

// Get the pressure reading from the GPIO pin and convert to [barA] (Absolute pressure)
float NRCGreg::get_lptankP()
{
    float P = (P_gradient * (float)adc.getADC() + P_offset);
    return P;
}