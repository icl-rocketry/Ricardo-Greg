#include "nrcthanos.h"
#include <math.h>
#include <Arduino.h>

#include <libriccore/commands/commandhandler.h>
#include <libriccore/riccorelogging.h>

#include "Config/services_config.h"

// Setup for the E-Reg Controller
void NRCThanos::setup()
{
    fuelServo.setup();
    regServo.setup();

    adc.setup();

    fuelServo.setAngleLims(fuelClosedAngle, fuelMaxOpenAngle);
    regServo.setAngleLims(regClosedAngle, regMaxOpenAngle);

    pinMode(_overrideGPIO, INPUT_PULLUP); //May still keep manual override, not sure
    pinMode(_pTankGPIO, INPUT_PULLUP); //Figure out how this GPIO pin is defined and where

}

void NRCThanos::update()
{
    // Close valves if component disarmed
    if (this->_state.flagSet(COMPONENT_STATUS_FLAGS::DISARMED))
    {
        currentEngineState = EngineState::Default;
    }
    /*
    // Close valves if abort is used - CHECK IF KEEPING ABORT OR NOT
    if (digitalRead(_overrideGPIO) == 1)
    {
        currentEngineState = EngineState::ShutDown;
    }
    */

    // Close valves after test duration
    if ((millis() - startTime > testDuration) && _ignitionCalls > 0)
    {
        currentEngineState = EngineState::ShutDown;
        _ignitionCalls = 0; //Flag used to ensure that system can exit the shutdown state
    }

    adc.update();

    switch (currentEngineState)
    {

    // Default (turn on) state
    case EngineState::Default:
    {
        fuelServo.goto_Angle(fuelClosedAngle);
        regServo.goto_Angle(regClosedAngle);
        _polling = true; //what does this do?

        if (this->_state.flagSet(COMPONENT_STATUS_FLAGS::NOMINAL))
        {
            
        }
        break;
    }

    // Tank pressurisation state
    case EngineState::Filling:

    {   

        // Open reg valve to filling angle. Hold open until _lptankP reaches P_set and then close
        regServo.goto_Angle(regTankFillAngle);

        if (get_lptankP() >= (P_set + P_fill_add))
        {
            regServo.goto_Angle(regClosedAngle);
            currentEngineState = EngineState::Default;
            //resetVars();
            break;
        }


        break;
    }

    // Closed loop control (nominal) state
    case EngineState::Controlled:
    {

        //Open Fuel Valve to start the test
        if ((millis() - startTime > fuelServoAngle3Time)) {
            fuelServo.goto_Angle(fuelServoOpenAngle3);
        } else if ((millis() - startTime > fuelServoAngle2Time))
            {
                fuelServo.goto_Angle(fuelServoOpenAngle2);
            }
        else {
            fuelServo.goto_Angle(fuelServoOpenAngle);
        }

        //fuelServo.goto_Angle(fuelServoOpenAngle);

        //Start closed loop control of regulator valve
        regServo.goto_Angle(closedLoopAngle());

        if (((millis() - startTime > testDuration)) || !nominalRegOp())
        {
            currentEngineState = EngineState::ShutDown;
            // resetVars();
            break;
        }

        break;
    }

    // Open loop control state (only to be used for certain tests)
    case EngineState::Openloop:
    {

        //
        if (((millis() - startTime > testDuration)) || !nominalRegOp())
        {
            currentEngineState = EngineState::ShutDown;
            // resetVars();
            break;
        }

        //Open Fuel Valve tot start the test
        fuelServo.goto_Angle(fuelServoOpenAngle);

        if (((millis() - startTime > 150))) //Only open the reg valve after 100ms to prevent initial overshoot
        {
            regServo.goto_Angle(regServoOpenAngle);
            //break;
        }

        //Start open loop control of regulator valve after a short delay (move to set position)
        //regServo.goto_Angle(regServoOpenAngle);

        break;
    }

    // System shutdown state (close both valves)
    case EngineState::ShutDown:
    {
        regServo.goto_Angle(regClosedAngle);
        fuelServo.goto_Angle(fuelClosedAngle);
        _polling = false;

        break;
    }

    default:
    {
        break;
    }
    }
}

// Function to perform state machine transitions for the main operational system states
void NRCThanos::execute_impl(packetptr_t packetptr)
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
        currentEngineState = EngineState::Controlled; //Change 'Controlled' to 'Openloop' if doing an open loop test
        startTime = millis();
        _ignitionCalls = 1;
        _polling = true;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Test Start");
        break;
    }
    case 2:
    {
        currentEngineState = EngineState::ShutDown;
        _ignitionCalls = 0;
        resetVars();
        _polling = false;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("ShutDown");
        break;
    }
    case 3:
    {
        if (currentEngineState != EngineState::Default)
        {
            break;
        }
        _polling = false;
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
        _polling = false;
        currentEngineState = EngineState::Filling;
        RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Pressurisation Start");
        break;
    }
    }
}

// Extra states for use during system debugging
void NRCThanos::extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr)
{
    SimpleCommandPacket command_packet(*packetptr);
    switch (static_cast<uint8_t>(commandID))
    {
    case 6:
    {
        if (currentEngineState == EngineState::Debug)
        {
            fuelServo.goto_Angle(command_packet.arg);
            break;
        }
        else
        {
            break;
        }
    }
    case 7:
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
uint16_t NRCThanos::closedLoopAngle()
{
    //float _HPtankP = _HPtankP; //Replace with actual measurement of the upstream tank pressure
    // float P_HP_TANK = 200;

    float error = P_set - get_lptankP(); //Calculate error in tank pressure
    float dt = (millis() - t_prev)/1000; //Calculate the time since the last
    t_prev = millis();

    if (error/prev_error < 0) //Detect for error zero crossings (error changing from +ve to -ve or vice versa)
    {
        I_error = 0; //Reset the integrator to zero
    }

    prev_error = error;

    I_error = I_error + error*dt; //Increment the integrator

    //Set upper and lower bounds to the integral term to prevent windup
    if (I_error > I_lim)
    {
        I_error = I_lim;
    }
    else if (I_error < -I_lim)
    {
        I_error = -I_lim;
    }

    float I_term = K_i*I_error;

    float K_p = K_p_0 + K_p_alpha*Angle_integrator;

    uint16_t reg_angle = (int) (K_p*error + I_term + feedforward());

    //Set upper and lower bounds for the regualtor valve angle (figure out where these values should be defined from calibration)
    if (reg_angle > regMaxOpenAngle)
    {
        reg_angle = regMaxOpenAngle; //Prevent reg opening too much
    }
    else if (reg_angle < regMinOpenAngle)
    {
        reg_angle = regMinOpenAngle; //Prevent reg closing
    }

    Angle_integrator += (reg_angle-regMinOpenAngle)*dt; //Increment the angle integrator (proxy for ullage volume change over time)

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

//Function to calculate the feedforward angle based on upstream pressure, set pressure and expected flowrate
uint16_t NRCThanos::feedforward()
{
    //uint16_t FF_angle = C_2*((Q_water*(P_set/_HPtankP))/K_1) + C_1; //CHANGE TO PUT _HPtankP instead of 200
    //uint16_t FF_angle = 47;
    uint16_t FF_angle = ((C_2*Q_water*P_set)/_HPtankP) + C_1;
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
bool NRCThanos::nominalRegOp()
{
    if (get_lptankP() > lowAbortP && get_lptankP() < highAbortP)
    {
        return true;
    }
    else
    {
        return false; //Trigger an abort
    }
}

//Not sure what the point of this function is??
void NRCThanos::updateChamberP(float chamberP)
{
    lastTimeChamberPUpdate = millis();
    _chamberP = chamberP;
}

void NRCThanos::updateThrust(float thrust)
{
    lastTimeThrustUpdate = millis();
    _thrust = abs(thrust);
}

void NRCThanos::updateHPtankP(float HPtankP)
{
    lastTimeHPtankPUpdate = millis();
    _HPtankP = HPtankP;
}

// Get the pressure reading from the GPIO pin and convert to [barA] (Absolute pressure)
float NRCThanos::get_lptankP()
{
    float P = (P_gradient* (float) adc.getADC() + P_offset);
    return P;
}

/*

// Not sure what this is for, some network thing
bool NRCThanos::pValUpdated()
{
    if ((millis() - lastTimeChamberPUpdate) > pressureUpdateTimeLim || (millis() - lastTimeThrustUpdate) > pressureUpdateTimeLim)
    {
        return false;
    }
    else
    {
        return true;
    }
}

// Not sure this is needed
bool NRCThanos::timeFrameCheck(int64_t start_time, int64_t end_time)
{
    if (millis() - ignitionTime > start_time && end_time == -1)
    {
        return true;
    }

    else if (millis() - ignitionTime > start_time && millis() - ignitionTime < end_time)
    {
        return true;
    }

    else
    {
        return false;
    }
}


void NRCThanos::gotoWithSpeed(NRCRemoteServo &Servo, uint16_t demandAngle, float speed, float &prevAngle, float &currAngle, uint32_t &prevUpdateT)
{
    if (millis() - prevUpdateT < 10)
    {
        return;
    }

    if (prevUpdateT == 0)
    {
        prevUpdateT = millis();
        return;
    }

    float timeSinceLast = (float)(millis() - prevUpdateT) / 1000.0; // in seconds;

    if ((demandAngle - prevAngle) > 0)
    {
        currAngle = prevAngle + (timeSinceLast * speed);
    }
    else if ((demandAngle - prevAngle) < 0)
    {
        currAngle = prevAngle - (timeSinceLast * speed);
    }
    else
    {
        currAngle = currAngle;
    }

    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(currAngle));
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(_thrust));
    // Servo.goto_Angle(static_cast<uint16_t>(currAngle));
    Servo.goto_AngleHighRes(currAngle);
    prevAngle = currAngle;

    prevUpdateT = millis();
}


void NRCThanos::gotoThrust(float target, float closespeed, float openspeed)
{
    // if ((target * 1.02 > _thrust || target * 0.98 < _thrust) && !m_thrustreached)
    // {
    //     m_timeThrustreached = millis();
    //     m_thrustreached = true;
    // }

    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(target));
    // RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(std::to_string(_thrust));

    if (target * 1.02 < _thrust)
    {
        gotoWithSpeed(oxServo, 70, closespeed, m_oxServoPrevAngle, m_oxServoCurrAngle, m_oxServoPrevUpdate);
    }
    else if (_thrust < target * 0.98)
    {
        gotoWithSpeed(oxServo, 180, openspeed, m_oxServoPrevAngle, m_oxServoCurrAngle, m_oxServoPrevUpdate);
    }
    else
    {
        oxServo.goto_Angle(m_oxServoCurrAngle);
    }

    m_oxPercent = (float)(m_oxServoCurrAngle - oxServoPreAngle) / (float)(m_oxThrottleRange);
    m_fuelPercent = m_oxPercent + m_fuelExtra;
    float fuelAngle = (float)(m_fuelPercent * m_fuelThrottleRange) + fuelServoPreAngle;

    if (fuelAngle < fuelServoPreAngle)
    {
        fuelServo.goto_AngleHighRes(fuelServoPreAngle);
    }
    else
    {
        fuelServo.goto_AngleHighRes(fuelAngle);
    }
}

void NRCThanos::firePyro(uint32_t duration)
{
    if (millis() - _prevFiring > _ignitionCommandSendDelta)
    {
        if (_ignitionCalls < _ignitionCommandMaxCalls)
        {
            SimpleCommandPacket ignition_command(2, duration);
            ignition_command.header.source_service = static_cast<uint8_t>(Services::ID::Thanos);
            ignition_command.header.destination_service = m_ingitionService;
            ignition_command.header.source = _address;
            ignition_command.header.destination = m_ignitionNode;
            ignition_command.header.uid = 0;
            _networkmanager.sendPacket(ignition_command);
            _prevFiring = millis();
            _ignitionCalls++;
        }
    }
}

*/


