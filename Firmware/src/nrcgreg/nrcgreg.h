#pragma once
/**
 * @file nrcgreg.h
 * @author Martin England
 * @author Andrei Paduraru (ap2621@ic.ac.uk)
 * @brief The greg class is responsible for all control related to the E-Reg.
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <librrc/Remote/nrcremoteactuatorbase.h>
#include <librrc/Remote/nrcremoteservo.h>
#include <librrc/Remote/nrcremoteptap.h>
#include <librrc/Helpers/sensorpoller.h>

#include <librrc/HAL/localpwm.h>

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <libriccore/fsm/statemachine.h>
#include "gregtypes.h"
    
class NRCGreg : public NRCRemoteActuatorBase<NRCGreg>
{

    public:

        NRCGreg(RnpNetworkManager &networkmanager,
                    uint8_t regServoGPIO,
                    uint8_t regServoChannel,
                    NRCRemotePTap& OxTankPT,
                    SensorPoller& NitrogenPPoller,
                    SensorPoller& FuelTankPPoller
                    ):
            NRCRemoteActuatorBase(networkmanager),
            m_networkmanager(networkmanager),      
            m_reg_PWM(regServoGPIO,regServoChannel),
            m_regServo(m_reg_PWM,networkmanager,"Srvo0"),
            m_regAdapter(0,m_regServo,[](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
            m_OxPT(OxTankPT),
            m_PressTankPoller(NitrogenPPoller),
            m_FuelTankPoller(FuelTankPPoller)
            {};

        void setup();
        void update();

        //Controller functions

        /**
        * @brief Function that calculates the feedforward angle based on input pressure.
        * @return feedforward angle as a float. */
        float feedforward();

        /**
        * @brief Function that calculates the proportional gain based on input pressure.
        * @return KP as a float */
        float Kp();

        /**
        * @brief Function that calculates the next angle the controller should move to.
        * @return Next controlled angle as an integer. */
        uint32_t nextAngle();

        //Getters
        uint32_t getRegClosedAngle(){return m_regClosedAngle;};
        float getOxTankP(){return m_OxPT.getPressure();};
        uint32_t getRegAngle(){return m_regServo.getValue();};
        float getPAngle(){return m_P_angle;};

        Greg::PressuriseParams getPressuriseParams(){
            Greg::PressuriseParams Params = {getOxTankP(), 
            m_regPressuriseAngle, 
            m_P_setpoint, 
            m_P_press_extra};
            return Params;
        }

        float getHalfAbortP(){return m_P_half_abort;};
        float getFullAbortP(){return m_P_full_abort;};

    protected:

        //Networking
        RnpNetworkManager &m_networkmanager;
        friend class NRCRemoteActuatorBase;
        friend class NRCRemoteBase;

        //NRC components
        //Actuators
        LocalPWM m_reg_PWM;
        Types::Servo_t m_regServo;
        Types::ServoAdapter_t m_regAdapter;

        //Sensors
        //Connected locally
        NRCRemotePTap& m_OxPT;

        //Network sensor
        SensorPoller& m_PressTankPoller;
        SensorPoller& m_FuelTankPoller;

        void execute_impl(packetptr_t packetptr);
        void override_impl(packetptr_t packetptr);
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr);
        
        // FSM related stuff
        Types::EREGTypes::StateMachine_t m_GregMachine;
        Types::EREGTypes::SystemStatus_t m_GregStatus;

        Greg::DefaultStateInit m_DefaultStateParams = {m_GregStatus, m_regAdapter, m_regClosedAngle};

        // ---------- Controller Parameters ----------
        // FF Params
        float m_FF_min = 55.0;
        float m_FF_max = 80.0;
        float m_FF_0 = 34.0;
        float m_FF_Alpha = 5570.0;

        // KP calculation Params
        float m_Kp_min = 2.0;
        float m_Kp_max = 3.0;
        float m_Kp_0 = 1.143;
        float m_Kp_Beta = 222.9;

        // Controller setpoints
        float m_P_setpoint = 35; //Running pressure setpoint.
        float m_P_press_extra = 1.5; //Extra pressure to add during pressurisation to make sure setpoint is reached.

        // Operating pressure limits
        float m_P_half_abort = 50; //Above this value, a half abort will be triggered.
        float m_P_full_abort = 60; //Above thi svalue, a full abort will be triggered.

        //        --- HARDWARE LIMITS ---
        const uint32_t m_regClosedAngle = 0;
        const uint32_t m_regMaxOpenAngle = 85;
        const uint32_t m_regMinOpenAngle = 40;
        uint32_t m_regPressuriseAngle = 49;

        //Variables to log out
        float m_P_angle;

};