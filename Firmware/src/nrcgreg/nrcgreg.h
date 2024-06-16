#pragma once

#include <librrc/Remote/nrcremoteactuatorbase.h>
#include <librrc/Remote/nrcremoteservo.h>

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>
#include <libriccore/platform/esp32/ADC.h>

class NRCGreg : public NRCRemoteActuatorBase<NRCGreg>
{

    public:

        NRCGreg(RnpNetworkManager &networkmanager,
                    uint8_t regServoGPIO,
                    uint8_t regServoChannel,
                    uint8_t overrideGPIO,
                    uint8_t pTankGPIO,
                    uint8_t address
                    ):
            NRCRemoteActuatorBase(networkmanager),
            _networkmanager(networkmanager),      
            _regServoGPIO(regServoGPIO),
            _regServoChannel(regServoChannel),
            _overrideGPIO(overrideGPIO),
            _pTankGPIO(pTankGPIO),
            _address(address),
            regServo(regServoGPIO,regServoChannel, networkmanager,0,0,180,10,140),
            adc(pTankGPIO)
            {};

        void setup();
        void update();
        void updateFuelTankP(float fueltankP);
        void updateHPtankP(float HPtankP);
        bool getPollingStatus() { return m_polling; };

        float get_lptankP();
        uint16_t closedLoopAngle();
        float feedforward();

        float getRegAngle() { return m_regAngleHiRes; };
        float getP() {return m_P_angle;};
        float getI() {return m_I_angle;};
        float getFF() {return feedforward();};
        uint8_t getStatus(){return static_cast<uint8_t>(currentEngineState);};

    protected:

        RnpNetworkManager &_networkmanager;
        const uint8_t _regServoGPIO;
        const uint8_t _regServoChannel;
        const uint8_t _overrideGPIO;
        const uint8_t _pTankGPIO;
        const uint8_t _address;

        NRCRemoteServo regServo;

        ADC adc;

        friend class NRCRemoteActuatorBase;
        friend class NRCRemoteBase;

        //-----------------------------------------------------------------------------------
        //---- State Machine Setup ----------------------------------------------------------
        //-----------------------------------------------------------------------------------

        //Define the codes for each engine state (use bit shifting to make it obvious when 2 are triggered at once)
        enum class EngineState : uint8_t
        {
            Default = 1<<0,
            Filling = 1<<1,
            ShutDown = 1<<2,
            Controlled = 1<<3,
            Openloop = 1<<4,
            Debug = 1<<5
        };

        //-----------------------------------------------------------------------------------
        //---- Function Definitions ---------------------------------------------------------
        //-----------------------------------------------------------------------------------

        //bool timeFrameCheck(int64_t start_time, int64_t end_time = -1);
        bool nominalRegOp();
        bool pValUpdated();
        
        void resetVars(){
            I_error = 0;
            Angle_integrator = 0;
            m_I_angle = 0;
            m_P_angle = 0;
            m_regAngleHiRes = regServo.getAngle();
        };

        float Q_interp();

        //Functions related to perfoming the state transitions
        void execute_impl(packetptr_t packetptr);
        void override_impl(packetptr_t packetptr);
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr);

        //-----------------------------------------------------------------------------------
        //---- Various Variable Definitions -------------------------------------------------
        //-----------------------------------------------------------------------------------

        //Variables used to hold the last time a network sensor measurement was updated
        uint64_t lastTimeFuelPUpdate;
        uint64_t lastTimeHPtankPUpdate;

        //Variables used to hold the current value of a network sensor measurement
        float m_HPtankP;
        float m_fueltankP;

        uint64_t startTime; //Variable used to store the system time of test start in ms
        uint16_t _ignitionCalls = 0;

        EngineState currentEngineState = EngineState::Default; //Set the starting engine state as Default/Disarmed

        bool shutdown_called = false; //WHAT IS THIS USED FOR?

        bool m_polling = false;

        //-----------------------------------------------------------------------------------
        //---- Component Calibration Constants ----------------------------------------------
        //-----------------------------------------------------------------------------------

        //GPIO Pressure transducer calibration constants
        const float P_gradient = 0.038; //CALIBRATED
        const float P_offset = -12.76; //CALIBRATED (should be -11.76 but reduced by 1 to match daq measurement)
        
        //Regulator valve calibration angles
        const uint16_t regClosedAngle = 0; //CALIBRATED
        const uint16_t regMinOpenAngle = 40; //CALIBRATED
        const uint16_t regMaxOpenAngle = 85; //CALIBRATED (Safeguard to prevent too high flow rates)
        const uint16_t regTankFillAngle = 51; //CALIBRATED

        //Fuel valve calibration angles
        const uint16_t fuelClosedAngle = 0; //CALIBRATED
        const uint16_t fuelMinOpenAngle = 85; //CALIBRATED
        const uint16_t fuelMaxOpenAngle = 170; //CALIBRATED

        //-----------------------------------------------------------------------------------
        //---- Test Parameters --------------------------------------------------------------
        //-----------------------------------------------------------------------------------
        // Reserved for things that will change between experiments

        const uint16_t fuelServoOpenAngle = 170; //angle to move the fuel servo valve to during test
        const uint32_t fuelServoAngle2Time = 100; //time when the valve moves to the 2nd angle
        const uint16_t fuelServoOpenAngle2 = 170; //2nd angle to move the fuel servo to
        const uint32_t fuelServoAngle3Time = 5000; //time when the valve moves to the 3rd angle
        const uint16_t fuelServoOpenAngle3 = 170; //3rd angle to move the fuel servo to
        const uint32_t testDuration = 8000; //time the valves are opened for in ms

        const uint16_t regServoOpenAngle = 46; //angle to open the reg valve to in the case of open loop control

        const float P_set = 37.5; //LP tank set pressure [bar]
        const float P_fill_add = 1.5; //Additional amount to add to the set pressure during filling to reach the target
        float Q_water = 0; //Expect volumetric flow rate of water [L/s]. Not necessarily constant

        //-----------------------------------------------------------------------------------
        //---- Controller Parameters --------------------------------------------------------
        //-----------------------------------------------------------------------------------
        
        //Abort Parameters
        const float highAbortP = 50; //Upper pressure redline [bar] to cause abort
        const float lowAbortP = -2; //Lower pressure redline [bar] to cause abort (to detect sensor disconnect)

        //PID Controller constants
        const float K_p_0 = 2; //Initial Proportional controller gain
        const float K_p_alpha = 0.015; //Gain increase proportional constant
        const float K_i = 1.5; //Integral controller gain
        const uint16_t I_lim = 8; //Limit for integral component to prevent integral windup

        //Feed forward calculation constants
        const uint32_t FF_Precede = 0; //Amount of time the e-reg opening precedes the fuel valve opening in ms
        const uint32_t K_1 = 372; //Constant that converts Q * pressure ratio across valve into the required Kv for the valve
        const uint16_t C_1 = regMinOpenAngle; //Equivalent to min. open angle of e-reg
        const uint32_t C_2 = 50; //Constant used to convert Kv to valve angle using linear function 171000 was found before

        //Variables used by controller
        uint64_t t_prev; //Used to save the time from the previous iteration (in milliseconds)
        float prev_error = 0; //Used to save the previous error to detect zero crossings
        float I_error = 0; //Used to save the current integral value to be added to or subtracted from
        float Angle_integrator = 0; //Used to count up the cumulative open angle over time (estimate ullage change)

        //Variables used for logging
        float m_I_angle = 0; //Used to keep track of the current integrator contribution to final output
        float m_P_angle = 0; //Used to keep track of the current proprotional contribution to final output
        float m_regAngleHiRes = 0; //Used to keep track of the reg angle as a float instead of uint16_t.

        //vectors to define throttle profile from ignition
        std::vector<float> m_FlowRate = {0.5,2};
        std::vector<uint32_t> m_testTime = {0,1000};
};