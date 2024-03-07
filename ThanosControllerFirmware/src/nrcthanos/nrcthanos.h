#pragma once

#include "librrc/nrcremoteactuatorbase.h"
#include "librrc/nrcremoteservo.h"

#include <librnp/rnp_networkmanager.h>
#include <librnp/rnp_packet.h>

#include <SiC43x.h>


class NRCThanos : public NRCRemoteActuatorBase<NRCThanos>
{

    public:

        NRCThanos(RnpNetworkManager &networkmanager,
                    uint8_t fuelServoGPIO,
                    uint8_t fuelServoChannel,
                    uint8_t oxServoGPIO,
                    uint8_t oxServoChannel,
                    uint8_t overrideGPIO,
                    uint8_t tvc0,
                    uint8_t tvc1,
                    uint8_t tvc2,
                    uint8_t address,
                    SiC43x& Buck
                    ):
            NRCRemoteActuatorBase(networkmanager),
            _networkmanager(networkmanager),      
            _fuelServoGPIO(fuelServoGPIO),
            _fuelServoChannel(fuelServoChannel),
            _regServoGPIO(regServoGPIO),
            _regServoChannel(regServoChannel),
            _overrideGPIO(overrideGPIO),
            _address(address),
            fuelServo(fuelServoGPIO,fuelServoChannel,networkmanager,0,0,180,0,175),
            regServo(regServoGPIO,regServoChannel, networkmanager,0,0,180,10,140),
            _Buck(Buck)
            {};

        void setup();
        void update();
        void updateThrust(float thrust);
        void updateChamberP(float chamberP);
        bool getPollingStatus() { return _polling; };

        uint16_t getFuelAngle() { return fuelServo.getAngle(); };
        uint16_t getRegAngle() { return regServo.getAngle(); };
        uint8_t getStatus(){return static_cast<uint8_t>(currentEngineState);};

    protected:
        RnpNetworkManager &_networkmanager;
        const uint8_t _fuelServoGPIO;
        const uint8_t _fuelServoChannel;
        const uint8_t _regServoGPIO;
        const uint8_t _regServoChannel;
        const uint8_t _overrideGPIO;

        const uint8_t _address;



        NRCRemoteServo fuelServo;
        NRCRemoteServo regServo;

        SiC43x& _Buck;

        friend class NRCRemoteActuatorBase;
        friend class NRCRemoteBase;

        void execute_impl(packetptr_t packetptr);
        // void arm_impl(packetptr_t packetptr);
        // void disarm_impl(packetptr_t packetptr);
        void override_impl(packetptr_t packetptr);
        void extendedCommandHandler_impl(const NRCPacket::NRC_COMMAND_ID commandID, packetptr_t packetptr);

        // enum class EngineState : uint8_t
        // {
        //     Default = 0,
        //     Ignition = 1,
        //     ShutDown = 2,
        //     NominalT = 3,
        //     ThrottledT = 4,
        //     // Fullbore = 4,
        //     Debug = 5
        // };

        enum class EngineState : uint8_t
        {
            Default = 1<<0,
            Ignition = 1<<1,
            ShutDown = 1<<2,
            NominalT = 1<<3,
            ThrottledT = 1<<4,
            Calibration = 1 << 5,
            // Fullbore = 4,
            Debug = 1<<6
        };


        bool fullbore_called = false;
        bool shutdown_called = false;

        EngineState currentEngineState = EngineState::Default;

        uint64_t ignitionTime;

        //Also include value for _lptankP, taken directly from GPIO pin
        //float _chamberP; //Chamber pressure value (replace with _hptankP)
        float _hptankP;
        float _lptankP;
        float _thrust;

        bool timeFrameCheck(int64_t start_time, int64_t end_time = -1);
        bool nominalEngineOp();
        bool pValUpdated();

        void gotoWithSpeed(NRCRemoteServo &Servo, uint16_t demandAngle, float speed, float &prevAngle, float &currAngle, uint32_t &prevUpdateT);

        void gotoThrust(float target, float closespeed, float openspeed);
        void firePyro(uint32_t duration);

        void resetVars(){
            m_fuelServoPrevUpdate = 0;
            m_oxServoPrevUpdate = 0;
            m_fuelServoPrevAngle = fuelServo.getValue();
            m_oxServoPrevAngle = oxServo.getValue();
            m_thrustreached = false;
        };

        // Ignition sequence timings from moment ignition command received
        const uint64_t pyroFires = 0;
        const uint64_t endOfIgnitionSeq = 500;

        const uint16_t fuelServoPreAngle = 105;
        const uint16_t regServoPreAngle = 70;

        uint64_t lastTimeThrustUpdate;
        uint64_t lastTimeChamberPUpdate;

        const uint64_t pressureUpdateTimeLim = 1000;
        const uint32_t m_firstNominalTime = 4900;
        const uint32_t m_throttledDownTime = 4400;
        const uint32_t m_secondNominalTime = 3000;
        const uint32_t m_cutoffTime = 14000;
        const uint32_t m_calibrationTime = 65000;
        const uint32_t m_oxDelay = 100;
        const uint32_t m_oxFillCloseTime = 13000;
        const uint32_t m_edgingDelay = 100;

        uint8_t _ignitionCalls = 0;
        const uint8_t _ignitionCommandMaxCalls = 2;
        const uint8_t _ignitionCommandSendDelta = 50;
        uint32_t _prevFiring = 0;

        bool _polling = false;

        bool oxFillClosed = false;
        uint8_t closeOxFillCalls = 0;

        //
        uint8_t m_ingitionService = 12;
        uint8_t m_ignitionNode = 107;

        const uint8_t m_oxFillService = 10;
        const uint8_t m_oxFillNode = 103;

        //-----------------------------------------------------------------------------------
        //---- Controller Parameters --------------------------------------------------------
        //-----------------------------------------------------------------------------------

        //PID Controller constants
        const float K_p = 1.5; //Proportional controller gain
        const float K_i = 3; //Integral controller gain
        const float I_lim = 20 //Limit for integral component to prevent integral windup

        //Feed forward calculation constants
        const float FF_Precede = 0.2; //Amount of time the e-reg opening precedes the fuel valve opening
        const float K_1 = 372.1; //Constant that converts Q * pressure ratio across valve into the required Kv for the valve
        const float C_1 = 20; //Equivalent to min. open angle of e-reg
        const float C_2 = 171000; //Constant used to convert Kv to valve angle using linear function
        const float Q_water = 1; //Expect volumetric flow rate of water in L/s. Not necessarily constant

        //Variables used by controller
        uint32_t t_prev; //Used to save the time from the previous iteration (in milliseconds)
        float I_error = 0; //Used to save the current integral value to be added to or subtracted from


        uint32_t m_calibrationStart = 0;

        //
        bool m_thrustreached = false;
        uint32_t m_throttledEntry;
        uint32_t m_nominalEntry;
        bool m_firstNominal = false;
        bool m_calibrationDone = false;

        //
        float m_oxPercent = 0;
        float m_fuelPercent = 0;
        uint16_t m_oxThrottleRange = 0;
        uint16_t m_fuelThrottleRange = 0;

        float m_fuelExtra = -0.1;

};