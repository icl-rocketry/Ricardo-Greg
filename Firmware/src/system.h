#pragma once

#include <libriccore/riccoresystem.h>
#include <libriccore/networkinterfaces/can/canbus.h>
#include <libriccore/platform/esp32/ADC.h>
#include <librrc/Interface/networksensor.h>
#include <librrc/Helpers/sensorpoller.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"


#include "SiC43x.h"
#include "nrcgreg/nrcgreg.h"

#include "Commands/commands.h"


class System : public RicCoreSystem<System,SYSTEM_FLAG,Commands::ID>
{
    public:

        System();
        
        void systemSetup();

        void systemUpdate();

        SiC43x Buck;

        CanBus<SYSTEM_FLAG> canbus;

        NetworkSensor FuelTankPTap;
        NetworkSensor OxTankPTap;
        NetworkSensor HPtankPTap;
        
        SensorPoller FuelTankPoller;
        SensorPoller OxTankPoller;
        SensorPoller HPTankPTapPoller;

        NRCRemotePTap m_FuelPTLocal;
        NRCGreg Greg;

    private:

        ADC m_FuelPTLocalADC;
        

        

        

};