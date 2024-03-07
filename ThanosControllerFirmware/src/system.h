#pragma once

#include <libriccore/riccoresystem.h>
#include <libriccore/networkinterfaces/can/canbus.h>
#include <librrc/networksensor.h>
#include "librrc/sensorpoller.h"

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"


#include "SiC43x.h"
#include "nrcthanos/nrcthanos.h"

#include "Commands/commands.h"


class System : public RicCoreSystem<System,SYSTEM_FLAG,Commands::ID>
{
    public:

        System();
        
        void systemSetup();

        void systemUpdate();

        SiC43x Buck;

        CanBus<SYSTEM_FLAG> canbus;

        NetworkSensor chamberPTap;
        NetworkSensor thrustGauge;
        NetworkSensor HPtankPTap;
        SensorPoller chamberPTapPoller;
        SensorPoller thrustGaugePoller;
        SensorPoller HPtankPTapPoller;

        NRCThanos Thanos;

    // private:

        

        

};