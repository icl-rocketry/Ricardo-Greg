#include "system.h"

#include <memory>

#include <libriccore/riccoresystem.h>

#include <HardwareSerial.h>

#include "Config/systemflags_config.h"
#include "Config/commands_config.h"
#include "Config/pinmap_config.h"
#include "Config/general_config.h"
#include "Config/services_config.h"

#include "Commands/commands.h"

#include "States/idle.h"

#include <librrc/Interface/rocketcomponent.h>

//I'm guessing this is where most things in the overall system are connected together using existing definitions from other header files
System::System():
RicCoreSystem(Commands::command_map,Commands::defaultEnabledCommands,Serial),
Buck(PinMap::BuckPGOOD, PinMap::BuckEN, 1, 1, PinMap::BuckOutputV, 1500, 470),
canbus(systemstatus,PinMap::TxCan,PinMap::RxCan,3),
fuelTankPTap(1, GeneralConfig::Kermitaddr, static_cast<uint8_t>(Services::ID::fuelTankPTap), static_cast<uint8_t>(Services::ID::fuelTankPTap), networkmanager, [](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
HPtankPTap(2, GeneralConfig::Kermitaddr, static_cast<uint8_t>(Services::ID::HPtankPTap), static_cast<uint8_t>(Services::ID::HPtankPTap), networkmanager, [](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
fuelTankPoller(50, &fuelTankPTap),
HPTankPTapPoller(50, &HPtankPTap),

//This is used in nrcthanos.h to define the pinouts using pinmap_config.h
Thanos(networkmanager,PinMap::ServoPWM1,0,PinMap::EngineOverride,PinMap::LPTankP,networkmanager.getAddress())
{};


void System::systemSetup(){
    
    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);
   
    //intialize rnp message logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(networkmanager);

    //initialize statemachine with idle state
    statemachine.initalize(std::make_unique<Idle>(systemstatus,commandhandler));
    
    //any other setup goes here
    
    Buck.setup();
    fuelTankPoller.setup();
    HPTankPTapPoller.setup(); //CHECK THIS IS CORRECT
    Thanos.setup();
    canbus.setup();
    networkmanager.addInterface(&canbus);

    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});

    //Defining these so the methods following are less ugly
    uint8_t thanosservice = static_cast<uint8_t>(Services::ID::Thanos);
    uint8_t fuelTankPTapservice = static_cast<uint8_t>(Services::ID::fuelTankPTap);
    uint8_t HPtankPTapservice = static_cast<uint8_t>(Services::ID::HPtankPTap);

    networkmanager.registerService(thanosservice,Thanos.getThisNetworkCallback());
    networkmanager.registerService(fuelTankPTapservice,[this](packetptr_t packetptr){fuelTankPTap.networkCallback(std::move(packetptr));});
    networkmanager.registerService(HPtankPTapservice,[this](packetptr_t packetptr){HPtankPTap.networkCallback(std::move(packetptr));});
};

void System::systemUpdate(){
    Buck.update();

    if(Thanos.getPollingStatus()){  
        fuelTankPoller.update();
        HPTankPTapPoller.update();
    }
    
    if(fuelTankPoller.newdata)
    {
        Thanos.updateFuelTankP(fuelTankPoller.getVal());
    }

    if(HPTankPTapPoller.newdata)
    {
        Thanos.updateHPtankP(HPTankPTapPoller.getVal());
    }

    Thanos.update();
};