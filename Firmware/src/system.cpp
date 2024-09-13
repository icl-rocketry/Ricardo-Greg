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
FuelTankPTap(0, GeneralConfig::KermitAddr, static_cast<uint8_t>(Services::ID::FuelTankPTRemote), static_cast<uint8_t>(Services::ID::FuelTankPTRemote), networkmanager, [](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
OxTankPTap(1, GeneralConfig::KermitAddr, static_cast<uint8_t>(Services::ID::OxTankPT), static_cast<uint8_t>(Services::ID::OxTankPT), networkmanager, [](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
HPtankPTap(2, GeneralConfig::KermitAddr, static_cast<uint8_t>(Services::ID::HPTankPT), static_cast<uint8_t>(Services::ID::HPTankPT), networkmanager, [](const std::string& msg){RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>(msg);}),
FuelTankPoller(50, &FuelTankPTap),
OxTankPoller(50, &OxTankPTap),
HPTankPTapPoller(50, &HPtankPTap),
m_FuelPTLocal(networkmanager,0),
Greg(networkmanager,PinMap::ServoPWM0,0,m_FuelPTLocal,HPTankPTapPoller,OxTankPoller,FuelTankPoller),
m_FuelPTLocalADC(PinMap::OxPTADCPin)
{};


void System::systemSetup(){
    
    Serial.setRxBufferSize(GeneralConfig::SerialRxSize);
    Serial.setTxBufferSize(GeneralConfig::SerialRxSize);
    Serial.begin(GeneralConfig::SerialBaud);
   
    //intialize rnp message logger
    loggerhandler.retrieve_logger<RicCoreLoggingConfig::LOGGERS::SYS>().initialize(networkmanager);

    //initialize statemachine with idle state
    statemachine.initalize(std::make_unique<Idle>(systemstatus,commandhandler));
    
    //any other setup goes here
    
    Buck.setup();
    FuelTankPoller.setup();
    OxTankPoller.setup();
    HPTankPTapPoller.setup();
    Greg.setup();
    canbus.setup();
    m_FuelPTLocalADC.setAttenuation(ADC_ATTEN_DB_2_5); //0 -> 1250mV input range
    m_FuelPTLocalADC.setup();
    m_FuelPTLocal.setup();
    
    networkmanager.addInterface(&canbus);

    networkmanager.setNodeType(NODETYPE::HUB);
    networkmanager.setNoRouteAction(NOROUTE_ACTION::BROADCAST,{1,3});

    //Defining these so the methods following are less ugly
    uint8_t Gregservice = static_cast<uint8_t>(Services::ID::Greg);
    uint8_t FuelTankPTapremoteservice = static_cast<uint8_t>(Services::ID::FuelTankPTRemote);
    uint8_t FuelTankPTaplocalservice = static_cast<uint8_t>(Services::ID::FuelTankPTLocal);
    uint8_t HPtankPTapservice = static_cast<uint8_t>(Services::ID::HPTankPT);
    uint8_t OxTankPTService = static_cast<uint8_t>(Services::ID::OxTankPT);

    networkmanager.registerService(Gregservice,Greg.getThisNetworkCallback());
    networkmanager.registerService(FuelTankPTaplocalservice,m_FuelPTLocal.getThisNetworkCallback());
    networkmanager.registerService(FuelTankPTapremoteservice,[this](packetptr_t packetptr){FuelTankPTap.networkCallback(std::move(packetptr));});
    networkmanager.registerService(HPtankPTapservice,[this](packetptr_t packetptr){HPtankPTap.networkCallback(std::move(packetptr));});
    networkmanager.registerService(OxTankPTService,[this](packetptr_t packetptr){OxTankPTap.networkCallback(std::move(packetptr));});
    
};

void System::systemUpdate(){
    m_FuelPTLocalADC.update();
    m_FuelPTLocal.update(static_cast<int32_t>(m_FuelPTLocalADC.getADC()));
    Buck.update();
    Greg.update();
};