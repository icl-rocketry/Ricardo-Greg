/**
 * @file commands.cpp
 * @author Kiran de Silva (kd619@ic.ac.uk)
 * @brief Implementation of commands for system
 * @version 0.1
 * @date 2023-06-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "commands.h"

#include "packets/ChadTelemPacket.h"

#include <librnp/rnp_packet.h>
#include <libriccore/commands/commandhandler.h>

#include "system.h"


void Commands::FreeRamCommand(System& sm, const RnpPacketSerialized& packet)
{	
	//avliable in all states
	//returning as simple string packet for ease
	//currently only returning free ram
	MessagePacket_Base<0,static_cast<uint8_t>(decltype(System::commandhandler)::PACKET_TYPES::MESSAGE_RESPONSE)> message("FreeRam: " + std::to_string(esp_get_free_heap_size()));
	// this is not great as it assumes a single command handler with the same service ID
	// would be better if we could pass some context through the function paramters so it has an idea who has called it
	// or make it much clearer that only a single command handler should exist in the system
	message.header.source_service = sm.commandhandler.getServiceID(); 
	
	
	message.header.destination_service = packet.header.source_service;
	message.header.source = packet.header.destination;
	message.header.destination = packet.header.source;
	message.header.uid = packet.header.uid;
	sm.networkmanager.sendPacket(message);
	
}

void Commands::ChadTelemCommand(System& sm, const RnpPacketSerialized& packet)
{	
	SimpleCommandPacket commandpacket(packet);

	ChadTelemPacket chadtelem;

	chadtelem.header.type = static_cast<uint8_t>(103);
	chadtelem.header.source = sm.networkmanager.getAddress();
	chadtelem.header.source_service = sm.commandhandler.getServiceID();
	chadtelem.header.destination = commandpacket.header.source;
	chadtelem.header.destination_service = commandpacket.header.source_service;
	chadtelem.header.uid = commandpacket.header.uid; 
	chadtelem.lptankP = sm.Thanos.get_lptankP();
	chadtelem.fuelAngle = sm.Thanos.getFuelAngle();
	chadtelem.regAngle = sm.Thanos.getRegAngle(); //Changed name from ox to reg
	chadtelem.thanosState = sm.Thanos.getStatus();
	chadtelem.system_status = sm.systemstatus.getStatus();
	chadtelem.system_time = millis();
	
	sm.networkmanager.sendPacket(chadtelem);
	
}

void Commands::BuckRestartCommand(System& sm, const RnpPacketSerialized& packet)
{	
	SimpleCommandPacket receivedpacket(packet);

	sm.Buck.restart(receivedpacket.arg); //Arg should be the time the buck is held off before startup is called again 

}