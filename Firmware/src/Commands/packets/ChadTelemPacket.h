#pragma once

#include <librnp/rnp_packet.h>
#include <librnp/rnp_serializer.h>

#include <vector>

//shamelessly copied from pickle rick's telemetry packet

class ChadTelemPacket : public RnpPacket{
    private:
    //serializer framework
        static constexpr auto getSerializer()
        {
            auto ret = RnpSerializer(
                &ChadTelemPacket::servoVoltage,
                &ChadTelemPacket::ox_tankP,
                &ChadTelemPacket::regAngle,
                &ChadTelemPacket::P_angle,
                &ChadTelemPacket::I_angle,
                &ChadTelemPacket::system_status,
                &ChadTelemPacket::system_time
            );

            return ret;
        }
        
    public:
        ~ChadTelemPacket();

        ChadTelemPacket();
        /**
         * @brief Deserialize Telemetry Packet
         * 
         * @param data 
         */
        ChadTelemPacket(const RnpPacketSerialized& packet);

        /**
         * @brief Serialize Telemetry Packet
         * 
         * @param buf 
         */
        void serialize(std::vector<uint8_t>& buf) override;

        float servoVoltage;
        float ox_tankP;
        float regAngle;
        float P_angle;
        float I_angle;
        uint32_t system_status;
        uint64_t system_time;

        static constexpr size_t size(){
            return getSerializer().member_size();
        }

};


