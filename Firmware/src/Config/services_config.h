/**
 * @file services_config.h
 * @author Kiran de Silva (kd619@ic.ac.uk)
 * @brief Defintion of user defined services
 * @version 0.1
 * @date 2023-06-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <stdint.h>


namespace Services{
    /**
     * @brief ID of user defined services NB start at 2, all ID's below 2 are reserved for default services found in 'rnp_networkmanager.h'
     * 
     */
    enum class ID:uint8_t{
        Greg = 10,
        FuelTankPTLocal = 11,
        OxTankPT = 16,
        FuelTankPTRemote = 12,
        HPTankPT = 13 //CHECK, THIS IS PROBABLY WRONG
    };

};