/*
**********************
* PINS               *
**********************
 */
#pragma once
#include <stdint.h>

namespace PinMap{
    
    static constexpr uint8_t TxCan = 33;
    static constexpr uint8_t RxCan = 34;

    static constexpr uint8_t ServoPWM1 = 36;
    static constexpr uint8_t ServoPWM2 = 37;

    static constexpr uint8_t BuckEN = 38;
    static constexpr uint8_t BuckPGOOD = 37;
    static constexpr uint8_t BuckOutputV = 4;

    static constexpr uint8_t EngineOverride = 8;
    static constexpr uint8_t LPTankP = 9; //Find out what pin the PT is on

};


