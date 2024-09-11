/*
**********************
* PINS               *
**********************
 */
#pragma once
#include <stdint.h>

namespace PinMap{
    //Greg
    static constexpr uint8_t TxCan = 33;
    static constexpr uint8_t RxCan = 34;

    //Pickle
    // static constexpr int TxCan = 32;
    // static constexpr int RxCan = 33;

    static constexpr uint8_t ServoPWM0 = 36;
    static constexpr uint8_t ServoPWM1 = 37;

    static constexpr uint8_t BuckEN = 38;
    static constexpr uint8_t BuckPGOOD = 37;
    static constexpr uint8_t BuckOutputV = 4;

    static constexpr uint8_t EngineOverride = 8;
    static constexpr uint8_t OxPTADCPin = 7; //Find out what pin the PT is on

};


