#pragma once


namespace GeneralConfig{
    //Serial baud rate
    static constexpr int SerialBaud = 115200;
    //Serial rx buffer size
    static constexpr int SerialRxSize = 2048;

    //I2C frequrency - 4Khz
    static constexpr int I2C_FREQUENCY = 400000;

    //Addressing
    static constexpr uint8_t KermitAddr = 12;
    static constexpr uint8_t StarkAddr = 10;
    
};







