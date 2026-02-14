#pragma once

#include <stdint.h>

uint8_t ADSL_HopChannel(uint8_t Sec, int32_t Alt); // decide on the channel to hop based on Second and Altitude
                                                   // 0 => 868.200MHz nRF905-like
                                                   // 1 => 868.400MHz nRF905-like
                                                   // 2 => 869.525MHz LDR = Low Data Rate
                                                   // 3 => 869.525MHz HDR = High Data Rate
