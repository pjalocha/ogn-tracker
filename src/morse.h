#pragma once

#define MORSE_DOT                                      0b0
#define MORSE_DASH                                     0b1
#define MORSE_GUARDBIT                                 0b1
#define MORSE_UNSUPPORTED                              0xFF
#define MORSE_ASCII_OFFSET                             32

#define MORSE_INTER_SYMBOL                             0x00
#define MORSE_CHAR_COMPLETE                            0x01
#define MORSE_WORD_COMPLETE                            0x02
#define MORSE_UNKNOWN_SYMBOL                           '*'

// Morse character table: - using codes defined in ITU-R M.1677-1
//                        - Morse code representation is saved LSb first, using additional bit as guard
//                        - position in array corresponds ASCII code minus MORSE_ASCII_OFFSET
//                        - ASCII characters marked MORSE_UNSUPPORTED do not have ITU-R M.1677-1 equivalent
static const uint8_t MorseTable[] PROGMEM = {
    0b00,                         // space
    0b110101,                     // ! (unsupported)
    0b1010010,                    // "
    MORSE_UNSUPPORTED,   // # (unsupported)
    MORSE_UNSUPPORTED,   // $ (unsupported)
    MORSE_UNSUPPORTED,   // % (unsupported)
    MORSE_UNSUPPORTED,   // & (unsupported)
    0b1011110,                    // '
    0b101101,                     // (
    0b1101101,                    // )
    MORSE_UNSUPPORTED,   // * (unsupported)
    0b101010,                     // +
    0b1110011,                    // ,
    0b1100001,                    // -
    0b1101010,                    // .
    0b101001,                     // /
    0b111111,                     // 0
    0b111110,                     // 1
    0b111100,                     // 2
    0b111000,                     // 3
    0b110000,                     // 4
    0b100000,                     // 5
    0b100001,                     // 6
    0b100011,                     // 7
    0b100111,                     // 8
    0b101111,                     // 9
    0b1000111,                    // :
    MORSE_UNSUPPORTED,   // ; (unsupported)
    MORSE_UNSUPPORTED,   // < (unsupported)
    0b110001,                     // =
    MORSE_UNSUPPORTED,   // > (unsupported)
    0b1001100,                    // ?
    0b1010110,                    // @
    0b110,                        // A
    0b10001,                      // B
    0b10101,                      // C
    0b1001,                       // D
    0b10,                         // E
    0b10100,                      // F
    0b1011,                       // G
    0b10000,                      // H
    0b100,                        // I
    0b11110,                      // J
    0b1101,                       // K
    0b10010,                      // L
    0b111,                        // M
    0b101,                        // N
    0b1111,                       // O
    0b10110,                      // P
    0b11011,                      // Q
    0b1010,                       // R
    0b1000,                       // S
    0b11,                         // T
    0b1100,                       // U
    0b11000,                      // V
    0b1110,                       // W
    0b11001,                      // X
    0b11101,                      // Y
    0b10011,                      // Z
    MORSE_UNSUPPORTED,   // [ (unsupported)
    MORSE_UNSUPPORTED,   // \ (unsupported)
    MORSE_UNSUPPORTED,   // ] (unsupported)
    0b1101000,                    // ^ (unsupported, used as alias for end of work)
    0b110101                      // _ (unsupported, used as alias for starting signal)
};
