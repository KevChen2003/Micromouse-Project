#pragma once

#include <Arduino.h>

namespace mtrn3100 {

    class DualEncoder {
    public:
        DualEncoder(uint8_t enc1, uint8_t enc2, uint8_t enc3, uint8_t enc4);

        void readLeftEncoder();
        void readRightEncoder();
        
        float getLeftRotation();
        float getRightRotation();

        const uint8_t mot1_int, mot1_dir, mot2_int, mot2_dir;
        volatile int8_t direction;
        float l_position = 0;
        float r_position = 0;

        uint16_t counts_per_revolution = 700;
        volatile long l_count = 0;
        volatile long r_count = 0;
        uint32_t prev_time;
        bool read = false;

    private:
        static void readLeftEncoderISR();
        static void readRightEncoderISR();

        static DualEncoder* instance ;
    };
    // DualEncoder* DualEncoder::instance = nullptr;
}