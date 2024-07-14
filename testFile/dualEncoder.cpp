#include "dualEncoder.hpp"

// // mtrn3100::DualEncoder::instance;
// // DualEncoder::instance = nullptr;
// // static DualEncoder* mtrn3100::DualEncoder::instance = nullptr;
// mtrn3100::DualEncoder::DualEncoder(uint8_t enc1, uint8_t enc2, uint8_t enc3, uint8_t enc4)
//  : mot1_int(enc1), mot1_dir(enc2), mot2_int(enc3), mot2_dir(enc4) {
//     // instance = this;  // Store the instance pointer
//     static DualEncoder* instance = this;
//     pinMode(mot1_int, INPUT_PULLUP);
//     pinMode(mot1_dir, INPUT_PULLUP);
//     pinMode(mot2_int, INPUT_PULLUP);
//     pinMode(mot2_dir, INPUT_PULLUP);
    
//     attachInterrupt(digitalPinToInterrupt(mot1_int), DualEncoder::readLeftEncoderISR, RISING);
//     attachInterrupt(digitalPinToInterrupt(mot2_int), DualEncoder::readRightEncoderISR, RISING);
// }

// // Encoder function used to update the encoder
// void mtrn3100::DualEncoder::readLeftEncoder() {
//     noInterrupts();
//     direction = digitalRead(mot1_dir) ? 1 : -1;
//     l_count += direction;
//     interrupts();
// }
    
// void mtrn3100::DualEncoder::readRightEncoder() {
//     noInterrupts();
//     direction = digitalRead(mot2_dir) ? 1 : -1;
//     r_count += direction;
//     interrupts();
// }

// // Helper function to convert encouder count to radians
// float mtrn3100::DualEncoder::getLeftRotation() {
//     return (static_cast<float>(l_count) / counts_per_revolution ) * 2*PI;
// }

// float mtrn3100::DualEncoder::getRightRotation() {
//     return (static_cast<float>(r_count) / counts_per_revolution ) * 2 * PI;
// }

// static void mtrn3100::DualEncoder::readLeftEncoderISR() {
//     if (instance != nullptr) {
//         instance->readLeftEncoder();
//     }
// }

// static void mtrn3100::DualEncoder::readRightEncoderISR() {
//     if (instance != nullptr) {
//         instance->readRightEncoder();
//     }
// }


// mtrn3100::DualEncoder::instance;
// DualEncoder::instance = nullptr;
// static DualEncoder* mtrn3100::DualEncoder::instance = nullptr;
namespace mtrn3100{
  DualEncoder* DualEncoder::instance = nullptr;
DualEncoder::DualEncoder(uint8_t enc1, uint8_t enc2, uint8_t enc3, uint8_t enc4)
 : mot1_int(enc1), mot1_dir(enc2), mot2_int(enc3), mot2_dir(enc4) {
    // instance = this;  // Store the instance pointer
    static DualEncoder* instance = this;
    pinMode(mot1_int, INPUT_PULLUP);
    pinMode(mot1_dir, INPUT_PULLUP);
    pinMode(mot2_int, INPUT_PULLUP);
    pinMode(mot2_dir, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(mot1_int), DualEncoder::readLeftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(mot2_int), DualEncoder::readRightEncoderISR, RISING);
}

// Encoder function used to update the encoder
void DualEncoder::readLeftEncoder() {
    noInterrupts();
    direction = digitalRead(mot1_dir) ? 1 : -1;
    l_count += direction;
    l_position += getLeftRotation();
    interrupts();
}
    
void DualEncoder::readRightEncoder() {
    noInterrupts();
    direction = digitalRead(mot2_dir) ? 1 : -1;
    r_count += direction;
    r_position += getRightRotation();
    interrupts();
}

// Helper function to convert encouder count to radians
float DualEncoder::getLeftRotation() {
    return (static_cast<float>(l_count) / counts_per_revolution ) * 2*PI;
}

float DualEncoder::getRightRotation() {
    return (static_cast<float>(r_count) / counts_per_revolution ) * 2 * PI;
}

static void DualEncoder::readLeftEncoderISR() {
    if (instance != nullptr) {
        instance->readLeftEncoder();
    }
}

static void DualEncoder::readRightEncoderISR() {
    if (instance != nullptr) {
        instance->readRightEncoder();
    }
}
}
