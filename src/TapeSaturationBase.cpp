/*
 * Company: ChowDSP
 * Effect Name: Tape Saturation
 * Description: A distortion effect based on the Electro-Harmonix Big Muff.
 *
 * This file was auto-generated by Aviate Audio Effect Creator for the Multiverse.
 */
#include <cmath>
#include "Aviate/LibBasicFunctions.h"
#include "TapeSaturation.h"

using namespace Aviate;

namespace ChowDSP_TapeSaturation {

void TapeSaturation::mapMidiControl(int parameter, int midiCC, int midiChannel)
{
    if (parameter >= NUM_CONTROLS) {
        return ; // Invalid midi parameter
    }
    m_midiConfig[parameter][MIDI_CHANNEL] = midiChannel;
    m_midiConfig[parameter][MIDI_CONTROL] = midiCC;
}

void TapeSaturation::setParam(int paramIndex, float paramValue)
{
    switch(paramIndex) {
    case 0 : bypass( (paramValue - 0.000000) / (1.000000 - 0.000000) ); break;
    case 1 : volume( (paramValue - 0.000000) / (1.000000 - 0.000000) ); break;
    case 2 : drive( (paramValue - 0.000000) / (1.000000 - 0.000000) ); break;
    case 3 : saturation( (paramValue - 0.000000) / (1.000000 - 0.000000) ); break;
    case 4 : bias( (paramValue - 0.000000) / (1.000000 - 0.000000) ); break;
    case 5 : tone( (paramValue - -1.000000) / (1.000000 - -1.000000) ); break;
    case 6 : speed( (paramValue - 0.000000) / (50.000000 - 0.000000) ); break;
    default : break;
    }
}

float TapeSaturation::getUserParamValue(int paramIndex, float normalizedParamValue)
{
    switch(paramIndex) {
    case 0 : return ( ((1.000000 - 0.000000) * normalizedParamValue) + 0.000000 ); // bypass
    case 1 : return ( ((1.000000 - 0.000000) * normalizedParamValue) + 0.000000 ); // volume
    case 2 : return ( ((1.000000 - 0.000000) * normalizedParamValue) + 0.000000 ); // drive
    case 3 : return ( ((1.000000 - 0.000000) * normalizedParamValue) + 0.000000 ); // saturation
    case 4 : return ( ((1.000000 - 0.000000) * normalizedParamValue) + 0.000000 ); // bias
    case 5 : return ( ((1.000000 - -1.000000) * normalizedParamValue) + -1.000000 ); // tone
    case 6 : return ( ((50.000000 - 0.000000) * normalizedParamValue) + 0.000000 ); // speed
    default : return 0.0f;
    }
}

void TapeSaturation::processMidi(int channel, int control, int value)
{
    float val = (float)value / 127.0f;

    if ((m_midiConfig[Bypass_e][MIDI_CHANNEL] == channel) && (m_midiConfig[Bypass_e][MIDI_CONTROL] == control)) {
        bypass(val);
        return;
    }

    if ((m_midiConfig[Volume_e][MIDI_CHANNEL] == channel) && (m_midiConfig[Volume_e][MIDI_CONTROL] == control)) {
        volume(val);
        return;
    }

    if ((m_midiConfig[Drive_e][MIDI_CHANNEL] == channel) && (m_midiConfig[Drive_e][MIDI_CONTROL] == control)) {
        drive(val);
        return;
    }

    if ((m_midiConfig[Saturation_e][MIDI_CHANNEL] == channel) && (m_midiConfig[Saturation_e][MIDI_CONTROL] == control)) {
        saturation(val);
        return;
    }

    if ((m_midiConfig[Bias_e][MIDI_CHANNEL] == channel) && (m_midiConfig[Bias_e][MIDI_CONTROL] == control)) {
        bias(val);
        return;
    }

    if ((m_midiConfig[Tone_e][MIDI_CHANNEL] == channel) && (m_midiConfig[Tone_e][MIDI_CONTROL] == control)) {
        tone(val);
        return;
    }

    if ((m_midiConfig[Speed_e][MIDI_CHANNEL] == channel) && (m_midiConfig[Speed_e][MIDI_CONTROL] == control)) {
        speed(val);
        return;
    }

}

audio_block_t* TapeSaturation::m_basicInputCheck(audio_block_t* inputAudioBlock, unsigned outputChannel)
{
    // Check if effect is disabled
    if (m_enable == false) {
        // do not transmit or process any audio, return as quickly as possible after releasing the inputs
        if (inputAudioBlock) { release(inputAudioBlock); }
        return nullptr; // disabled, no further EFX processing in update()
    }  // end of enable check

    // check if effect is in bypass
    if (m_bypass == true) {
        // drive input directly to the specified output. ie. bypass
        if (inputAudioBlock != nullptr) {
            // valid input, drive to outputChannel if specified
            if (outputChannel >= 0) {
                transmit(inputAudioBlock, outputChannel); // drive to specified output
            }
            release(inputAudioBlock); // release the input block as we are done with it
        } else { // invalid input block, allocate a block and drive silence if specified
            if (outputChannel >= 0) {
                audio_block_t* silenceBlock = allocate();
                if (silenceBlock) {
                    clearAudioBlock(silenceBlock);  // create silence in the buffer
                    transmit(silenceBlock, outputChannel);
                    release(silenceBlock);
                }
            }
        }
        return nullptr;  // bypassed, no further EFX processing in update()
    }  // end of bypass check

    // If not disabled or bypassed, create silence if the input block is invalid then
    // return the valid audio block so update() can continue.
    if (inputAudioBlock == nullptr) {
        inputAudioBlock = allocate();
        if (inputAudioBlock == nullptr) { return nullptr; } // check if allocate was unsuccessful
        // else
        clearAudioBlock(inputAudioBlock);
    }
    return inputAudioBlock; // inputAudioBLock is valid and ready for update() processing
}

const uint8_t rblk[256] = { 0xd0, 0x3e, 0x53, 0xd6, 0x6e, 0x42, 0xfc, 0x5f, 0xdf, 0x8b, 0x50, 0x16, 0x02, 0xa2, 0xe1, 0x99, 0x8f, 0x4c, 0x30, 0x43, 0x79, 0x1e, 0xec, 0xfa, 0x5e, 0x5f, 0x1c, 0xff, 0x96, 0x0c, 0x4c, 0xe1, 0x67, 0x09, 0x77, 0x07, 0x46, 0x07, 0xf4, 0x41, 0x18, 0x41, 0x4a, 0xe3, 0xe9, 0xcf, 0x12, 0xd6, 0x1f, 0xc8, 0xf9, 0x08, 0x45, 0x4b, 0xe0, 0xc4, 0x17, 0x17, 0x2e, 0x88, 0x5f, 0x7e, 0xdb, 0x04, 0x02, 0x6b, 0xf2, 0x01, 0x96, 0xc4, 0xdd, 0x2f, 0x96, 0xe6, 0x5e, 0x73, 0xa1, 0x3f, 0x49, 0xfa, 0x60, 0xe9, 0xb6, 0x6f, 0x3c, 0x87, 0xd1, 0xe9, 0xfb, 0x1e, 0x29, 0xf8, 0xdf, 0x43, 0xf2, 0xcd, 0x15, 0x80, 0x23, 0xdc, 0x1d, 0x21, 0x0a, 0x70, 0x4f, 0x28, 0xf1, 0x0b, 0xf4, 0x02, 0x6b, 0x12, 0xa3, 0x44, 0xa3, 0x45, 0x16, 0x07, 0x56, 0x02, 0x35, 0x16, 0x47, 0x57, 0x27, 0x16, 0x47, 0x96, 0xf6, 0xe6, 0x00, 0x3f, 0x33, 0xf7, 0xc9, 0xbe, 0x78, 0xa4, 0x28, 0x0f, 0x6b, 0xad, 0x80, 0x28, 0x7e, 0xba, 0xf9, 0x0a, 0xb2, 0x74, 0x8c, 0xd0, 0x55, 0xbb, 0xc7, 0x9b, 0x1d, 0x5a, 0xec, 0x70, 0x9d, 0xc9, 0xe6, 0x72, 0xb3, 0x7e, 0x66, 0x2f, 0x7f, 0x28, 0x49, 0xf7, 0x66, 0x0f, 0x91, 0xef, 0xa0, 0x54, 0x31, 0x8a, 0x5d, 0xca, 0xe5, 0x75, 0x20, 0xc6, 0xa0, 0x05, 0x7b, 0x8a, 0x33, 0xf0, 0x9a, 0xd9, 0x13, 0x59, 0x90, 0xb5, 0xb0, 0x14, 0x81, 0x54, 0x12, 0xdd, 0x5d, 0x13, 0x85, 0x4b, 0x47, 0x1d, 0x97, 0xb5, 0x48, 0x2a, 0x7f, 0x0c, 0xed, 0x61, 0xff, 0x22, 0x2b, 0xc6, 0x62, 0x18, 0x31, 0xdd, 0xc4, 0x40, 0x1b, 0x64, 0x85, 0x33, 0xcb, 0xa4, 0xdb, 0x3c, 0xc8, 0x0f, 0xbe, 0x01, 0xbf, 0x3d, 0xe4, 0x1e, 0x03, 0x08, 0x5e, 0x08, 0xcb, 0x6d, 0x16, 0x23, 0x79, 0xed, 0x1c, 0x04};
const uint8_t* TapeSaturation::getRblk() { return rblk; }
static constexpr char PROGMEM TapeSaturation_name[] = {0x43, 0x68, 0x6f, 0x77, 0x44, 0x53, 0x50, 0x3a, 0x44, 0x3a, 0x54, 0x61, 0x70, 0x65, 0x20, 0x53, 0x61, 0x74, 0x75, 0x72, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x0};
const char* TapeSaturation::getName() { return TapeSaturation_name; }

}
