/*
 * Company: ChowDSP
 * Effect Name: Tape Saturation
 * Description: A distortion effect based on a physically-modelled reel-to-reel tape machine.
 *
 * This file was auto-generated by Aviate Audio Effect Creator for the Multiverse.
 */
#include <cmath>
#include "Aviate/LibBasicFunctions.h"
#include "TapeSaturation.h"

// Some useful aliases, file scope only
#define audioBlockReceiveReadOnly receiveReadOnlyFloat
#define audioBlockReceiveWritable receiveWritableFloat
#define audioBlockAllocate        allocateFloat

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

void TapeSaturation::processMidi(int status, int data1, int data2)
{
}

audio_block_float32_t* TapeSaturation::m_basicInputCheck(audio_block_float32_t* inputAudioBlock, unsigned outputChannel)
{
    // Check if effect is disabled
    if (m_enable == false) {
        // do not transmit or process any audio, return as quickly as possible after releasing the inputs
        if (inputAudioBlock) { AudioStream::release(inputAudioBlock); }
        return nullptr; // disabled, no further EFX processing in update()
    }  // end of enable check

    // check if effect is in bypass
    if (m_bypass == true) {
        // drive input directly to the specified output. ie. bypass
        if (inputAudioBlock != nullptr) {
            // valid input, drive to outputChannel if specified
            if (outputChannel >= 0) {
                AudioStream::transmit(inputAudioBlock, outputChannel); // drive to specified output
            }
            AudioStream::release(inputAudioBlock); // release the input block as we are done with it
        } else { // invalid input block, allocate a block and drive silence if specified
            if (outputChannel >= 0) {
                audio_block_float32_t* silenceBlock = allocateFloat();
                if (silenceBlock) {
                    clearAudioBlock(silenceBlock);  // create silence in the buffer
                    AudioStream::transmit(silenceBlock, outputChannel);
                    AudioStream::release(silenceBlock);
                }
            }
        }
        return nullptr;  // bypassed, no further EFX processing in update()
    }  // end of bypass check

    // If not disabled or bypassed, create silence if the input block is invalid then
    // return the valid audio block so update() can continue.
    if (inputAudioBlock == nullptr) {
        inputAudioBlock = AudioStream::allocateFloat();
        if (inputAudioBlock == nullptr) { return nullptr; } // check if allocate was unsuccessful
        // else
        clearAudioBlock(inputAudioBlock);
    }
    return inputAudioBlock; // inputAudioBLock is valid and ready for update() processing
}

const uint8_t rblk[256] = TEENSY_AUDIO_BLOCK;
const uint8_t* TapeSaturation::getRblk() { return rblk; }
static constexpr char PROGMEM TapeSaturation_name[] = {0x43, 0x68, 0x6f, 0x77, 0x44, 0x53, 0x50, 0x3a, 0x54, 0x61, 0x70, 0x65, 0x20, 0x53, 0x61, 0x74, 0x75, 0x72, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x0};
const char* TapeSaturation::getName() { return TapeSaturation_name; }

}
