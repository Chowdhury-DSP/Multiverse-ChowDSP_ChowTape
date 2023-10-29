/*
 * Company: ChowDSP
 * Effect Name: Muff Drive
 * Description: A distortion effect based on the Electro-Harmonix Big Muff.
 *
 * This file was auto-generated by Aviate Audio Effect Creator for the Multiverse.
 */
#include "Aviate/EfxPrint.h"
#include "TapeSaturation.h"

// teensy defines these things, which causes mad conflicts...
#undef abs
#undef round
#undef B0
#undef B1
#undef B2
#include "tape_saturation_plugin.h"

using namespace Aviate;

namespace ChowDSP_TapeSaturation {

TapeSaturation::TapeSaturation()
: AudioStream(NUM_INPUTS, m_inputQueueArray)
{\''
    // perform any necessary class initialization here
    plugin = std::make_unique<ne_pedal::plugins::tape_saturation::TapeSaturationPlugin>();
    plugin->prepare (AUDIO_SAMPLE_RATE, AUDIO_SAMPLES_PER_BLOCK);
}

TapeSaturation::~TapeSaturation()
{
    // perform any necessary clean up here, though destructors are not
    // called on the hardware, only in the simulator.
}

void TapeSaturation::update(void)
{
    audio_block_t *inputAudioBlock = receiveWritable(); // get the next block of input samples
    inputAudioBlock = m_basicInputCheck(inputAudioBlock, 0); // check for disable mode, bypass, or invalid inputs. Transmit to channel 0 in bypass
    if (!inputAudioBlock) { return; } // no further processing for this update() call

    // You must call m_updateInputPeak() before processing the audio
    m_updateInputPeak(inputAudioBlock);

    // DO AUDIO EFFECT PROCESSING
    float bufferf[AUDIO_SAMPLES_PER_BLOCK];
    arm_q15_to_float (inputAudioBlock->data, bufferf, AUDIO_SAMPLES_PER_BLOCK);

    plugin->parameters[0]->set_value(m_drive);
    plugin->parameters[1]->set_value(m_saturation);
    plugin->parameters[2]->set_value(m_bias);
    plugin->parameters[3]->set_value(m_tone);
    plugin->parameters[4]->set_value(m_volume);
    plugin->processBlock (chowdsp::BufferView<float> { bufferf, (int) AUDIO_SAMPLES_PER_BLOCK });

    arm_float_to_q15(bufferf, inputAudioBlock->data, AUDIO_SAMPLES_PER_BLOCK);

    m_updateOutputPeak(inputAudioBlock); // you must call m_upateOutputPeak() at the end of update() before transmit
    transmit(inputAudioBlock);
    release(inputAudioBlock);
}

void TapeSaturation::volume(float value)
{
    m_volume = value;
}

void TapeSaturation::drive(float value)
{
    // perform any necessary conversion to user variables, validation, etc.
    m_drive = value;
}

void TapeSaturation::saturation(float value)
{
    // perform any necessary conversion to user variables, validation, etc.
    m_saturation = value;
}

void TapeSaturation::bias(float value)
{
    // perform any necessary conversion to user variables, validation, etc.
    m_bias = value;
}

void TapeSaturation::tone(float value)
{
    // perform any necessary conversion to user variables, validation, etc.
    m_tone = value;
}
}
