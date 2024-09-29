#if ! NE_PEDAL_MULTIVERSE

#include "PluginBase.h"
#include <clap/helpers/host-proxy.hxx>
#include <clap/helpers/plugin.hxx>

// teensy defines these things, which causes mad conflicts...
#undef abs
#undef B0
#undef B1
#undef B2
// clang-format off
#include <chowdsp_core/chowdsp_core.cpp>
#include <chowdsp_buffers/chowdsp_buffers.cpp>
#include <chowdsp_dsp_data_structures/chowdsp_dsp_data_structures.cpp>
#include <chowdsp_math/chowdsp_math.cpp>
// clang-format on

template class clap::helpers::Plugin<ne_pedal::plugins::CLAPMisbehaviourHandler, ne_pedal::plugins::CLAPCheckingLevel>;
template class clap::helpers::HostProxy<ne_pedal::plugins::CLAPMisbehaviourHandler, ne_pedal::plugins::CLAPCheckingLevel>;

#endif
