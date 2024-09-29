#pragma once

#include "Parameters.h"
#include <cstring>

// teensy defines these things, which causes mad conflicts...
#undef abs
#undef B0
#undef B1
#undef B2
#include <chowdsp_core/chowdsp_core.h>
#include <chowdsp_dsp_data_structures/chowdsp_dsp_data_structures.h>
#include <chowdsp_math/chowdsp_math.h>

#if NE_PEDAL_MULTIVERSE
using clap_host = void;
using clap_plugin_descriptor = chowdsp::NullType;
using clap_log_severity = int32_t;
#else
#include <clap/helpers/plugin.hh>
#endif

namespace ne_pedal::plugins
{
#if NE_PEDAL_MULTIVERSE
struct CLAPPlugin
{
    virtual ~CLAPPlugin() = default;
};
#else
static constexpr auto CLAPMisbehaviourHandler = clap::helpers::MisbehaviourHandler::Ignore;
static constexpr auto CLAPCheckingLevel = clap::helpers::CheckingLevel::Minimal;
using CLAPPlugin = clap::helpers::Plugin<CLAPMisbehaviourHandler, CLAPCheckingLevel>;
#endif

template <typename PluginType, size_t max_num_params = 4>
class PluginBase : public CLAPPlugin
{
public:
    explicit PluginBase (const clap_host* host);
    ~PluginBase() override = default;

    virtual void prepare (double sampleRate, uint32_t samplesPerBlock) = 0;
    virtual void processBlock (const chowdsp::BufferView<float>& buffer) = 0;

    void addParameter (Parameter* new_param);
    void addParameters (std::initializer_list<Parameter*> new_params);

    void log_message (clap_log_severity severity, const std::string& message);

    std::array<Parameter*, max_num_params> parameters {};

private:
#if NE_PEDAL_MULTIVERSE
    uint32_t paramsCount() const noexcept;
#else
    /** clap plugin */
    bool activate (double sampleRate, uint32_t minFrameCount, uint32_t maxFrameCount) noexcept override;
    clap_process_status process (const clap_process* process) noexcept override;
    void processInputEvents (const clap_input_events& input_events);

    /** clap audio ports */
    bool implementsAudioPorts() const noexcept override { return true; }
    uint32_t audioPortsCount (bool /*isInput*/) const noexcept override { return 1; }
    bool audioPortsInfo (uint32_t index, bool isInput, clap_audio_port_info* info) const noexcept override;

    /** clap params */
    bool implementsParams() const noexcept override { return true; }
    uint32_t paramsCount() const noexcept override;
    bool paramsInfo (uint32_t paramIndex, clap_param_info* info) const noexcept override;
    bool paramsValue (clap_id paramId, double* value) noexcept override;
    bool paramsValueToText (clap_id paramId, double value, char* display, uint32_t size) noexcept override;
    bool paramsTextToValue (clap_id paramId, const char* display, double* value) noexcept override;
    void paramsFlush (const clap_input_events* in, const clap_output_events* out) noexcept override {}
    bool isValidParamId (clap_id paramId) const noexcept override;
#endif
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (PluginBase)
};

template <typename PluginType, size_t max_num_params>
PluginBase<PluginType, max_num_params>::PluginBase (const clap_host* host)
#if ! NE_PEDAL_MULTIVERSE
    : clap::helpers::Plugin<CLAPMisbehaviourHandler, CLAPCheckingLevel> (&PluginType::description, host)
#endif
{
}

template <typename PluginType, size_t max_num_params>
void PluginBase<PluginType, max_num_params>::addParameter (Parameter* new_param)
{
    for (auto& param : parameters)
    {
        if (param == nullptr)
        {
            param = new_param;
            return;
        }
    }

    // too many parameters!
    assert (false);
}
template <typename PluginType, size_t max_num_params>
void PluginBase<PluginType, max_num_params>::addParameters (std::initializer_list<Parameter*> new_params)
{
    auto param_count = paramsCount();
    for (auto* param : new_params)
    {
        if (param_count == max_num_params)
        {
            assert (false);
            return;
        }
        parameters[param_count++] = param;
    }
}

template <typename PluginType, size_t max_num_params>
uint32_t PluginBase<PluginType, max_num_params>::paramsCount() const noexcept
{
    uint32_t count = 0;
    for (const auto* param : parameters)
    {
        if (param == nullptr)
            break;
        count++;
    }
    return count;
}

template <typename PluginType, size_t max_num_params>
void PluginBase<PluginType, max_num_params>::log_message (clap_log_severity severity, const std::string& message)
{
#if ! NE_PEDAL_MULTIVERSE
    if (_host.canUseHostLog())
        _host.log (severity, message.data());
#endif
}

#if ! NE_PEDAL_MULTIVERSE
template <typename PluginType, size_t max_num_params>
bool PluginBase<PluginType, max_num_params>::activate (double sampleRate, uint32_t /*minFrameCount*/, uint32_t maxFrameCount) noexcept
{
    prepare (sampleRate, maxFrameCount);
    return true;
}

template <typename PluginType, size_t max_num_params>
clap_process_status PluginBase<PluginType, max_num_params>::process (const clap_process* process) noexcept
{
    if (process->in_events != nullptr)
        processInputEvents (*process->in_events);

    if (process->audio_inputs != process->audio_outputs)
    {
        // for now, we require IN PLACE processing
        for (uint32_t ch = 0; ch < process->audio_inputs->channel_count; ++ch)
            juce::FloatVectorOperations::copy (process->audio_outputs->data32[ch], process->audio_inputs->data32[ch], (int) process->frames_count);
    }

    chowdsp::BufferView<float> buffer { process->audio_outputs->data32, (int) process->audio_outputs->channel_count, (int) process->frames_count };
    processBlock (buffer);

    return CLAP_PROCESS_CONTINUE;
}

template <typename PluginType, size_t max_num_params>
void PluginBase<PluginType, max_num_params>::processInputEvents (const clap_input_events& input_events)
{
    auto process_event = [this] (const clap_event_header& event_header)
    {
        if (event_header.space_id != CLAP_CORE_EVENT_SPACE_ID)
            return; // only core events for now

        if (event_header.type == CLAP_EVENT_PARAM_VALUE)
        {
            const auto& param_event = reinterpret_cast<const clap_event_param_value&> (event_header);
            assert (parameters[param_event.param_id] == param_event.cookie);
            parameters[param_event.param_id]->set_value ((float) param_event.value);
        }
        else if (event_header.type == CLAP_EVENT_PARAM_MOD)
        {
            const auto& param_mod_event = reinterpret_cast<const clap_event_param_mod&> (event_header);
            assert (parameters[param_mod_event.param_id] == param_mod_event.cookie);

            auto param = parameters[param_mod_event.param_id];
            if (param->supports_modulation())
            {
                param->apply_modulation ((float) param_mod_event.amount);
            }
            else
            {
                // this should never happen!
                // assert!
            }
        }
    };

    const auto num_events = input_events.size (&input_events);
    for (uint32_t i = 0; i < num_events; ++i)
    {
        const auto event = input_events.get (&input_events, i);
        if (event == nullptr)
            continue;

        process_event (*event);
    }
}

template <typename PluginType, size_t max_num_params>
bool PluginBase<PluginType, max_num_params>::audioPortsInfo (uint32_t index, bool isInput, clap_audio_port_info* info) const noexcept
{
    auto get_port_index = [] (bool input_port)
    {
        return input_port ? 0 : 1;
    };

    info->id = get_port_index (isInput);
    info->in_place_pair = get_port_index (! isInput);
    strncpy (info->name, "main", CLAP_NAME_SIZE);
    info->flags = CLAP_AUDIO_PORT_IS_MAIN;
    info->channel_count = 1;
    info->port_type = CLAP_PORT_MONO;
    return true;
}

template <typename PluginType, size_t max_num_params>
bool PluginBase<PluginType, max_num_params>::paramsInfo (uint32_t paramIndex, clap_param_info* info) const noexcept
{
    if (paramIndex >= parameters.size())
        return false;

    parameters[paramIndex]->get_info (info);
    return true;
}

template <typename PluginType, size_t max_num_params>
bool PluginBase<PluginType, max_num_params>::paramsValue (clap_id paramId, double* value) noexcept
{
    if (! isValidParamId (paramId))
        return false;

    *value = (double) parameters[paramId]->get();
    return true;
}

template <typename PluginType, size_t max_num_params>
bool PluginBase<PluginType, max_num_params>::paramsValueToText (clap_id paramId, double value, char* display, uint32_t size) noexcept
{
    if (! isValidParamId (paramId))
        return false;

    const auto param = parameters[paramId];
    if (param->valueToText == nullptr)
        return false;

    const auto text = param->valueToText ((float) value);
    strncpy (display, text.data(), size);
    return true;
}

template <typename PluginType, size_t max_num_params>
bool PluginBase<PluginType, max_num_params>::paramsTextToValue (clap_id paramId, const char* display, double* value) noexcept
{
    if (! isValidParamId (paramId))
        return false;

    const auto param = parameters[paramId];
    if (param->textToValue == nullptr)
        return false;

    *value = (double) param->textToValue (display);
    return true;
}

template <typename PluginType, size_t max_num_params>
bool PluginBase<PluginType, max_num_params>::isValidParamId (clap_id paramId) const noexcept
{
    return paramId < paramsCount();
}
#endif // ! NE_PEDAL_MULTIVERSE
} // namespace ne_pedal::plugins

#if NE_PEDAL_MULTIVERSE
#define MAKE_CLAP_DESCRIPTION(PluginClass, plugin_id, plugin_name, plugin_description)
#else
#define MAKE_CLAP_DESCRIPTION(PluginClass, plugin_id, plugin_name, plugin_description)                                                \
    const char* features[] = { CLAP_PLUGIN_FEATURE_AUDIO_EFFECT, nullptr };                                                           \
    clap_plugin_descriptor PluginClass::description = {                                                                               \
        CLAP_VERSION, plugin_id, plugin_name, "chowdsp", "https://chowdsp.com", "", "", NEPEDAL_VERSION, plugin_description, features \
    };
#endif

#if ! (JUCE_TEENSY || NE_PEDAL_TESTS)
#include "PluginEntry.h"
#define EXPORT_CLAP_PLUGIN_SYMBOLS(plugin)                        \
    JUCE_BEGIN_IGNORE_WARNINGS_GCC_LIKE ("-Wattributes")          \
    extern "C"                                                    \
    {                                                             \
        const CLAP_EXPORT struct clap_plugin_entry clap_entry = { \
            CLAP_VERSION,                                         \
            ne_pedal::plugins::plugin_entry::clap_init,           \
            ne_pedal::plugins::plugin_entry::clap_deinit,         \
            ne_pedal::plugins::plugin_entry::get_factory<         \
                ne_pedal::plugins::plugin>                        \
        };                                                        \
    }                                                             \
    JUCE_END_IGNORE_WARNINGS_GCC_LIKE
#else
#define EXPORT_CLAP_PLUGIN_SYMBOLS(plugin)
#endif
