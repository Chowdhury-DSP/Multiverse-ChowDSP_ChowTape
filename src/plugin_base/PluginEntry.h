#pragma once

#include <clap/clap.h>
#include <chowdsp_core/chowdsp_core.h>

JUCE_BEGIN_IGNORE_WARNINGS_GCC_LIKE ("-Wattributes")

namespace ne_pedal::plugins::plugin_entry
{
inline uint32_t clap_get_plugin_count (const clap_plugin_factory*) { return 1; }

template <typename PluginType>
const clap_plugin_descriptor*
    clap_get_plugin_descriptor (const clap_plugin_factory*, uint32_t)
{
    return &PluginType::description;
}

template <typename PluginType>
static const clap_plugin* clap_create_plugin (const clap_plugin_factory*,
                                              const clap_host* host,
                                              const char* plugin_id)
{
    if (strcmp (plugin_id, PluginType::description.id))
        return nullptr;

    // From baconpaul: I know it looks like a leak right? but the
    // clap-plugin-helpers basically take ownership and destroy the wrapper when
    // the host destroys the underlying plugin (look at Plugin<h, l>::clapDestroy
    // if you don't believe me!)
    auto p = new PluginType (host);
    return p->clapPlugin();
}

template <typename PluginType>
const CLAP_EXPORT struct clap_plugin_factory plugin_factory = {
    clap_get_plugin_count,
    clap_get_plugin_descriptor<PluginType>,
    clap_create_plugin<PluginType>,
};

template <typename PluginType>
static const void* get_factory (const char* /*factory_id*/)
{
    return &plugin_factory<PluginType>;
}

// clap_init and clap_deinit are required to be fast, but we have nothing we
// need to do here
inline bool clap_init (const char*) { return true; }
inline void clap_deinit() {}

} // namespace ne_pedal::plugins::plugin_entry

JUCE_END_IGNORE_WARNINGS_GCC_LIKE
