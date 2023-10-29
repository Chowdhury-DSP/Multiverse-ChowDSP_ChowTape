#pragma once

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#if NE_PEDAL_MULTIVERSE
using clap_id = uint32_t;
static constexpr size_t CLAP_NAME_SIZE = 256;

struct clap_param_info {
   // Stable parameter identifier, it must never change.
   clap_id id;

   // This value is optional and set by the plugin.
   // Its purpose is to provide fast access to the plugin parameter object by caching its pointer.
   // For instance:
   //
   // in clap_plugin_params.get_info():
   //    Parameter *p = findParameter(param_id);
   //    param_info->cookie = p;
   //
   // later, in clap_plugin.process():
   //
   //    Parameter *p = (Parameter *)event->cookie;
   //    if (!p) [[unlikely]]
   //       p = findParameter(event->param_id);
   //
   // where findParameter() is a function the plugin implements to map parameter ids to internal
   // objects.
   //
   // Important:
   //  - The cookie is invalidated by a call to clap_host_params->rescan(CLAP_PARAM_RESCAN_ALL) or
   //    when the plugin is destroyed.
   //  - The host will either provide the cookie as issued or nullptr in events addressing
   //    parameters.
   //  - The plugin must gracefully handle the case of a cookie which is nullptr.
   //  - Many plugins will process the parameter events more quickly if the host can provide the
   //    cookie in a faster time than a hashmap lookup per param per event.
   void *cookie;

   // The display name. eg: "Volume". This does not need to be unique. Do not include the module
   // text in this. The host should concatenate/format the module + name in the case where showing
   // the name alone would be too vague.
   char name[CLAP_NAME_SIZE];

   double min_value;     // Minimum plain value
   double max_value;     // Maximum plain value
   double default_value; // Default plain value
};
#else
#include <clap/ext/params.h>
#endif

namespace ne_pedal::plugins
{
class Parameter
{
public:
    Parameter (clap_id param_id, std::string&& param_name)
        : id (param_id), name (std::move (param_name)) {}
    virtual ~Parameter() = default;

    [[nodiscard]] virtual float get() const noexcept = 0;
    virtual void get_info (clap_param_info* /*info*/) = 0;
    virtual void set_value (float new_value) = 0;

    virtual bool supports_modulation() const noexcept { return false; }
    virtual void apply_modulation (float /*modulation*/) {};

    std::function<std::string (float)> valueToText = nullptr;
    std::function<float (const std::string&)> textToValue = nullptr;

protected:
    const clap_id id;
    const std::string name;
};

/** Based on juce::NormalisableRange */
template <typename ValueType>
class NormalisableRange
{
public:
    NormalisableRange (const NormalisableRange&) = default;
    NormalisableRange& operator= (const NormalisableRange&) = default;
    NormalisableRange (NormalisableRange&&) noexcept = default;
    NormalisableRange& operator= (NormalisableRange&&) noexcept = default;

    /** Creates a NormalisableRange with a given range, interval and skew factor. */
    NormalisableRange (ValueType rangeStart,
                       ValueType rangeEnd,
                       ValueType centerValue) noexcept
        : start (rangeStart), end (rangeEnd), skew (std::log (static_cast<ValueType> (0.5)) / std::log ((centerValue - start) / (end - start)))
    {
    }

    /** Creates a NormalisableRange with a given range, continuous interval, but a dummy skew-factor. */
    NormalisableRange (ValueType rangeStart,
                       ValueType rangeEnd) noexcept
        : start (rangeStart), end (rangeEnd)
    {
    }

    /** Uses the properties of this mapping to convert a non-normalised value to
        its 0->1 representation.
    */
    ValueType convertTo0to1 (ValueType v) const noexcept
    {
        auto proportion = clampTo0To1 ((v - start) / (end - start));

        if (skew == static_cast<ValueType> (1))
            return proportion;

        return std::pow (proportion, skew);
    }

    /** Uses the properties of this mapping to convert a normalised 0->1 value to
        its full-range representation.
    */
    ValueType convertFrom0to1 (ValueType proportion) const noexcept
    {
        proportion = clampTo0To1 (proportion);

        if (skew != static_cast<ValueType> (1) && proportion > ValueType())
            proportion = std::exp (std::log (proportion) / skew);

        return start + (end - start) * proportion;
    }

private:
    static ValueType clampTo0To1 (ValueType value)
    {
        return std::clamp (value, static_cast<ValueType> (0), static_cast<ValueType> (1));
    }

    const ValueType start = 0;
    const ValueType end = 1;
    const ValueType skew = 1;
};

class FloatParameter : public Parameter
{
public:
    FloatParameter (clap_id param_id, std::string&& param_name, NormalisableRange<float>&& range, float default_value, const std::string& suffix = {});

    [[nodiscard]] float get() const noexcept override { return current_value_01; }
    void get_info (clap_param_info* /*info*/) override;
    void set_value (float new_value_01) override;

    bool supports_modulation() const noexcept override { return true; }
    void apply_modulation (float modulation) override;

    operator float() const noexcept // NOLINT(google-explicit-constructor)
    {
        return current_value_with_modulation;
    }

private:
    void compute_value_with_mod();

    const float default_value;

    float current_modulation = 0.0f;
    float current_value_with_modulation;

    const NormalisableRange<float> range;
    float current_value_01;
};
} // namespace ne_pedal::plugins
