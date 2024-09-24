#include "Parameters.h"
#include <algorithm>
#include <cstring>

namespace ne_pedal::plugins
{
FloatParameter::FloatParameter (clap_id param_id,
                                std::string_view param_name,
                                NormalisableRange<float>&& param_range,
                                float def_value,
                                std::string_view suffix)
    : Parameter (param_id, std::move (param_name)),
      range (std::move (param_range)),
      default_value (def_value),
      current_value_with_modulation (default_value),
      current_value_01 (range.convertTo0to1 (default_value))
{
    valueToText = [this, suffix] (float x)
    {
        return [] (float x, int precision) -> std::string
        {
            return std::to_string (x).substr (0, std::to_string (x).find ('.') + precision + 1);
        }(range.convertFrom0to1 (x), 6) + std::string { suffix };
    };

    textToValue = [this] (const std::string& x)
    {
        return range.convertTo0to1 (std::stof (x));
    };
}

void FloatParameter::get_info (clap_param_info* info)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
    info->id = id;
    info->cookie = this;
    strncpy (info->name, name.data(), CLAP_NAME_SIZE);
#pragma GCC diagnostic pop

#if ! NE_PEDAL_MULTIVERSE
    info->flags = CLAP_PARAM_IS_AUTOMATABLE | CLAP_PARAM_IS_MODULATABLE;
#endif

    info->min_value = 0.0;
    info->max_value = 1.0;
    info->default_value = (double) range.convertTo0to1 (default_value);
}

void FloatParameter::set_value (float new_value_01)
{
    current_value_01 = new_value_01;
    compute_value_with_mod();
}

void FloatParameter::apply_modulation (float modulation)
{
    current_modulation = modulation;
    compute_value_with_mod();
}

void FloatParameter::compute_value_with_mod()
{
    current_value_with_modulation = range.convertFrom0to1 (current_value_01 + current_modulation);
}
} // namespace ne_pedal::plugins
