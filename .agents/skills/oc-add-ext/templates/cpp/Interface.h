// oc-add-ext: include/{{module_path}}/I{{Name}}.h (cpp.api)
#pragma once

#include <carb/Interface.h>

namespace {{cpp_namespace}}
{

/// Example Carbonite interface; acquire with carb::getCachedInterface<I{{Name}}>().
struct I{{Name}}
{
    CARB_PLUGIN_INTERFACE("{{cpp_namespace}}::I{{Name}}", 1, 0)

    double(CARB_ABI* add)(double a, double b);
};

} // namespace {{cpp_namespace}}
