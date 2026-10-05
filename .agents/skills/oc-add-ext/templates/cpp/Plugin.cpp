// oc-add-ext: plugins/{{ext_id}}/{{Name}}Plugin.cpp (cpp.ext or cpp.api)
#define CARB_EXPORTS

#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>
[if cpp.ext]
#include <omni/ext/IExt.h>
[end]
[if cpp.api]
#include <{{module_path}}/I{{Name}}.h>
[end]
[if cpp.ogn]
#include <omni/graph/core/ogn/Registration.h>
[end]

const struct carb::PluginImplDesc kPluginImpl = { "{{ext_id}}.plugin", "{{description}}", "{{org}}",
                                                  carb::PluginHotReload::eDisabled, "dev" };
[if cpp.ogn]

DECLARE_OGN_NODES()
[end]

namespace {{cpp_namespace}}
{
[if cpp.api]

double add(double a, double b)
{
    return a + b;
}
[end]
[if cpp.ext]

class Extension : public omni::ext::IExt
{
public:
    void onStartup(const char* extId) override
    {
        CARB_LOG_INFO("[%s] startup", extId);
[if cpp.ogn]
        INITIALIZE_OGN_NODES()
[end]
    }

    void onShutdown() override
    {
[if cpp.ogn]
        RELEASE_OGN_NODES()
[end]
        CARB_LOG_INFO("[{{ext_id}}] shutdown");
    }
};
[end]

} // namespace {{cpp_namespace}}

// oc-add-ext: keep only the chosen classes in this list: Extension (cpp.ext), I{{Name}} (cpp.api).
CARB_PLUGIN_IMPL(kPluginImpl, {{cpp_namespace}}::Extension, {{cpp_namespace}}::I{{Name}})
CARB_PLUGIN_IMPL_NO_DEPS()
[if cpp.ext]

void fillInterface({{cpp_namespace}}::Extension& iface)
{
}
[end]
[if cpp.api]

void fillInterface({{cpp_namespace}}::I{{Name}}& iface)
{
    iface.add = {{cpp_namespace}}::add;
}
[end]
