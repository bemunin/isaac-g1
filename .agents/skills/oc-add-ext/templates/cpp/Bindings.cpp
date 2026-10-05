// oc-add-ext: bindings/python/{{ext_id}}/{{Name}}Bindings.cpp (cpp.pybind)
#include <carb/BindingsPythonUtils.h>

#include <{{module_path}}/I{{Name}}.h>

CARB_BINDINGS("{{ext_id}}.python")

PYBIND11_MODULE(_{{ext_name}}, m)
{
    using namespace {{cpp_namespace}};

    m.doc() = "Python bindings for I{{Name}}.";

    carb::defineInterfaceClass<I{{Name}}>(m, "I{{Name}}", "acquire_{{ext_name}}_interface",
                                         "release_{{ext_name}}_interface")
        .def("add", carb::wrapInterfaceFunction(&I{{Name}}::add));
}
