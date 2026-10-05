// oc-add-ext: tests/Test{{Name}}.cpp (cpp.tests)
#include <doctest/doctest.h>
[if cpp.api]

#include <carb/InterfaceUtils.h>
#include <{{module_path}}/I{{Name}}.h>
[end]

TEST_SUITE("{{ext_id}}")
{
    TEST_CASE("sanity")
    {
        CHECK(1 + 1 == 2);
    }
[if cpp.api]

    TEST_CASE("add")
    {
        auto iface = carb::getCachedInterface<{{cpp_namespace}}::I{{Name}}>();
        REQUIRE(iface != nullptr);
        CHECK(iface->add(2.0, 3.0) == 5.0);
    }
[end]
}
