-- oc-add-ext: premake5.lua (any.cpp or cpp.tests)
local ext = get_current_extension_info()
project_ext(ext)

repo_build.prebuild_link {
    { "docs", ext.target_dir.."/docs" },
[if any.py]
    { "{{org}}", ext.target_dir.."/{{org}}" },
[end]
}
[if cpp.ogn]

local ogn = get_ogn_project_information(ext, "{{module_path}}")
project_ext_ogn(ext, ogn)
[end]
[if any.cpp]

project_ext_plugin(ext, ext.id..".plugin")
    cppdialect "C++17"
    add_files("impl", "plugins/"..ext.id)
    includedirs { "include", "plugins/"..ext.id }
[if cpp.ogn]
    add_files("nodes", "plugins/nodes")
    add_ogn_dependencies(ogn, { "plugins/nodes" })
[end]
[end]
[if cpp.pybind]

project_ext_bindings {
    ext = ext,
    project_name = ext.id..".python",
    module = "_{{ext_name}}",
    src = "bindings/python/"..ext.id,
    target_subdir = "{{module_path}}",
}
    cppdialect "C++17"
    includedirs { "include" }
[end]
[if cpp.tests]

project_ext_tests(ext, ext.id..".tests")
    cppdialect "C++17"
    add_files("impl", "tests")
    includedirs { "include" }
    externalincludedirs { "%{target_deps}/doctest/include" }
[end]
