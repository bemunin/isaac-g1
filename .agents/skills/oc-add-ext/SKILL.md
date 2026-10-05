---
name: oc-add-ext
description: Scaffold a new Isaac Sim / Omniverse Kit extension (Python or C++), like the Isaac Sim VS Code "Template → Extension" form. Use when asked to create, add, or scaffold a Kit or Isaac Sim extension.
license: MIT
---

# oc-add-ext

Create a Kit extension from [`templates/`](templates/), mirroring the VS Code "Create extension" form.

## 1. Detect the project

- **Root**: `git rev-parse --show-toplevel`, else the working directory.
- **Extensions folder**: `<root>/sim/exts` if `<root>/sim/` exists, else `<root>/exts`.
- **Organization hint**: the first segment shared by every folder in the extensions folder, if any.
- **Kit app template**: `premake5.lua` plus `deps/` or `tools/deps/kit-sdk-deps.packman.xml` exist.

## 2. Collect the form

The user fills the form; never fill a field they have not given or accepted. Ask only for what is
missing, in these rounds:

1. **Ext. name**: `<organization>.<group>.<ext_name>`, three lowercase `[a-z0-9_]` segments. If it is
   invalid, ask again and name the broken rule. Suggest the organization hint.
2. **Language**: Python, C++, or both. No default. If C++ is chosen and there is no Kit app template,
   warn that C++ will be source-only.
3. **Components** of the chosen language(s) only. No default; at least one per chosen language. Ask
   one multi-select per language; Tests is an option in it like any other component, never a separate
   question. Make each component its own option (never merge two, e.g. UI and Tests). Split a language
   across questions when a menu caps options.

   | Component | Python | C++ |
   |---|---|---|
   | Extension class (startup/shutdown) | `py.ext` | `cpp.ext` |
   | API (functions / carb interface) | `py.api` | `cpp.api` |
   | OmniGraph node | `py.ogn` | `cpp.ogn` |
   | Python binding (pybind11 over C++ API) | — | `cpp.pybind` |
   | UI (omni.ui window) | `py.ui` | — |
   | Tests | `py.tests` | `cpp.tests` |

   Add the implied components and tell the user: `py.ui` → `py.ext`, `cpp.pybind` → `cpp.api`, `cpp.ogn` → `cpp.ext`.
4. **Metadata** (plus Ext. path): show one block headed `Create extension: <ext_id>`. Start it with
   the settled language and components, then one `field: value` line per field below, using the value
   the user gave or else the suggested default (`(omitted)` when there is none). Then ask one question:
   `Use these` | `Edit`. On Edit, the user replies in chat with only the changes, e.g.
   `title: ROS DDS, category: Robotics`. Apply them, keep the rest, and do not ask again unless a value
   is invalid (License must be None or MIT). Skip the question if the user already gave every field.

   | Field | Suggested default | Hint |
   |---|---|---|
   | Ext. path | the extensions folder; `<root>/source/extensions` if any C++ and a Kit app template | Folder holding the extension; Kit loads it only from its extension search path |
   | Title | `<ext_name>` in Title Case | Display name in the Extension Manager |
   | Description | `"<Title> extension."` | One-line summary in the Extension Manager |
   | Keywords | `["<group>", "<ext_name>"]` | Extension Manager search terms |
   | Authors | `["<git user.name> <<git user.email>>"]`; omit if unset | List of `"Name <email>"` |
   | Repository | `git remote get-url origin` as https; omit if none | Source URL shown in the Extension Manager |
   | Category | undefined (omitted) | Extension Manager group, e.g. Simulation, Rendering, Robotics, Utility |
   | License | None | None = no LICENSE file; MIT = LICENSE, holder git user.name |

When the form is complete: if `<path>/<ext_id>` exists, stop and ask. Otherwise go straight to step 3,
without confirming.

## 3. Render the templates

Placeholders for the ID `org.group.ext_name` (e.g. `omnicraft.bench.ros_pub`): `{{ext_id}}`,
`{{org}}` (`omnicraft`), `{{module_path}}` (`omnicraft/bench/ros_pub`), `{{cpp_namespace}}`
(`omnicraft::bench::ros_pub`), `{{ext_name}}` (`ros_pub`), `{{Name}}` (`RosPub`), `{{title}}`,
`{{description}}`, `{{category}}`, `{{repository}}`, `{{keywords}}` and `{{authors}}` (TOML arrays),
`{{date}}` (today, `YYYY-MM-DD`), `{{year}}`, `{{copyright_holder}}`.

Markup:
- The first line of each template is `oc-add-ext: <destination> (<condition>)`. The condition is
  `always`, `license = MIT`, or keys joined by `or`. Write every template whose condition holds to
  `<path>/<ext_id>/<destination>`, and nothing else. Template file names (and `.tmpl`) do not matter.
- Files in `templates/data/` (`icon.png`, `preview.png`) are binary and have no first line: always
  copy each one unchanged to `<path>/<ext_id>/data/<same name>`.
- Any line containing `oc-add-ext:` is an instruction: apply it, then delete the line.
- Keep `[if <key>]` … `[end]` blocks (they may nest) only when `<key>` holds; always delete the
  marker lines.
- Union keys: `any.py` (any `py.*`, `cpp.pybind`, `cpp.ogn`), `any.cpp` (any `cpp.*` but
  `cpp.tests`), `any.ogn`, `any.tests`.

Done when `grep -rnI 'oc-add-ext:\|{{\|\[if \|\[end\]' <ext dir>` prints nothing and
`python3 -c "import tomllib,sys; tomllib.load(open(sys.argv[1],'rb'))" <ext dir>/config/extension.toml` succeeds.

## 4. Report

Print the form as applied and the created tree. Then:
- Grep the project's launchers and `.kit` files (`--ext-folder`, `exts.folders`) for `<path>`, without
  editing them. If it is missing, print `--ext-folder <path> --enable <ext_id>`.
- Python tests: Extension Manager → the extension → Tests tab.
- Any C++: it builds only inside a Kit-SDK-based app (Isaac Sim open-source app or Kit App Template).
  The templates are unbuilt starting points. If it landed in the extensions folder, say to move it to a Kit app's
  `source/extensions/`. Print the `kit-sdk-deps.packman.xml` lines it needs: `usd` for `cpp.ogn`,
  `doctest` for `cpp.tests`:
  ```xml
  <import path="...all-deps.packman.xml">
      <filter include="usd-${config}"/>   <!-- cpp.ogn -->
      <filter include="doctest"/>         <!-- cpp.tests -->
  </import>
  <dependency name="usd-${config}" linkPath="../_build/target-deps/usd/${config}"/>
  <dependency name="doctest" linkPath="../_build/target-deps/doctest"/>
  ```
