// oc-add-ext: plugins/nodes/Ogn{{Name}}.cpp (cpp.ogn)
#include <Ogn{{Name}}Database.h>

namespace {{cpp_namespace}}
{

class Ogn{{Name}}
{
public:
    static bool compute(Ogn{{Name}}Database& db)
    {
        db.outputs.sum() = db.inputs.a() + db.inputs.b();
        return true;
    }
};

REGISTER_OGN_NODE()

} // namespace {{cpp_namespace}}
