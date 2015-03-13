#include "ed/entity.h"

#include <geolib/Shape.h>
#include <geolib/Mesh.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Entity::Entity(const UUID& id, const TYPE& type) :
    id_(id),
    type_(type)
{
}

// ----------------------------------------------------------------------------------------------------

Entity::~Entity()
{
}

// ----------------------------------------------------------------------------------------------------

UUID Entity::generateID() {
    static const char alphanum[] =
        "0123456789"
        "abcdef";

    std::string s;
    for (int i = 0; i < 32; ++i) {
        int n = rand() / (RAND_MAX / (sizeof(alphanum) - 1) + 1);
        s += alphanum[n];
    }

    return UUID(s);
}

}
