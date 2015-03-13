#ifndef ED_UPDATE_REQUEST_H_
#define ED_UPDATE_REQUEST_H_

#include "ed/types.h"
#include "ed/uuid.h"
#include "ed/property.h"
#include "ed/property_key.h"

#include <tue/config/data_pointer.h>

#include <map>
#include <vector>
#include <geolib/datatypes.h>

namespace ed
{

class UpdateRequest
{

public:

    UpdateRequest() : empty_(true) {}

    // TYPES

    std::map<UUID, std::string> types;
    void setType(const UUID& id, const std::string& type) { types[id] = type; empty_ = false; }

    // RELATIONS

    std::map<UUID, std::map<UUID, RelationConstPtr> > relations;
    void setRelation(const UUID& id1, const UUID& id2, const RelationConstPtr& r) { relations[id1][id2] = r; empty_ = false; }

    // DATA

    std::map<UUID, tue::config::DataConstPointer> datas;

    void addData(const UUID& id, const tue::config::DataConstPointer& data)
    {
        std::map<UUID, tue::config::DataConstPointer>::iterator it = datas.find(id);
        if (it == datas.end())
        {
            datas[id] = data;
        }
        else
        {
            tue::config::DataPointer data_total;
            data_total.add(it->second);
            data_total.add(data);

            it->second = data_total;
        }

        empty_ = false;
    }

    std::map<UUID, std::map<Idx, Property> > properties;

    template<typename T>
    void setProperty(const UUID& id, const PropertyKey<T>& key, const T& value)
    {
        if (!key.valid())
            return;

        Property& p = properties[id][key.idx];
        p.info = key.info;
        p.value = value;
        empty_ = false;
    }


    // REMOVED ENTITIES

    std::set<UUID> removed_entities;

    void removeEntity(const UUID& id) { removed_entities.insert(id); empty_ = false; }

    bool empty() const { return empty_; }

private:

    bool empty_;

};

}

#endif
