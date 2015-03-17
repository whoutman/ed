#ifndef entity_h_
#define entity_h_

#include "ed/types.h"
#include "ed/uuid.h"

#include <tue/config/data.h>

#include <boost/circular_buffer.hpp>

#include "ed/property.h"
#include "ed/property_key.h"

namespace ed
{

class Entity
{

public:
    Entity(const UUID& id = generateID(), const TYPE& type = "");
    ~Entity();

    static UUID generateID();
    const UUID& id() const { return id_; }

    const TYPE& type() const { return type_; }
    void setType(const TYPE& type) { type_ = type; }

    inline const tue::config::DataConstPointer& data() const { return config_; }
    inline void setData(const tue::config::DataConstPointer& data) { config_ = data; }

    inline void setRelationTo(Idx child_idx, Idx r_idx) { relations_to_[child_idx] = r_idx; }
    inline void setRelationFrom(Idx parent_idx, Idx r_idx) { relations_from_[parent_idx] = r_idx; }

    //! What do we want this, where is a relation defined? (Rein)
    inline Idx relationTo(Idx child_idx) const
    {
        std::map<Idx, Idx>::const_iterator it = relations_to_.find(child_idx);
        if (it == relations_to_.end())
            return INVALID_IDX;
        return it->second;
    }

    inline Idx relationFrom(Idx parent_idx) const
    {
        std::map<Idx, Idx>::const_iterator it = relations_from_.find(parent_idx);
        if (it == relations_from_.end())
            return INVALID_IDX;
        return it->second;
    }

    const std::map<Idx, Idx>& relationsFrom() const { return relations_from_; }

    const std::map<Idx, Idx>& relationsTo() const { return relations_to_; }

    template<typename T>
    const T* property(const PropertyKey<T>& key) const
    {
        std::map<Idx, Property>::const_iterator it = properties_.find(key.idx);
        if (it == properties_.end())
            return 0;

        const Property& p = it->second;

        try
        {
            return &p.value.getValue<T>();
        }
        catch (std::bad_cast& e)
        {
            return 0;
        }
    }

    template<typename T>
    void setProperty(const PropertyKey<T>& key, const T& t)
    {
        if (!key.valid())
            return;

        std::map<Idx, Property>::iterator it = properties_.find(key.idx);
        if (it == properties_.end())
        {
            Property& p = properties_[key.idx];
            p.entry = key.entry;
            p.revision = 0;
            p.value.setValue(t);
        }
        else
        {
            Property& p = it->second;
            p.value.setValue(t);
            ++(p.revision);
        }
    }

    void setProperty(Idx idx, const Property& p)
    {
        std::map<Idx, Property>::iterator it = properties_.find(idx);
        if (it == properties_.end())
        {
            Property& p_new = properties_[idx];
            p_new.entry = p.entry;
            p_new.revision = 0;
            p_new.value = p.value;
        }
        else
        {
            Property& p_new = it->second;
            p_new.value = p.value;
            ++(p_new.revision);
        }
    }

    const std::map<Idx, Property>& properties() const { return properties_; }

private:

    UUID id_;

    TYPE type_;

    tue::config::DataConstPointer config_;

    //! What do we want this, where is a relation defined? (Rein)
    std::map<Idx, Idx> relations_from_;
    std::map<Idx, Idx> relations_to_;

    // Generic property map
    std::map<Idx, Property> properties_;

    void updateConvexHull();

};

}

#endif
