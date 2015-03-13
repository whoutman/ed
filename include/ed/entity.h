#ifndef entity_h_
#define entity_h_

#include "ed/types.h"
#include "ed/uuid.h"

#include <tue/config/data.h>

#include <boost/circular_buffer.hpp>

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

    //! This should be the blackboard, should we also include the type, I think not (Rein)
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

private:

    UUID id_;

    TYPE type_;

    tue::config::DataConstPointer config_;

    //! What do we want this, where is a relation defined? (Rein)
    std::map<Idx, Idx> relations_from_;
    std::map<Idx, Idx> relations_to_;

};

}

#endif
