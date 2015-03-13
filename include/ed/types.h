#ifndef ED_TYPES_H_
#define ED_TYPES_H_

//#include <rgbd/types.h>
#include <geolib/datatypes.h>

#include <boost/shared_ptr.hpp>

namespace ed
{

typedef unsigned int Idx;
static const unsigned int INVALID_IDX = -1;

class Entity;
typedef boost::shared_ptr<Entity> EntityPtr;
typedef boost::shared_ptr<const Entity> EntityConstPtr;

class Plugin;
typedef boost::shared_ptr<Plugin> PluginPtr;
typedef boost::shared_ptr<const Plugin> PluginConstPtr;

class WorldModel;
typedef boost::shared_ptr<WorldModel> WorldModelPtr;
typedef boost::shared_ptr<const WorldModel> WorldModelConstPtr;

class UpdateRequest;
typedef boost::shared_ptr<UpdateRequest> UpdateRequestPtr;
typedef boost::shared_ptr<const UpdateRequest> UpdateRequestConstPtr;

class PluginContainer;
typedef boost::shared_ptr<PluginContainer> PluginContainerPtr;
typedef boost::shared_ptr<const PluginContainer> PluginContainerConstPtr;

class Relation;
typedef boost::shared_ptr<Relation> RelationPtr;
typedef boost::shared_ptr<const Relation> RelationConstPtr;

class UUID;

typedef std::string TYPE;

}

#endif
