#ifndef ED_SERVER_H_
#define ED_SERVER_H_

#include "ed/types.h"

#include <tue/config/configuration.h>

#include <queue>

namespace ed
{

class Server
{

public:
    Server();
    virtual ~Server();    

    void configure(tue::Configuration& config, bool reconfigure = false);

    void initialize();

    void reset();

    void update();

    void update(const std::string& update_str, std::string& error);

//    int size() const { return entities_.size(); }

//    const std::map<UUID, EntityConstPtr>& entities() const { return entities_; }

    WorldModelConstPtr world_model() const { return world_model_; }

    void addPluginPath(const std::string& path) { plugin_paths_.push_back(path); }

    PluginContainerPtr loadPlugin(const std::string& plugin_name, const std::string& lib_file, tue::Configuration config);

    void stepPlugins();

    void publishStatistics() const;

private:

    // World model datastructure
    WorldModelConstPtr world_model_;

    std::queue<UpdateRequest> update_requests_;

    void initializeWorld();

    //! Plugins
    std::vector<std::string> plugin_paths_;
    std::vector<PluginContainerPtr> plugin_containers_;

    std::string getFullLibraryPath(const std::string& lib);
};

}

#endif
