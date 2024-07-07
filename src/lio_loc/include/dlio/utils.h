#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

namespace dlio {

template <typename T>
struct identity {
    typedef T type;
};

template <typename T>
void declare_param(ros::NodeHandle* node, 
                   const std::string param_name, 
                   T& param, 
                   const typename identity<T>::type& default_value) {
    node->setParam(param_name, default_value);
    node->getParam(param_name, param);
    ROS_INFO_STREAM(param_name << "=" << param);
}

template <typename T>
void declare_param_vector(ros::NodeHandle* node, 
                          const std::string param_name, 
                          T& param, 
                          const typename identity<T>::type& default_value) {
    node->setParam(param_name, default_value);
    node->getParam(param_name, param);
    for (int i = 0; i < param.size(); i++) {
        ROS_INFO_STREAM(param_name << "[" << i <<"]" << "=" <<param[i]);
    }
}

template <typename T> 
T yaml_get_value(const YAML::Node& node, const std::string& key) {
    try {
        return node[key].as<T>();
    } catch (YAML::Exception& e) {
        std::stringstream ss;
        ss << "Failed to parse YAML tag " << key << "for reason:" << e.msg;
        throw YAML::Exception(e.mark, ss.str());
    }
}
}