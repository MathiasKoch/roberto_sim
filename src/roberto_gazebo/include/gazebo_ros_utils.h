/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef GAZEBO_ROS_UTILS_H
#define GAZEBO_ROS_UTILS_H
#include <map>
#include <boost/algorithm/string.hpp>
#include <boost/graph/graph_concepts.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Sensor.hh>
#include <ros/ros.h>

namespace gazebo
{
    
    
/**
* Accessing model name like suggested by nkoenig at http://answers.gazebosim.org/question/4878/multiple-robots-with-ros-plugins-sensor-plugin-vs/
* @param parent
* @return accessing model name
**/
inline std::string GetModelName ( const sensors::SensorPtr &parent )
{
    std::string modelName;
    std::vector<std::string> values;
    std::string scopedName = parent->ScopedName();
    boost::replace_all ( scopedName, "::", "," );
    boost::split ( values, scopedName, boost::is_any_of ( "," ) );
    if ( values.size() < 2 ) {
        modelName = "";
    } else {
        modelName = values[1];
    }
    return modelName;
}

/**
* @brief Reads the name space tag of a sensor plugin
* @param parent
* @param sdf
* @param pInfo
* @return node namespace
**/
inline std::string GetRobotNamespace ( const sensors::SensorPtr &parent, const sdf::ElementPtr &sdf, const char *pInfo = NULL )
{
    std::string name_space;
    std::stringstream ss;
    if ( sdf->HasElement ( "robotNamespace" ) ) {
        name_space = sdf->Get<std::string> ( "robotNamespace" );
        if ( name_space.empty() ) {
            ss << "the 'robotNamespace' param was empty";
            name_space = GetModelName ( parent );
        } else {
            ss << "Using the 'robotNamespace' param: '" <<  name_space << "'";
        }
    } else {
        ss << "the 'robotNamespace' param did not exit";
    }
//     if ( pInfo != NULL ) {
//         ROS_INFO ( "%s Plugin (robotNamespace = %s), Info: %s" , pInfo, name_space.c_str(), ss.str().c_str() );
//     }
    return name_space;
}

/**
 * Gazebo ros helper class
 * The class simplifies the parameter and rosnode handling
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 **/
class GazeboRos
{
private:
    sdf::ElementPtr base_sdf_;  /// base sdf
    sdf::ElementPtr sdf_;       /// sdf to read
    std::string plugin_;        /// name of the plugin class
    std::string namespace_;     /// name of the launched node
    boost::shared_ptr<ros::NodeHandle> rosnode_; /// rosnode
    std::string tf_prefix_;     /// prefix for the ros tf plublisher if not set it uses the namespace_
    std::string info_text;      /// info text for log messages to identify the node
    boost::mutex lock_;
    /**
     * Reads the common plugin parameters used by the constructor
     **/
    void readCommonParameter ();
public:
    /**
     * Constructor
     * @param _parent models parent
     * @param _sdf sdf to read
     * @param _name of the plugin class
     * @param _internalNS specifies weather the namespace is nested under /gazebo or not; defaults to false
     **/
    GazeboRos ( physics::ModelPtr &_parent, sdf::ElementPtr _sdf, const std::string &_plugin, bool _internalNS=false )
        : base_sdf_( _sdf ), sdf_ ( _sdf ), plugin_ ( _plugin ) {
        namespace_ = _parent->GetName ();
        if ( !sdf_->HasElement ( "robotNamespace" ) ) {
            ROS_INFO_NAMED (plugin_.c_str(), "%s missing <robotNamespace>, defaults is %s", plugin_.c_str(), namespace_.c_str() );
        }  else {
            namespace_ = sdf_->GetElement ( "robotNamespace" )->Get<std::string>();
            if ( namespace_.empty() ) { namespace_ = _parent->GetName(); }
        }
        if ( !namespace_.empty() ){ namespace_ += "/"; }
	if(_internalNS) { rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( "~/" + namespace_ ) ); }
	else            { rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle (        namespace_ ) ); }
        info_text = plugin_ + "(ns = " + namespace_ + ")";
        readCommonParameter ();
    }
    /**
     * Destructor
     **/
    virtual ~GazeboRos() {
	ROS_DEBUG_NAMED(plugin_.c_str(), "-------Destructed plugin %s!-------", plugin_.c_str());
	rosnode_->shutdown();
    }
    /**
     * Constructor
     * @param _parent sensor parent
     * @param _sdf sdf to read
     * @param _name of the plugin class
     **/
    GazeboRos ( sensors::SensorPtr _parent, sdf::ElementPtr _sdf, const std::string &_plugin, bool _internalNS=false  )
        : base_sdf_( _sdf ), sdf_ ( _sdf ), plugin_ ( _plugin ) {
	namespace_ = _parent->Name();
        if ( !sdf_->HasElement ( "robotNamespace" ) ) {
            ROS_INFO_NAMED(plugin_.c_str(), "%s missing <robotNamespace>, defaults is %s", plugin_.c_str(), namespace_.c_str() );
        }  else {
            namespace_ = sdf_->GetElement ( "robotNamespace" )->Get<std::string>();
            if ( namespace_.empty() ) { namespace_ = GetModelName(_parent); }
        }
        if ( !namespace_.empty() ){ namespace_ += "/"; }
	if(_internalNS) { rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( "~/" + namespace_ ) ); }
	else            { rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle (        namespace_ ) ); }
        info_text = plugin_ + "(ns = " + namespace_ + ")";
        readCommonParameter ();
    }

    /**
     * Returns in-use sdf data
     * @return in-use sdf data
     **/
    sdf::ElementPtr& Sdf() { return sdf_; }
    /**
     * Returns base sdf data
     * @return base sdf data
     **/
    sdf::ElementPtr& baseSdf() { return base_sdf_; }
    /**
     * Returns info text used for log messages
     * @return class name and node name as string
     **/
    const char* info() const;
    /**
     * returns the initialized created within the constuctor
     * @return rosnode
     **/
    boost::shared_ptr<ros::NodeHandle>& node();;
    /**
     * returns the initialized within the constuctor
     * @return rosnode
     **/
    const boost::shared_ptr<ros::NodeHandle>& node() const;
    /**
     * resolves a tf frame name by adding the tf_prefix initialized within the constuctor
     * @param _name
     * @retun resolved tf name
     **/
    std::string resolveTF ( const std::string &_name );

    /**
     * reads the follwoing _tag_name paramer or sets a _default value
     * @param _value
     * @param _tag_name
     * @param _default
     * @retun sdf tag value
     **/
    void getParameterBoolean ( bool &_value, const char *_tag_name, const bool &_default );
    /**
     * reads the follwoing _tag_name paramer
     * @param _value
     * @param _tag_name
     * @retun sdf tag value
     **/
    void getParameterBoolean ( bool &_value, const char *_tag_name );
    /**
     * retuns a JointPtr based on an sdf tag_name entry
     * @param _value
     * @param _tag_name
     * @param _joint_default_name
     * @retun JointPtr
     **/
    physics::JointPtr getJoint ( physics::ModelPtr &_parent, const char *_tag_name, const std::string &_joint_default_name );
    /**
     * checks if the ros not is initialized
     * @retun JointPtr
     **/
    void isInitialized();


    const std::string getPluginName() const;
    const std::string getNamespace() const;
    /**
     * reads the follwoing _tag_name paramer or sets a _default value
     * @param _value
     * @param _tag_name
     * @param _default
     * @retun sdf tag value
     **/
    template <class T>
    void getParameter ( T &_value, const char *_tag_name, const T &_default, bool _debugOutput = true  ) {
        _value = _default;
        if ( !sdf_->HasElement ( _tag_name ) ) {
            ROS_WARN_NAMED(plugin_.c_str(), "%s: missing <%s> default is %s", info(), _tag_name, boost::lexical_cast<std::string> ( _default ).c_str() );
        } else {
            getParameter<T> ( _value, _tag_name, _debugOutput );
        }
    }
    /**
     * reads the follwoing _tag_name paramer
     * @param _value
     * @param _tag_name
     * @param _default
     * @retun sdf tag value
     **/
    template <class T>
    void getParameter ( T &_value, const char *_tag_name, bool _debugOutput = true ) {
        if ( sdf_->HasElement ( _tag_name ) ) {
            _value = sdf_->GetElement ( _tag_name )->Get<T>();//TODO: should be Get<T>(_tag_name);
        }
        ROS_DEBUG_COND_NAMED(_debugOutput, plugin_.c_str(), "%s: <%s> = %s", info(), _tag_name, boost::lexical_cast<std::string> ( _value ).c_str() );
    }

    /**
     * reads the following _tag_name paramer and compares the value with an _options map keys and retuns the corresponding value.
     * @param _value
     * @param _tag_name
     * @param _default
     * @retun sdf tag value
     **/
    template <class T>
    void getParameter ( T &_value, const char *_tag_name, const std::map<std::string, T> &_options, const T &_default, bool _debugOutput = true ) {
        _value = _default;
        if ( !sdf_->HasElement ( _tag_name ) ) {
            ROS_WARN_NAMED(plugin_.c_str(), "%s: missing <%s> default is %s", info(), _tag_name, boost::lexical_cast<std::string> ( _default ).c_str() );
        } else {
            getParameter<T> ( _value, _tag_name, _options, _debugOutput );
        }
    }
    /**
     * reads the following _tag_name paramer and compares the value with an _options map keys and retuns the corresponding value.
     * @param _value
     * @param _tag_name
     * @param _default
     * @retun sdf tag value
     **/
    template <class T>
    void getParameter ( T &_value, const char *_tag_name, const std::map<std::string, T> &_options, bool _debugOutput = true ) {
        typename std::map<std::string, T >::const_iterator it;
        if ( sdf_->HasElement ( _tag_name ) ) {
            std::string value = sdf_->GetElement ( _tag_name )->Get<std::string>();
            it = _options.find ( value );
            if ( it == _options.end() ) {
                ROS_WARN_NAMED(plugin_.c_str(), "%s: <%s> no matching key to %s", info(), _tag_name, value.c_str() );
            } else {
                _value = it->second;
            }
        }
        ROS_DEBUG_COND_NAMED(_debugOutput,  plugin_.c_str(), "%s: <%s> = %s := %s",  info(), _tag_name, ( it == _options.end() ?"default":it->first.c_str() ), boost::lexical_cast<std::string> ( _value ).c_str() );
    }
};

typedef boost::shared_ptr<GazeboRos> GazeboRosPtr;
}
#endif


