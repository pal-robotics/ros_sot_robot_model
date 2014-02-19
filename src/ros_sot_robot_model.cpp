/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2010, CNRS
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
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
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
 */

/** \author Gennaro Raiola
 *   inspired on the RosRobotModel class written by Thomas Moulard, available here http://wiki.ros.org/dynamic_graph_bridge#RosRobotModel.
 */

# include <ros/ros.h>
# include <dynamic-graph/all-commands.h>
# include <dynamic-graph/factory.h>
# include "dynamic_graph_bridge/ros_init.hh"
# include <ros_sot_robot_model/ros_sot_robot_model.hh>

#include <urdf/model.h>

# include "tf/transform_listener.h"
# include "tf/message_filter.h"

namespace
{
// Function taken from the parser
matrix4d normalizeFrameOrientation (jrl::dynamics::urdf::Parser::UrdfJointConstPtrType urdfJoint)
{
    if (!urdfJoint)
        throw std::runtime_error
            ("invalid joint in normalizeFrameOrientation");
    matrix4d result;
    result.setIdentity ();

    vector3d x (urdfJoint->axis.x,
                urdfJoint->axis.y,
                urdfJoint->axis.z);
    x.normalize ();

    vector3d y (0., 0., 0.);
    vector3d z (0., 0., 0.);

    unsigned smallestComponent = 0;
    for (unsigned i = 0; i < 3; ++i)
        if (std::fabs(x[i]) < std::fabs(x[smallestComponent]))
            smallestComponent = i;

    y[smallestComponent] = 1.;
    z = x ^ y;
    y = z ^ x;
    // (x, y, z) is an orthonormal basis.

    for (unsigned i = 0; i < 3; ++i)
    {
        result (i, 0) = x[i];
        result (i, 1) = y[i];
        result (i, 2) = z[i];
    }

    return result;
}

vectorN convertVector(const ml::Vector& v)
{
    vectorN res (v.size());
    for (unsigned i = 0; i < v.size(); ++i)
        res[i] = v(i);
    return res;
}

ml::Vector convertVector(const vectorN& v)
{
    ml::Vector res;
    res.resize(v.size());
    for (unsigned i = 0; i < v.size(); ++i)
        res(i) = v[i];
    return res;
}

int sign(double value){
    int res;

    if(value==0)
        res = 0;
    if(value>0)
        res = 1;
    if(value<0)
        res = -1;
	
    return res;
}

ml::Vector mat2quat(const matrix4d& mat){

      ml::Vector  quat;
      quat.resize(4);
      quat(3) = 0.5 * std::sqrt(mat(0,0)+mat(1,1)+mat(2,2)+1);
      quat(0) = 0.5 * sign(mat(2,1)-mat(1,2))*std::sqrt(mat(0,0)-mat(1,1)-mat(2,2)+1);
      quat(1) = 0.5 * sign(mat(0,2)-mat(2,0))*std::sqrt(mat(1,1)-mat(2,2)-mat(0,0)+1);
      quat(2) = 0.5 * sign(mat(1,0)-mat(0,1))*std::sqrt(mat(2,2)-mat(0,0)-mat(1,1)+1);
      return quat;
}

} // end of anonymous namespace.



namespace dynamicgraph
{

RosSotRobotModel::RosSotRobotModel(const std::string& name)
    : Dynamic(name,false),
      pn_("jrl_joints_list"),
      ns_("sot_controller")
{

    std::string docstring;

    docstring =
            "\n"
            "  Load the robot model from the parameter server.\n"
            "\n"
            "  This is the recommended method.\n"
            "\n";
    addCommand("loadFromParameterServer", command::makeCommandVoid0(*this,&RosSotRobotModel::loadFromParameterServer,docstring));

    docstring =
            "\n"
            "  Load the robot model from an URDF file.\n"
            "\n";
    addCommand("loadUrdf", command::makeCommandVoid1(*this,&RosSotRobotModel::loadUrdf,docstring));

    docstring =
            "\n"
            "  Set the namespace."
            "\n";
    addCommand("setNamespace", command::makeCommandVoid1(*this,&RosSotRobotModel::setNamespace,docstring));

    docstring =
            "\n"
            "  Set the parameter name."
            "\n";
    addCommand("setParameterName", command::makeCommandVoid1(*this,&RosSotRobotModel::setNamespace,docstring));

    docstring =
            "\n"
            "  Get current configuration of the robot.\n"
            "\n";
    addCommand ("curConf", new command::Getter<RosSotRobotModel,Vector> (*this,&RosSotRobotModel::curConf,docstring));

    parser_.reset(new jrl::dynamics::urdf::Parser());
}

RosSotRobotModel::~RosSotRobotModel()
{}

void RosSotRobotModel::loadUrdf (const std::string& filename)
{

    m_HDR = parser_->parse(filename);
    setParamsJointNames();
    setParamsJointOrientations();
}

void RosSotRobotModel::setNamespace (const std::string& ns)
{
    ns_ = ns;
}

void RosSotRobotModel::setParameterName (const std::string& pn)
{
    pn_ = pn;
}

void RosSotRobotModel::loadFromParameterServer()
{
    rosInit (false);
    std::string robotDescription;
    ros::param::param<std::string> (ns_ + "/robot_description", robotDescription, "");

    if (robotDescription.empty ())
        throw std::runtime_error("No model available as ROS parameter. Fail.");

    m_HDR = parser_->parseStream(robotDescription);


    setParamsJointNames();
    setParamsJointOrientations();

}

void RosSotRobotModel::setParamsJointNames(){

    // Load a list of joints ordered by rank
    std::vector<CjrlJoint*> tmp_jv = m_HDR->jointVector();
    std::vector<CjrlJoint*> actJointsVect = parser_->actuatedJoints();
    for (unsigned int i=0;i<tmp_jv.size();i++)
        if (std::find(actJointsVect.begin(), actJointsVect.end(),tmp_jv[i])!=actJointsVect.end())
            jointNames_[tmp_jv[i]->rankInConfiguration()-6] = tmp_jv[i]->getName();

    ros::NodeHandle nh(ns_);
    nh.setParam(pn_, jointNames_);
}

void RosSotRobotModel::setParamsJointOrientations(){

    boost::shared_ptr< ::urdf::ModelInterface> model = parser_->urdfModel();

    ros::NodeHandle nh(ns_);

    for (int i = 0; i < jointNames_.size(); ++i)
    {
        XmlRpc::XmlRpcValue &name_value = jointNames_[i];
        const std::string joint_name = static_cast<std::string>(name_value);
        std::map<std::string, boost::shared_ptr< ::urdf::Joint>  >::const_iterator it;
        it = model->joints_.find(joint_name);
        if (it->second){
            matrix4d compMatrix = normalizeFrameOrientation(model->getJoint (it->first));
            ml::Vector quat = mat2quat(compMatrix);

            nh.setParam(joint_name + "/quat/x",quat(0));
            nh.setParam(joint_name + "/quat/y",quat(1));
            nh.setParam(joint_name + "/quat/z",quat(2));
            nh.setParam(joint_name + "/quat/w",quat(3));
        }
    }
}

Vector RosSotRobotModel::curConf() const
{

    if (!m_HDR )
        throw std::runtime_error ("no robot loaded");
    else {
        vectorN currConf = m_HDR->currentConfiguration();
        Vector res;
        res = convertVector(currConf);

        return res;
    }
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosSotRobotModel, "RosSotRobotModel");
} // end of namespace dynamicgraph.
