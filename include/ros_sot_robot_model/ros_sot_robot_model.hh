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

# ifndef ROS_SOT_ROBOT_MODEL_HH
# define ROS_SOT_ROBOT_MODEL_HH
# include <string>

# include "jrl/mal/matrixabstractlayer.hh"

namespace ml = maal::boost;

# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <sot-dynamic/dynamic.h>
#include <jrl/dynamics/urdf/parser.hh>
# include "XmlRpcValue.h"
# include <ros/ros.h>

#include <boost/shared_ptr.hpp>
namespace dynamicgraph
{
  class RosSotRobotModel;

  /// \brief This entity load either the current model available in
  /// the robot_description parameter or a specified file and provides
  /// various data such as body positions, jacobians, etc.
  ///
  /// This relies on jrl_dynamics_urdf to load the model and jrl-dynamics
  /// to realize the computation.
  class RosSotRobotModel : public sot::Dynamic//, jrl::dynamics::urdf::Parser
  {
    DYNAMIC_GRAPH_ENTITY_DECL();
  public:

    RosSotRobotModel(const std::string& n);

    virtual ~RosSotRobotModel();

    void loadUrdf(const std::string& filename);
    void loadFromParameterServer();
    void setParamsJointNames();
    void setParamsJointOrientations();
    void setNamespace (const std::string& ns);
    void setParameterName (const std::string& pn);
    Vector curConf() const;

  private:

    /// \brief Name of the parameter where the joints list will be published
    std::string pn_;

    /// \brief Controller namespace
    std::string ns_;

    /// \brief List of actuated joints ordered by rank
    XmlRpc::XmlRpcValue jointNames_;

    boost::shared_ptr< jrl::dynamics::urdf::Parser > parser_;

  };
} // end of namespace dynamicgraph.

# endif //! ROS_SOT_ROBOT_MODEL_HH
