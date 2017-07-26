/*********************************************************************
*
* Software License Agreement (BSD License)
*
*
*  Copyright (c) 2016, Hamal Marino <hamal dot marino at gmail dot com>
*  Copyright (c) 2017, George Jose Pollayil <gpollayil at gmail dot com>
*  Copyright (c) 2017, Mathew Jose Pollayil <mathewjosepollayil at gmail dot com>
*  Copyright (c) 2017, Centro di Ricerca "E. Piaggio", University of Pisa
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
*   * Neither the name of the ISR University of Coimbra nor the names of its
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
*********************************************************************/

#ifndef GETTING_INFO_STATE_H
#define GETTING_INFO_STATE_H

#include <abstract_state.h>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/gui_target_response.h>
#include <XmlRpcValue.h>
#include <pacman_vision_comm/estimate.h>
#include <pacman_vision_comm/peArray.h>

class getting_info_state : public abstract_state<transition>
{
public:
    getting_info_state(shared_memory& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
    virtual void reset();
private:
    shared_memory& data_;
    bool fresh_data;
    ros::NodeHandle n;
    ros::ServiceClient planner_client, gui_target_client, scene_object_client;
    ros::ServiceClient vision_client;
    ros::ServiceClient tracker_start_client,tracker_stop_client;
    
    const databaseMapper& db_mapper_;

    void get_start_position_from_vision(pacman_vision_comm::peArray& source_poses);
    
    /**
     * @brief given and object id and its pose, return the associated grasp id (checked from the database)
     * 
     * @param object_id
     *   the object_id of the object in the scene
     * @param pose
     *   the pose for which we want to know the grasp
     * @param ee_id
     *   the id of the end-effector, decided from the user (default = table)
     *   (at now table is the only implemented one)
     * @return the index of the associated grasp id
     */
    int get_grasp_id_from_database(int object_id, geometry_msgs::Pose pose, int ee_id = 3);

    /**
     * @brief Given the name of an object, returns the associated id. The name may contain a suffix w.r.t. the database name, but not a prefix.
     * 
     * @param obj_name name of the object to look up in the database
     * 
     * @return the object id from the database; -1 if no corresponding entry has been found
     */
    int get_object_id(std::string obj_name);

    void get_target_position_from_user(pacman_vision_comm::peArray source_poses);

    void parseParameters(XmlRpc::XmlRpcValue& params);

    ros::Subscriber target_sub;
    void gui_target_set_callback(const dual_manipulation_shared::gui_target_response::ConstPtr& msg);

    bool failed=false;
    std::atomic<bool> target_set;
    bool source_set=false;
    bool target_request=false;
    bool use_vision=true;
    int table_ee_id=3;
};

#endif // GETTING_INFO_STATE_H
