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

#ifndef SEMANTIC_PLANNING_STATE_H
#define SEMANTIC_PLANNING_STATE_H

#include "abstract_state.h"
#include "semantic_to_cartesian_converter.h"
#include <dual_manipulation_shared/databasemapper.h>
#include <dual_manipulation_shared/planner_service.h>
#include <ros/node_handle.h>

class semantic_planning_state : public abstract_state<transition>
{
public:
    semantic_planning_state(shared_memory& data);
    virtual std::map< transition, bool > getResults();

    /**
     * @brief Expects a cartesian source position, an object ID and a cartesian target position from the shared_memory
     * 
     * @return void
     */
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
    virtual void reset();
private:
    
    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::Publisher good_grasps_pub;
    dual_manipulation_shared::planner_service srv;
    const databaseMapper& database;
    shared_memory& data;
    bool completed;
    std::map< transition, bool > internal_state;
    semantic_to_cartesian_converter converter;
    void show_plan_with_markers();
    ros::Publisher planned_path_publisher_;
};

#endif // SEMANTIC_PLANNING_STATE_H
