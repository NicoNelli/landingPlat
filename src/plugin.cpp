#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "lcm/lcm-cpp.hpp"
#include "lcm_messages/geometry/pose.hpp"
#include "WaveGen.h"

#define VEL 1
#define MAX_DIST 5

namespace gazebo
{

    class PlatformManager : public ModelPlugin
    {

    private:

        physics::ModelPtr _model;
        event::ConnectionPtr _updateConnection;

        std::string _cmd_pose_topic;
        std::string _local_pose_topic;

        lcm::LCM _handler;

        double _actual_sim_time;
        double _prev_sim_time;
        bool _got_t_offs;
        double _time_offs;
        double _dt;
        double _platVel;
        double _dir;
        WaveGen wave;
        math::Pose _pose, _start_pose,_prev_pose;

        void manageTime(const common::UpdateInfo & _info){

            double t = (double)_info.simTime.sec +  (double)_info.simTime.nsec / (double)_info.simTime.nsInSec;;
            if(!_got_t_offs){
                _time_offs = t;
                _got_t_offs = true;
            }

            _actual_sim_time = - _time_offs + t;
            _dt = _actual_sim_time - _prev_sim_time;
            _prev_sim_time = _actual_sim_time;

        }

    public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
                // Store the pointer to the model
                this->_model = _parent;
                _model.get()->SetStatic(true);
                _model.get()->SetGravityMode(false);
                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
                        boost::bind(&PlatformManager::OnUpdate, this, _1));

                _local_pose_topic = _model.get()->GetName() + "/pose";
                _cmd_pose_topic   = _model.get()->GetName() + "/pose_cmd";

                wave.init(10,4.5,0.5,7.0);

                _pose = _model.get()->GetWorldPose();
                _prev_pose  = _pose;
                _start_pose = _pose;

                _got_t_offs = false;
                _actual_sim_time = 0;
                _prev_sim_time = 0;

                std::cout << "New platform registered, topics:" << std::endl;
                std::cout << "  "<<_local_pose_topic << std::endl;
                std::cout << "  "<<_cmd_pose_topic   << std::endl;
                std::cout << "And some info: " << std::endl;
                wave.info();
                _platVel = VEL;
                _dir = 1;

            }
            void invertDirection(){

                _dir = -_dir;

            }

            // Called by the world update start event
            void OnUpdate(const common::UpdateInfo & _info) {

                manageTime(_info);

                _pose = _model.get()->GetWorldPose();
                _pose.rot = math::Quaternion(1,0,0,0);
                //_pose.pos.z = wave.generateWaveHeightAtPoint(_start_pose.pos.x,_start_pose.pos.y,_actual_sim_time,0) + 3;

                double xTemp = _pose.pos.x;
                if(fabs(xTemp) > ((MAX_DIST - 0.01) + _start_pose.pos.x)) invertDirection();

                double step =  _dir * (_dt * _platVel);
                double newX = xTemp + step;

                _pose.pos.x = newX;
                _model.get()->SetAngularVel(math::Vector3(0,0,0));
                _pose.pos.z = 1;
                _model.get()->SetWorldPose(_pose);

                //Prepare LCM material
                double vx,vy,vz;
                geometry::pose plat_pose;

                vx = (_pose.pos.x - _prev_pose.pos.x)/_dt;
                vy = (_pose.pos.y - _prev_pose.pos.y)/_dt;
                vz = (_pose.pos.z - _prev_pose.pos.z)/_dt;

                plat_pose.position[0] = _pose.pos.x;
                plat_pose.position[1] = _pose.pos.y;
                plat_pose.position[2] = _pose.pos.z;

                plat_pose.velocity[0] = vx;
                plat_pose.velocity[1] = vy;
                plat_pose.velocity[2] = vz;

                plat_pose.isValid = 1;
                plat_pose.orientation[0] = _pose.rot.w;
                plat_pose.orientation[1] = _pose.rot.x;
                plat_pose.orientation[2] = _pose.rot.y;
                plat_pose.orientation[3] = _pose.rot.z;

                _handler.publish(_local_pose_topic,&plat_pose);

                _prev_pose = _pose;

            }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PlatformManager)
}