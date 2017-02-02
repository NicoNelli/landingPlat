#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "WaveGen.h"
namespace gazebo
{

    class PlatformManager : public ModelPlugin
    {


        //************Members**********************//
        // Pointer to the model
    private:

        physics::ModelPtr _model;
        event::ConnectionPtr _updateConnection;

        std::string _cmd_pose_topic;
        std::string _local_pose_topic;

        int _id;

        double _actual_sim_time;
        double _prev_sim_time;
        bool _got_t_offs;
        double _time_offs;
        double _dt;
        WaveGen wave;

        //************Methods**********************//

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

                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
                        boost::bind(&PlatformManager::OnUpdate, this, _1));

                // YEAH...WTF! GazeboID for this object go with a step of 6 starting from 10........
                _id = ((_model.get()->GetId()) - 10 )/6;

                _local_pose_topic = "platform_" + std::to_string(_id) + "/pose";
                _cmd_pose_topic   = "platform_" + std::to_string(_id) + "/pose_cmd";

                wave.init(10,4.5,0.5,7.0);

                _got_t_offs = false;
                _actual_sim_time = 0;
                _prev_sim_time = 0;
                std::cout << "New platform registered, topics:" << std::endl;
                std::cout << "  "<<_local_pose_topic << std::endl;
                std::cout << "  "<<_cmd_pose_topic   << std::endl;

            }

            // Called by the world update start event
            void OnUpdate(const common::UpdateInfo & _info) {

                manageTime(_info);


                math::Pose pose = _model.get()->GetRelativePose();

                pose.pos.z = wave.generateWaveHeightAtPoint(0,0,_actual_sim_time,0) + 5;

                _model.get()->SetAngularVel(math::Vector3(0,0,0));
                _model.get()->SetRelativePose(pose);

            }

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PlatformManager)
}