#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class AnimatedBox1 : public ModelPlugin
  {
    /* 唯一另一个必须的函数是Load(),其中_sdf接收一个包含sdf文件中指定的参数和属性 */

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test", 12.0, true));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(-1, 1, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set waypoint location after 2 seconds
        key = anim->CreateKeyFrame(2.0);
        key->Translation(ignition::math::Vector3d(-1, 2, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));


        key = anim->CreateKeyFrame(4.0);
        key->Translation(ignition::math::Vector3d(-1, 3, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));


        key = anim->CreateKeyFrame(6.0);
        key->Translation(ignition::math::Vector3d(-1, 4, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));


        key = anim->CreateKeyFrame(8.0);
        key->Translation(ignition::math::Vector3d(-1, 3, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(10.0);
        key->Translation(ignition::math::Vector3d(-1, 2, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        
        key = anim->CreateKeyFrame(12.0);
        key->Translation(ignition::math::Vector3d(-1, 1, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set the animation
        _parent->SetAnimation(anim);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

/* 使用GZ_REGISTER_WORLD_PLUGIN macro函数将插件注册，函数的需要的参数仅为插件类名
macro函数还包括GZ_REGISTER_MODEL_PLUGIN, GZ_REGISTER_SENSOR_PLUGIN, GZ_REGISTER_GUI_PLUGIN,
 GZ_REGISTER_SYSTEM_PLUGIN and GZ_REGISTER_VISUAL_PLUGIN*/
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox1)
}
