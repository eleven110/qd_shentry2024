#include <functional>   ///STL 定义运算函数（代替运算符）
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
 
namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    /* typedef boost::shared_ptr<Model> ModelPtr Model类型的共享指针
        sdf::ElementPtr指向sdf，即将.world文件中的参数属性导入进来*/
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      /* Store the pointer to the model
         让ModelPush中的model指向当前模型*/
      this->model = _parent;
 
      /* Listen to the update event. This event is broadcast every
         simulation iteration循环.
         typedef boost::shared_ptr<Connection> ConnectionPtr
         static ConnectionPtr ConnectWorldUpdateBegin(T _subscriber) */
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));//std::bind在functional中
    }
 
    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      // void SetLinearVel(const math::Vector3 & _vel) 	
      //model是指向physics中的Model类的智能指针，因此要在API中model class中找SetLinearVel
      this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }
 
    // Pointer to the model
    private: physics::ModelPtr model;
 
    /* Pointer to the update event connection
       typedef boost::shared_ptr<Connection> ConnectionPtr*/
    private: event::ConnectionPtr updateConnection;
  };
 
  // 注册插件
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}