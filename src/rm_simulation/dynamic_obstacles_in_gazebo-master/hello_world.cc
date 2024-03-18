#include <gazebo/gazebo.hh>  
/* 包含gazebo基本功能，
不包括gazebo/physics/physics.hh, gazebo/rendering/rendering.hh, 和 gazebo/sensors/sensors.hh */
 
namespace gazebo  //所有插件都需要在gazebo名空间中编写
{
 
/* 所有插件类的创建需要继承一种plugin类型，下面是继承WorldPlugin */
  class WorldPluginTutorial : public WorldPlugin  
  {
    public: WorldPluginTutorial() : WorldPlugin()//默认构造函数
            {
              printf("Hello World!\n");//只要创建该类类型就会打印
            }
/* 唯一另一个必须的函数是Load(),其中_sdf接收一个包含sdf文件中指定的参数和属性 */
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }
  };
/* 使用GZ_REGISTER_WORLD_PLUGIN macro函数将插件注册，函数的需要的参数仅为插件类名
macro函数还包括GZ_REGISTER_MODEL_PLUGIN, GZ_REGISTER_SENSOR_PLUGIN, GZ_REGISTER_GUI_PLUGIN,
 GZ_REGISTER_SYSTEM_PLUGIN and GZ_REGISTER_VISUAL_PLUGIN*/
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
 
}