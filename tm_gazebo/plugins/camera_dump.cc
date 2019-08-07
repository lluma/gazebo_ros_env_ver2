#include <gazebo/gazebo.hh>
#include <gazebo/plugins/CameraPlugin.hh>

namespace gazebo
{
  class CameraDump : public CameraPlugin
  {
    public : CameraDump() : CameraPlugin(), saveCount(0) {}

    public : void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      // Don't forget to load the camera plugin
      CameraPlugin::Load(_parent, _sdf);
    }

    // Update the controller
    public : void OnNewFrame(const unsigned char *_image, unsigned int _width, 
      unsigned int _height, unsigned int _depth, const std::string &_format)
    {
      char tmp[1024];
      snprintf(tmp, sizeof(tmp), "/tmp/%s-%04d.jpg", 
        this->parentSensor->GetCamera()->GetName().c_str(), this->saveCount);
    
      if (this->saveCount < dumpNumber)
      {
        this->parentSensor->GetCamera()->SaveFrame(_image, _width, _height, 
          _depth, _format, tmp);
        gzmsg << "Saving frame [" << this->saveCount << "] as [" << tmp << "]\n";
	this->saveCount++;
      }
    }

    private : int saveCount;
    private : int dumpNumber = 20;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(CameraDump)
}
