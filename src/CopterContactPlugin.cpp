#include "gazebo_fpi/CopterContactPlugin.h"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CopterContactPlugin)

/////////////////////////////////////////////////
CopterContactPlugin::CopterContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
CopterContactPlugin::~CopterContactPlugin()
{
}

/////////////////////////////////////////////////
void CopterContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&CopterContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
  
  this->nh = ros::NodeHandle("contact_plugin_node");
  this->pub = this->nh.advertise<geometry_msgs::Vector3>("pose", 10);

}

/////////////////////////////////////////////////
void CopterContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  
  if (contacts.contact_size() > 0) {
      
    geometry_msgs::Vector3 contact_position;
    contact_position.x = contacts.contact(0).position(0).x();
    contact_position.y = contacts.contact(0).position(0).y();
    contact_position.z = contacts.contact(0).position(0).z();
    this->pub.publish(contact_position);
  }
  
//   for (unsigned int i = 0; i < contacts.contact_size(); ++i)
//   {
//     std::cout << "Collision between[" << contacts.contact(i).collision1()
//               << "] and [" << contacts.contact(i).collision2() << "]\n";
// 
//     for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
//     {
//       std::cout << j << "  Position:"
//                 << contacts.contact(i).position(j).x() << " "
//                 << contacts.contact(i).position(j).y() << " "
//                 << contacts.contact(i).position(j).z() << "\n";
//       std::cout << "   Normal:"
//                 << contacts.contact(i).normal(j).x() << " "
//                 << contacts.contact(i).normal(j).y() << " "
//                 << contacts.contact(i).normal(j).z() << "\n";
//       std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
//     }
//   }

}
