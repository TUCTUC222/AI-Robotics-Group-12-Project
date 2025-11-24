#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class CylinderMover : public ModelPlugin
  {
  public:
    physics::ModelPtr model;
    event::ConnectionPtr updateConn;
    std::vector<ignition::math::Vector3d> waypoints;
    size_t index = 0;
    double speed = 0.8;

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      model = _parent;

      // Read waypoint list
      sdf::ElementPtr wp = _sdf->GetElement("waypoint");
      while (wp)
      {
        std::stringstream ss(wp->Get<std::string>());
        double x, y;
        ss >> x >> y;
        waypoints.push_back(ignition::math::Vector3d(x, y, 0.5));
        wp = wp->GetNextElement("waypoint");
      }

      if (_sdf->HasElement("speed"))
        speed = _sdf->Get<double>("speed");

      updateConn = event::Events::ConnectWorldUpdateBegin(
        std::bind(&CylinderMover::OnUpdate, this));
    }

    void OnUpdate()
    {
      if (waypoints.empty()) return;

      ignition::math::Pose3d pose = model->WorldPose();
      ignition::math::Vector3d pos = pose.Pos();
      ignition::math::Vector3d target = waypoints[index];

      ignition::math::Vector3d diff = target - pos;

      if (diff.Length() < 0.15)
      {
        index = (index + 1) % waypoints.size();
        return;
      }

      diff.Normalize();
      model->SetLinearVel(diff * speed);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(CylinderMover)
}
