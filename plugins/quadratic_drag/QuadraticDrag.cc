/*
 * QuadraticDrag — Gazebo system plugin for quadratic aerodynamic body drag.
 *
 * Applies F_drag = -½ρ CdA |v| v to a specified link, where:
 *   ρ    = air density (kg/m³, default 1.225)
 *   CdA  = drag coefficient × frontal area (m², default 0.020)
 *   v    = link linear velocity in world frame
 *
 * This replaces Gazebo's linear velocity_decay with physically correct
 * v² drag for high-speed drone simulation.
 *
 * SDF usage:
 *   <plugin filename="QuadraticDrag" name="quadratic_drag::QuadraticDrag">
 *     <link_name>base_link</link_name>
 *     <CdA>0.020</CdA>
 *     <air_density>1.225</air_density>
 *   </plugin>
 */

#include <string>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/plugin/Register.hh>
#include <sdf/Element.hh>

namespace quadratic_drag
{

class QuadraticDrag
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);

    if (_sdf->HasElement("CdA"))
      this->CdA = _sdf->Get<double>("CdA");
    if (_sdf->HasElement("air_density"))
      this->rho = _sdf->Get<double>("air_density");

    std::string linkName = "base_link";
    if (_sdf->HasElement("link_name"))
      linkName = _sdf->Get<std::string>("link_name");

    this->halfRhoCdA = 0.5 * this->rho * this->CdA;

    this->linkEntity = this->model.LinkByName(_ecm, linkName);
    if (this->linkEntity == gz::sim::kNullEntity)
    {
      gzerr << "QuadraticDrag: link [" << linkName << "] not found\n";
      return;
    }

    gz::sim::Link link(this->linkEntity);
    link.EnableVelocityChecks(_ecm, true);

    gzmsg << "QuadraticDrag: CdA=" << CdA
           << " rho=" << rho
           << " halfRhoCdA=" << halfRhoCdA
           << " link=" << linkName << "\n";
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (this->linkEntity == gz::sim::kNullEntity)
      return;

    gz::sim::Link link(this->linkEntity);
    auto velOpt = link.WorldLinearVelocity(_ecm);
    if (!velOpt)
      return;

    auto vel = velOpt.value();
    double speed = vel.Length();
    if (speed < 0.01)
      return;

    gz::math::Vector3d dragForce = -this->halfRhoCdA * speed * vel;
    link.AddWorldForce(_ecm, dragForce);
  }

private:
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};
  double CdA{0.020};
  double rho{1.225};
  double halfRhoCdA{0.01225};
};

}

GZ_ADD_PLUGIN(
    quadratic_drag::QuadraticDrag,
    gz::sim::System,
    quadratic_drag::QuadraticDrag::ISystemConfigure,
    quadratic_drag::QuadraticDrag::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    quadratic_drag::QuadraticDrag,
    "quadratic_drag::QuadraticDrag")
