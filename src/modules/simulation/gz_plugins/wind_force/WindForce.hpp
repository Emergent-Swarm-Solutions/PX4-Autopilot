/****************************************************************************
 *
 *   Copyright (c) 2026 Emergent Swarm Solutions. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <gz/sim/Util.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/System.hh>
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Wind.hh"

namespace px4
{

// Fuselage/body drag relative to the air mass. gz-sim's WindEffects applies no force to this vehicle (its
// base_link has no <enable_wind> flag) and its force model isn't independently tunable, so this applies a
// plain drag force instead. The wind is read from gz's world wind entity - the same air LiftDrag, rotor drag
// and the AirSpeed sensor use - which WindEffects updates from /world/<world>/wind (see wind_scenarios.py in
// the gzsim_bridge repository).
class WindForce:
	public gz::sim::System,
	public gz::sim::ISystemPreUpdate,
	public gz::sim::ISystemConfigure
{
public:
	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) final;

	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &eventMgr) override;

private:
	gz::sim::Entity _entity;
	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Entity _link_entity;
	gz::sim::Link _link;

	gz::sim::Entity _wind_entity{gz::sim::kNullEntity};

	// Combined drag-area coefficient (Cd * A) [m^2] of a simple isotropic drag
	// model: F = 0.5 * rho * (Cd*A) * |v_rel| * v_rel, in world frame, where
	// v_rel = wind_velocity - vehicle_velocity. This is on top of whatever
	// drag the airframe's own aerodynamic surfaces (e.g. LiftDrag) already
	// model, so keep it small relative to those - tune it (or override
	// per-vehicle via the <drag_area_coefficient> SDF tag) against the
	// airframe's real frontal area and drag coefficient.
	float _drag_area_coefficient{0.08f};
	float _air_density{1.225f}; // [kg/m^3], sea level

	// World Z [m] above which wind force is applied. Below it the vehicle is
	// treated as resting on the ground: real ground friction/landing gear keep
	// it in place there, not aerodynamic drag, and this plain drag model has no
	// notion of contact or PX4 arming state to tell the two situations apart.
	// Override per-vehicle via <min_height_for_wind> - set it a little above
	// the airframe's resting base_link height.
	float _min_height_for_wind{0.3f};
};
} // end namespace px4
