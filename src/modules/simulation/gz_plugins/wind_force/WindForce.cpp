/****************************************************************************
 *
 *   Copyright (c) 2026 Emergent Swarm Solutions. All rights reserved.
 *
 ****************************************************************************/

#include "WindForce.hpp"

#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

using namespace px4;

// Register the plugin
GZ_ADD_PLUGIN(
	WindForce,
	gz::sim::System,
	WindForce::ISystemPreUpdate,
	WindForce::ISystemConfigure
)

void WindForce::Configure(const gz::sim::Entity &entity,
			  const std::shared_ptr<const sdf::Element> &sdf,
			  gz::sim::EntityComponentManager &ecm,
			  gz::sim::EventManager &eventMgr)
{
	_entity = entity;
	_model = gz::sim::Model(entity);

	const std::string link_name = sdf->Get<std::string>("link_name");
	_link_entity = _model.LinkByName(ecm, link_name);
	std::string model_name = _model.Name(ecm);

	if (!_link_entity) {
		throw std::runtime_error("WindForce::Configure: Link \"" + link_name + "\" was not found. "
					 "Please ensure that your model contains the corresponding link.");
	}

	_link = gz::sim::Link(_link_entity);
	_link.EnableVelocityChecks(ecm, true);

	gz::sim::Entity world_entity = gz::sim::worldEntity(ecm);
	gz::sim::World world(world_entity);
	std::string world_name = world.Name(ecm).value_or("default");

	std::string wind_topic = "/world/" + world_name + "/wind_info";
	_node.Subscribe(wind_topic, &WindForce::windCallback, this);

	if (sdf->HasElement("drag_area_coefficient")) {
		_drag_area_coefficient = sdf->Get<float>("drag_area_coefficient");
	}

	if (sdf->HasElement("air_density")) {
		_air_density = sdf->Get<float>("air_density");
	}

	if (sdf->HasElement("wind_velocity")) {
		std::lock_guard<std::mutex> lock(_wind_velocity_mutex);
		_wind_velocity = sdf->Get<gz::math::Vector3d>("wind_velocity");
	}

	if (sdf->HasElement("min_height_for_wind")) {
		_min_height_for_wind = sdf->Get<float>("min_height_for_wind");
	}

	gzmsg << "WindForce::Configure: link=\"" << link_name << "\" drag_area_coefficient="
		<< _drag_area_coefficient << " air_density=" << _air_density << std::endl;
}

void WindForce::PreUpdate(const gz::sim::UpdateInfo &_info,
			  gz::sim::EntityComponentManager &_ecm)
{
	const auto optional_pose = _link.WorldPose(_ecm);

	if (!optional_pose.has_value()) {
		gzerr << "WindForce: Unable to get world pose" << std::endl;
		return;
	}

	if (optional_pose->Pos().Z() < (double)_min_height_for_wind) {
		// Resting on the ground - leave it to contact friction, not wind drag.
		return;
	}

	const auto optional_vel = _link.WorldLinearVelocity(_ecm);

	if (!optional_vel.has_value()) {
		gzerr << "WindForce: Unable to get linear velocity" << std::endl;
		return;
	}

	gz::math::Vector3d wind_velocity;
	{
		std::lock_guard<std::mutex> lock(_wind_velocity_mutex);
		wind_velocity = _wind_velocity;
	}

	// Simple isotropic drag model: F = 0.5 * rho * (Cd*A) * |v_rel| * v_rel,
	// applied in world coordinates at the link's center of mass.
	const gz::math::Vector3d relative_velocity = wind_velocity - optional_vel.value();
	const float speed = (float)relative_velocity.Length();
	const double force_scale = 0.5 * (double)_air_density * (double)_drag_area_coefficient * (double)speed;
	const gz::math::Vector3d force = relative_velocity * force_scale;

	_link.AddWorldForce(_ecm, force);
}

void WindForce::windCallback(const gz::msgs::Wind &msg)
{
	// Called from a gz-transport subscriber thread, not the simulation update thread.
	std::lock_guard<std::mutex> lock(_wind_velocity_mutex);
	_wind_velocity = gz::math::Vector3d(msg.linear_velocity().x(), msg.linear_velocity().y(), msg.linear_velocity().z());
}
