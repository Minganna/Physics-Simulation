#include "DynamicObject.h"
#include <GLM/gtc/type_ptr.hpp>
#include <GLM/gtc/matrix_transform.hpp>
#include"Utility.h"

using namespace PFG;

DynamicObject::DynamicObject()
{
	gravity = glm::vec3(0.0f, -9.8f, 0.0f);
	_mass = 1.67f;
	_bRadius = 0.4f;

	_R = glm::mat3(1.0f);

	_scale = glm::vec3(1.0f, 1.0f, 1.0f);
	_start = false;
}

DynamicObject::~DynamicObject()
{
}

void DynamicObject::Update(float deltaTs)
{
	if (_start == true)
	{
		//Clear All Forces
		ClearForces();
		//Compute each force acting on objects
		glm::vec3 f = ComputeForces();
		//Add the computed forces to the total force
		AddForce(f);

		glm::vec3 n(0.005f, 1.0f, 0.0f); //the normal of the plane pointing up
		n = glm::normalize(n);
		glm::vec3 q(0.0f, 0.0f, 0.0f);

		//Lab 7 Exercise 1 use point to plane to find the distance
		float d = PFG::DistanceToPlane(n, _position, q);
		
		glm::vec3 contactPosition;
		glm::vec3 c1 = _position + _velocity * deltaTs;
		glm::vec3 c0 = _position;
		bool collision = PFG::MovingSphereToPlaneCollision2(n, c0, c1, q, _bRadius, contactPosition);

		//Compute Collisions
		if (collision)
		{
			glm::vec3 velocityFloor(0.0f, 0.0f, 0.0f);
			glm::vec3 relativevelocity = _velocity - velocityFloor;
			float eCof = -(1.0f + dragCoeficent)*glm::dot(relativevelocity, n);
			float invMass = 1 / _mass;
			float invFloorMass = 0.0f;
			float j = eCof / (invMass + invFloorMass);

			glm::vec3 collision_impulse_force = j * n / deltaTs;
			glm::vec3 new_velocity = _velocity + j * n*invMass;

			glm::vec3 contact_force(0.0f, 9.8f*_mass, 0.0f);
			glm::vec3 total_force = collision_impulse_force + contact_force;
			AddForce(total_force);

			_torque =glm::cross(n*_bRadius, contact_force);
			std::cout << _mass << std::endl;
		}

		//Euler(deltaTs);
		RungeKutta2(deltaTs);
		//RungeKutta4(deltaTs);
		
	}
	UpdateModelMatrix();
}

void DynamicObject::ComputeInverseInertiaTensor()
{

	//Find the transpose of the rotation matrix _R

	_inertia_tensor_inverse = _R * _body_inertia_tensor_inverse*glm::transpose(_R);
}

void DynamicObject::Euler(float deltaTs)
{
	//numerical integration of the dynamic physics 
	float oneOverMass = 1 / _mass;
	//Compute current velocity based on previous velocity
	_velocity += (_force*oneOverMass)*deltaTs;
	//compute the current position based on previous position
	_position += _velocity * deltaTs;

	//rotation part of the physics computation
	
	//1.Compute the current angular momentum 
	_angular_momentum += _torque * deltaTs;


	//2.Compute Inverse Inertia Tensor
	ComputeInverseInertiaTensor();
	//Update the angular velocity
	_angular_velocity = _inertia_tensor_inverse * _angular_momentum;
	//compute skew mateix omega star 
	glm::mat3 omega_star = glm::mat3(0.0f, -_angular_velocity.z, _angular_velocity.y, _angular_velocity.z, 0.0f, -_angular_velocity.x, -_angular_velocity.y, _angular_velocity.x, 0.0f);
	//Update the rotation matrix R
	_R += omega_star * _R*deltaTs;
	
}

void DynamicObject::RungeKutta2(float deltaTs)
{
	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;

	//Evaluate once at t0
	force = _force;
	acceleration = force / _mass;
	k0 = deltaTs * acceleration;

	//Evaluate once at t0 +deltaT/2.0 using half of k0
	force = force + k0 / 2.0f;
	acceleration = force / _mass;
	k1 = deltaTs * acceleration;

	//evaluate once at t0 +deltaT using k1
	_velocity += k1;
	_position += _velocity * deltaTs;


	//rotation part of the physics computation

	//1.Compute the current angular momentum 
	_angular_momentum += _torque * deltaTs;
	//2.Compute Inverse Inertia Tensor
	ComputeInverseInertiaTensor();
	//Update the angular velocity
	_angular_velocity = _inertia_tensor_inverse * _angular_momentum;
	//compute skew mateix omega star 
	glm::mat3 omega_star = glm::mat3(0.0f, -_angular_velocity.z, _angular_velocity.y, _angular_velocity.z, 0.0f, -_angular_velocity.x, -_angular_velocity.y, _angular_velocity.x, 0.0f);
	//Update the rotation matrix R
	_R += omega_star * _R*deltaTs;
}

void DynamicObject::RungeKutta4(float deltaTs)
{
	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;
	glm::vec3 k2;
	glm::vec3 k3;

	//Evaluate once at t0 to find k0
	force = _force;
	acceleration = force / _mass;
	k0 = deltaTs * acceleration;
	//Evaluate twice at t0 + deltaT/2.0 using half of k0 and half of k1
	force = _force + k0 / 2.0f;
	acceleration = force / _mass;
	k1 = deltaTs * acceleration;

	force = _force + k1 / 2.0f;
	acceleration = force / _mass;
	k2 = deltaTs * acceleration;

	force = _force + k2;
	acceleration = force / _mass;
	k3 = deltaTs * acceleration;

	//Evaluate once at t0+deltaT using weighted sum of k0,k1,k2 and k3

	_velocity += (k0 + 2.0f*k1 + 2.0f*k2 + k3) / 6.0f;
	//Update position
	_position += _velocity * deltaTs;


	//rotation part of the physics computation

	//1.Compute the current angular momentum 
	_angular_momentum += _torque * deltaTs;
	//2.Compute Inverse Inertia Tensor
	ComputeInverseInertiaTensor();
	//Update the angular velocity
	_angular_velocity = _inertia_tensor_inverse * _angular_momentum;
	//compute skew mateix omega star 
	glm::mat3 omega_star = glm::mat3(0.0f, -_angular_velocity.z, _angular_velocity.y, _angular_velocity.z, 0.0f, -_angular_velocity.x, -_angular_velocity.y, _angular_velocity.x, 0.0f);
	//Update the rotation matrix R
	_R += omega_star * _R*deltaTs;


}


glm::vec3 DynamicObject::ComputeForces()
{
	//attempt to add air friction 
	//glm::vec3 force = _mass * ((_mass*gravity-airResistance)/_mass);

	glm::vec3 force = _mass * gravity;

	return  force;
}

void DynamicObject::UpdateModelMatrix()
{
	//Set model rotation using _R
	//Following line of code turns a 3x3 matrix into a 4x4 matrix
	glm::mat4 model_rotation = glm::mat4(_R);

	glm::quat tempQuat = glm::normalize(glm::quat_cast(model_rotation));

	_R = glm::mat3_cast(tempQuat);

	//Update the model matrix with the current position, orientation and scale
	_modelMatrix = glm::translate(glm::mat4(1.0f), _position);
	_modelMatrix = _modelMatrix * glm::mat4_cast(tempQuat);
	_modelMatrix = glm::scale(_modelMatrix, _scale);
	_invModelMatrix = glm::inverse(_modelMatrix);

}
