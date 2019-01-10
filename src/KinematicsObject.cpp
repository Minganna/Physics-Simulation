#include "KinematicsObject.h"
#include <GLM/gtc/type_ptr.hpp>
#include <GLM/gtc/matrix_transform.hpp>
#include"Utility.h"

using namespace PFG;

KinematicsObject::KinematicsObject()
{
	v_i = glm::vec3(0.02f, 0.3f, 0.01f); // set the initial velocity as zero for a free fall object
	gravity = glm::vec3(0.0f, -GRAVITY, 0.0f);
	v_f = glm::vec3(0.0f, 0.0f, 0.0f);

	_scale = glm::vec3(1.0f, 1.0f, 1.0f);
	_start = false;

}

KinematicsObject::~KinematicsObject()
{
}

void KinematicsObject::Update(float deltaTs)
{


		// Physics
     	displacement = GetPosition();
	    v_f.y=KinematicsEquationsFinalVelocity(deltaTs);
		KinematicsEquationsDisplacement(deltaTs);

		v_i = v_f;

		// Make stop on floor
		if (displacement.y <= 0.3f)
		{
			displacement.y = 0.3f;
			v_i.y = -v_i.y;
		}
		SetPosition(displacement);
		
		_modelMatrix = glm::translate(glm::mat4(1.0f), _position);
		_modelMatrix = glm::scale(_modelMatrix, _scale);

}

float KinematicsObject::KinematicsEquationsFinalVelocity(float deltaTs)
{
	
	
	v_f.y = v_i.y + (gravity.y * deltaTs);
	//glm::vec3 v_fpow2;
	//v_fpow2.y = pow(v_i.y, 2) + 2 * gravity.y*displacement.y;
	//v_f.y=sqrt(v_fpow2.y);

	return v_f.y;

}

glm::vec3 KinematicsObject::KinematicsEquationsDisplacement(float deltaTs)
{
	//displacement += v_i * deltaTs + 1 / 2 * gravity.y*pow(deltaTs, 2);
	displacement += (v_i + v_f) / 2.0f * deltaTs;
	return displacement;
}

void KinematicsObject::UpdateModelMatrix()
{
	return;
}
