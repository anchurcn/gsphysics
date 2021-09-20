#include"BulletUtils.h"

btRigidBody* CreateStaticBody(btTransform& start, btCollisionShape* shape, btDynamicsWorld* world)
{
	btScalar mass(0.);
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(start);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	world->addRigidBody(body);
	return body;
}

void DestryStaticBody(btRigidBody* body)
{
	delete body->getMotionState();
	delete body;
}