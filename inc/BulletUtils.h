#pragma once
#include<btBulletCollisionCommon.h>
#include<btBulletDynamicsCommon.h>


btRigidBody* CreateStaticBody(btTransform& start, btCollisionShape* shape, btDynamicsWorld* world);

void DestryStaticBody(btRigidBody* body);