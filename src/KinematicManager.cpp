//#include "KinematicManager.h"
//#include "PhysicsManager.h"
//
//
//class EntityMotionState:btMotionState
//{
//public:
//	EntityMotionState(cl_entity_t* pent);
//	~EntityMotionState();
//
//	virtual void getWorldTransform(btTransform& worldTrans) const;
//	virtual void setWorldTransform(const btTransform& worldTrans);
//private:
//	cl_entity_t* _pentity;
//};
//
//EntityMotionState::EntityMotionState(cl_entity_t* pent)
//{
//	_pentity = pent;
//}
//
//EntityMotionState::~EntityMotionState()
//{
//}
//
//void EntityMotionState::getWorldTransform(btTransform& worldTrans) const
//{
//	btVector3 origin()
//}
//
//void EntityMotionState::setWorldTransform(const btTransform& worldTrans)
//{
//}
//
//BrushEntityKinematicComponent::BrushEntityKinematicComponent(cl_entity_t* pEntity, btCollisionShape* pshape)
//{
//
//}
//
//void BrushEntityKinematicComponent::Enable()
//{
//    pgDynamicWorld->addRigidBody(_pKinematicRigidBody);
//}
//
//void BrushEntityKinematicComponent::Disable()
//{
//    pgDynamicWorld->removeRigidBody(_pKinematicRigidBody);
//}