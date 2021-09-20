#pragma once
#include<btBulletCollisionCommon.h>
#include<btBulletDynamicsCommon.h>
#include<PhysicsAssetsManager.h>
#include <assert.h>
#include "hud.h"
#include "cl_util.h"
#include "const.h"
#include "com_model.h"
#include "studio.h"
#include "entity_state.h"
#include "cl_entity.h"
#include "dlight.h"
#include "triangleapi.h"

#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <math.h>

#include "studio_util.h"
#include "r_studioint.h"
#include<ConstructionInfo.h>

#pragma region SkeletalPhysicsComponent

class IPhysicsComponent
{
public:
    virtual void Enable() = 0;
    virtual void Disable() = 0;
    virtual ~IPhysicsComponent() {};
    //virtual void SetPose(float pInBonesTransform[MAXSTUDIOBONES][3][4]) = 0;
    //virtual void SetupBones(float pOutBoneTransform[MAXSTUDIOBONES][3][4]) = 0;
    //virtual bool GetIsKinematic() = 0;
    //virtual void SetIsKinematic(bool value) = 0;
    //virtual void SetVelocity(float linearV[3]) = 0;
    //virtual void SetVelocity(float linearV[3],float angularV[3]) = 0;
    //
    //virtual void GetWorldTransform(btTransform& out) = 0;
    //virtual void GetWorldTransform(float out[3][4]) = 0; 
    //virtual void GetWorldTransform(float* const outOrigin, float* const outAngle) = 0;
};
enum class ComponentType
{
    /// <summary>
    ///     They are supposed to never move, they are not automatically updated by the engine.
    /// </summary>
    Static,
    /// <summary>
    ///     You can move this kind of rigidbody around and the physics engine will interpolate and perform dynamic interactions
    ///     with dynamic bodies
    ///     Notice that there is no dynamic interaction with static and other kinematic bodies
    /// </summary>
    Kinematic,
    /// <summary>
    ///     The Physics engine is the authority for this kind of rigidbody, you should move them using forces and/or impulses,
    ///     never directly editing the Transformation
    /// </summary>
    Dynamic
};
class EntityPhysicsComponent :public IPhysicsComponent
{
private:
    btRigidBody* _pRigidbody;
    btDynamicsWorld* _pWorld;
public:
    EntityPhysicsComponent(cl_entity_t* pent, btCollisionShape* pshape,btDynamicsWorld* pworld);
    EntityPhysicsComponent(float mass, cl_entity_t* pent, btCollisionShape* pshape, btDynamicsWorld* pworld);
    ~EntityPhysicsComponent();
    virtual void Enable();
    virtual void Disable();
    void SetComponentType(ComponentType type);
    btRigidBody* GetRigidbody() { return _pRigidbody; };
    ComponentType GetComponentType() const;

    // TODO:
    void SetIsKinematic(bool);
};


class SkeletalPhysicsComponent:public IPhysicsComponent
{
public:
    SkeletalPhysicsComponent(bool isKinematic,PhysicsComponentConstructionInfo* pConstructionInfo,btDynamicsWorld* pworld);
    SkeletalPhysicsComponent(
        bool isKinematic,
        PhysicsComponentConstructionInfo* pConstructionInfo,
        btDynamicsWorld* pworld,
        float initpose[MAXSTUDIOBONES][3][4]);
    ~SkeletalPhysicsComponent();

    virtual void Enable();
    virtual void Disable();

    void GetWorldTransform(float outMatrix[3][4]);
    void GetWorldTransform(float* const outOrigin, float* const outAngle) { }
    bool GetIsKinematic() { return _isKinematic; }
    void SetIsKinematic(bool value);
    void SetPose(float pInBonesTransform[MAXSTUDIOBONES][3][4]);
    void SetupBones(float pOutBoneTransform[MAXSTUDIOBONES][3][4]);
    void SetVelocity(btVector3& v);

private:
    PhysicsComponentConstructionInfo* _pConstructionInfo;
    float _boneLocalTransforms[MAXSTUDIOBONES][3][4];
    std::vector<btRigidBody*> _rigidbodies;
    std::vector<btTypedConstraint*> _constraints;
    bool _enabled;
    bool _isKinematic;
    studiohdr_t* _phdr;
    mstudiobone_t* _pbones;
    btDynamicsWorld* pWorld;

    btDynamicsWorld* getWorld() { return pWorld; }
    void SetKinematic();
    void SetDynamic();
    void Setup(
        bool isKinematic,
        PhysicsComponentConstructionInfo* pConstructionInfo,
        btDynamicsWorld* pworld,
        float initpose[MAXSTUDIOBONES][3][4]);

};
#pragma endregion

#pragma region MotionState

ATTRIBUTE_ALIGNED16(class)
BoneMotionState : public btMotionState
{
public:
    BT_DECLARE_ALIGNED_ALLOCATOR();
    BoneMotionState(const btTransform& bm, const btTransform& om) : bonematrix(bm), offsetmatrix(om)
    {

    }
    BoneMotionState(const btTransform& offsetMatrix):offsetmatrix(offsetMatrix),bonematrix()
    {
        bonematrix.setIdentity();
    }
    virtual void getWorldTransform(btTransform& worldTrans) const;
    virtual void setWorldTransform(const btTransform& worldTrans);
    void GetBoneTransform(float out[3][4]);
    void SetBoneTransform(float const in[3][4]);
    btTransform bonematrix;
    const btTransform offsetmatrix;
};


ATTRIBUTE_ALIGNED16(class)
EntityMotionState : public btMotionState
{
public:
    BT_DECLARE_ALIGNED_ALLOCATOR();
    EntityMotionState(cl_entity_t* pent)
    {
        _pEntity = pent;
    }
    virtual ~EntityMotionState()
    {

    }
    virtual void getWorldTransform(btTransform& worldTrans) const;
    virtual void setWorldTransform(const btTransform& worldTrans);

private:
    cl_entity_t* _pEntity;
};
#pragma endregion

