#pragma once
#include<btBulletCollisionCommon.h>
#include<btBulletDynamicsCommon.h>
#include<vector>
#include<hud.h>
#include<cl_util.h>
#include<const.h>
#include<com_model.h>
#include<cl_entity.h>
#include<studio.h>


enum class UserConstraintType
{
    None = 0,
    AddonJoint
};

enum class UserRigidbodyType
{
    None = 0,
    Addon
};

typedef struct ConstraintInfo_s
{
    int RigidBodyA;
    int RigidBodyB;
    btTransform LocalInA;
    btTransform LocalInB;
    btTypedConstraintType Type;
    UserConstraintType UserType;
    // cone limits
    float TwistSpan;
    float SwingSpan1;
    float SwingSpan2;
}ConstraintInfo;

typedef struct RigidbodyInfo_s
{
    int CollisionShape;
    int BoneIndex;
    /// <summary>
    /// the rigidbody local transform relatived to attached bone.
    /// RigidOffset = RigidWorldTransform * BoneWorldTransform^-1.
    /// Rigid = Offset * Bone.
    /// Bone = Offset^-1 * Rigid.
    /// </summary>
    btTransform RigidOffset;
    UserRigidbodyType UserType;
}RigidbodyInfo;

class PhysicsComponentConstructionInfo
{
public:
    PhysicsComponentConstructionInfo();
    ~PhysicsComponentConstructionInfo();

    void Parse(const char* const gpdStream,int streamLen,studiohdr_t* phdr);
    std::vector<btCollisionShape*> CollisionShapes;
    std::vector<RigidbodyInfo> RigidbodyInfos;
    std::vector<ConstraintInfo> ConstraintInfos;

    /// <summary>
    /// Studio model bones that controlled by dynamic rigidbody. (Including PhysicsAttachmen  tBones)
    /// </summary>
    std::vector<int> RigidbodyAnimatingBones;
    /// <summary>
    /// Studio model bones that controlled by animation sequences. (All bones except RigidbodyAnimationBones)
    /// </summary>
    std::vector< int> NonRigidbodyAnimatingBones;
    /// <summary>
    /// Studio model build-in attachment bones that controlled by rigidbody.
    /// </summary>
    std::vector< int> PhysicsAddonBones;
    /// <summary>
    /// Studio model bones that controlled by animation sequences. (All bones except RigidbodyAnimationBones)
    /// </summary>
    std::vector< int> NonPhysicsAddonBones;

    studiohdr_t* PStudioHeader;
};

