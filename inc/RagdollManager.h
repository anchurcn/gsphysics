#pragma once
#include<list>
#include"PhysicsComponent.h"


#define kRenderFxRagdoll 3601


/// <summary>
/// 更换地图会释放创建的对象。
/// 对象的生命周期结束可手动调用Destroy函数释放。
/// </summary>
class RagdollManager
{
public:
	RagdollManager();
	~RagdollManager();

	TEMPENTITY* CreateRagdoll(cl_entity_t* pent,entity_state_t* curstate,PhysicsComponentConstructionInfo* info,btDynamicsWorld* pworld);
	// custom life time, by provided update function.
	TEMPENTITY* CreateRagdollCustom();
	void DestroyRagdoll(SkeletalPhysicsComponent* component);
	bool IsRagdoll(cl_entity_t* ent);
	SkeletalPhysicsComponent* GetComponent(cl_entity_t* pent);

private:
	std::list<SkeletalPhysicsComponent*> _componentHandles;
};
