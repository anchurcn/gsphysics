#pragma once
#include<list>
#include"PhysicsComponent.h"


#define kRenderFxRagdoll 3601


/// <summary>
/// ������ͼ���ͷŴ����Ķ���
/// ������������ڽ������ֶ�����Destroy�����ͷš�
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
