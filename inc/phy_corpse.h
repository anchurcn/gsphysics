#pragma once
#include<activity.h>
#include"PhysicsComponent.h"
#include"RagdollManager.h"


#define MAX_ENTITIES 512



/// <summary>
/// 布娃娃尸体是临时实体，本类只管布娃娃使用的物理组件的创建和删除。
/// 本类记录了死亡的服务器实体，通过临时实体回调控制其生命周期，通过设置flag控制
/// TempEntUpdate移除临时实体。
/// </summary>
class CorpseManager
{
public:
	CorpseManager(PhysicsAssetManager& assetManager,RagdollManager& ragdollManager);
	~CorpseManager(void);

	bool AddPlayerEntity(cl_entity_t* pent);
	bool AddNormalEntity(cl_entity_t* pent);
	
private:
	// max server side entity count elements
	bool _entityDead[MAX_ENTITIES] = { false };
	bool _isDieActivity[ACT_FLINCH_RIGHTLEG + 1] = { false };
	
	PhysicsAssetManager& _assetManager;
	RagdollManager& _ragdollManager;
public:
	bool IsPlayingDeathSequence(studiohdr_t* phdr, int sequence);

	// check if the entity is already dead
	bool IsEntityAlreadyDead(cl_entity_t* ent);

	// tells the mgr that the entity died just now.
	void EntityDie(cl_entity_t* ent);

	// if entity plays any sequences other than death sequences,
	// we tells the mgr this entity is alive.
	void EntityRespawn(cl_entity_t* ent);
};
