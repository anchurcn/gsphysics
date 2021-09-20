#include<phy_corpse.h>
#include<cvarcfg.h>
#include<PreStudioModelRenderer.h>


extern engine_studio_api_t IEngineStudio;
extern btDynamicsWorld* DynamicWorld;


#pragma region phy_corpse.cpp

CorpseManager::CorpseManager(PhysicsAssetManager& assetManager, RagdollManager& ragdollManager)
	:_ragdollManager(ragdollManager), _assetManager(assetManager)
{
	_isDieActivity[ACT_DIESIMPLE] = true;
	_isDieActivity[ACT_DIEBACKWARD] = true;
	_isDieActivity[ACT_DIEFORWARD] = true;
	_isDieActivity[ACT_DIEVIOLENT] = true;
	_isDieActivity[ACT_DIE_HEADSHOT] = true;
	_isDieActivity[ACT_DIE_CHESTSHOT] = true;
	_isDieActivity[ACT_DIE_GUTSHOT] = true;
	_isDieActivity[ACT_DIE_BACKSHOT] = true;
}
CorpseManager::~CorpseManager(void)
{
	
}
bool CorpseManager::IsPlayingDeathSequence(studiohdr_t* phdr, int sequence)
{
	if (sequence >= phdr->numseq || sequence < 0)
	{
		return false;
	}
	auto pseqdesc = (mstudioseqdesc_t*)((byte*)phdr + phdr->seqindex) + sequence;
	return _isDieActivity[pseqdesc->activity];
}
bool CorpseManager::IsEntityAlreadyDead(cl_entity_t* ent)
{
	return _entityDead[ent->index];
}

void CorpseManager::EntityDie(cl_entity_t* ent)
{
	_entityDead[ent->index] = true;
}

void CorpseManager::EntityRespawn(cl_entity_t* ent)
{
	_entityDead[ent->index] = false;
}

#pragma endregion
