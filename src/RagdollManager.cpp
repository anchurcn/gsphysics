#include<RagdollManager.h>
#include<cvarcfg.h>


SkeletalPhysicsComponent* GetPhysComponent(cl_entity_t* ent) {
	return *(SkeletalPhysicsComponent**)&ent->curstate.iuser1;
}
void AssignPhysComponent(cl_entity_t* ent, SkeletalPhysicsComponent* h) {
	*(SkeletalPhysicsComponent**)&ent->curstate.iuser1 = h;
}
// default ragdoll update function
void RagdollUpdate(TEMPENTITY* tempent, float delta, float time);

RagdollManager::RagdollManager()
	:_componentHandles()
{
}

RagdollManager::~RagdollManager()
{
	for (auto& i : _componentHandles)
	{
		i->Disable();
		delete i;
	}
}

TEMPENTITY* RagdollManager::CreateRagdoll(cl_entity_t* pent, entity_state_t* curstate, PhysicsComponentConstructionInfo* info,btDynamicsWorld* pworld)
{
	// user data:
	// iuser1 iuser2
	// fuser
	TEMPENTITY* tempent = gEngfuncs.pEfxAPI->CL_TempEntAlloc(pent->curstate.origin, pent->model);

	tempent->entity.origin = pent->origin;
	tempent->entity.angles = pent->angles;
	tempent->entity.latched = pent->latched;

	entity_state_t* ragdollState = &tempent->entity.curstate;

	ragdollState->skin = curstate->skin;
	ragdollState->body = curstate->body;
	ragdollState->angles = curstate->angles;
	ragdollState->animtime = curstate->animtime;
	ragdollState->sequence = curstate->sequence;
	ragdollState->aiment = curstate->aiment;
	ragdollState->frame = curstate->frame;
	ragdollState->modelindex = curstate->modelindex;

	ragdollState->renderfx = kRenderFxRagdoll;

	tempent->flags |= FTENT_CLIENTCUSTOM | FTENT_PERSIST;
	tempent->callback = RagdollUpdate;
	// custom life time
	ragdollState->fuser1 = gEngfuncs.GetClientTime() + phys_corpsestay->value;
	tempent->die = FLT_MAX;

	SkeletalPhysicsComponent* component = new SkeletalPhysicsComponent(false, info, pworld);
	component->Disable();
	AssignPhysComponent(&tempent->entity, component);
	component->SetOwner(&tempent->entity);

	cl_entity_t* local = gEngfuncs.GetLocalPlayer();
	Vector v = (pent->origin - local->origin).Normalize();
	v = v * 5;
	//gPhysics.SetVelocity(h, (float*)&v);

	gEngfuncs.Con_DPrintf("corpse [%d]'s velocity is %f\n", tempent->entity.index, pent->curstate.velocity.Length());
	gEngfuncs.Con_DPrintf("create corpse [%d] for entity [%d]\n", tempent->entity.index, pent->index);
	_componentHandles.push_back(component);
	return tempent;
}
TEMPENTITY* RagdollManager::CreateRagdollCustom()
{
	// TODO:
	return nullptr;
}
void RagdollManager::DestroyRagdoll(SkeletalPhysicsComponent* component)
{
	delete component;
	_componentHandles.remove(component);
}
bool RagdollManager::IsRagdoll(cl_entity_t* ent)
{
	return ent->curstate.renderfx == kRenderFxRagdoll;
}
SkeletalPhysicsComponent* RagdollManager::GetComponent(cl_entity_t* pent)
{
	return GetPhysComponent(pent);
}