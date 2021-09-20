// Author: Anchur

#if !defined( PHYSTUDIOMODELRENDERER_H )
#define PHYSTUDIOMODELRENDERER_H
#if defined( _WIN32 )
#pragma once
#endif

#include<StudioModelRenderer.h>
/*
====================
PhyStudioModelRenderer

====================
*/
class PhyStudioModelRenderer : public CStudioModelRenderer
{
public:
	PhyStudioModelRenderer(void);
	int StudioDrawModelEx(int flags);
	// override public interfaces
	virtual int StudioDrawModel(int flags);
	virtual int StudioDrawPlayer(int flags, struct entity_state_s* pplayer);

	virtual void Init(void);
	virtual int StudioDrawPhysModel(int flags);
	virtual bool IsPhysAnimating(cl_entity_t* pent, entity_state_t* curstate);
	virtual void* GetPhysComponent(cl_entity_t* pent, entity_state_t* curstate);
};

#endif // PHYSTUDIOMODELRENDERER_H