//==========================================================================
//
//==========================================================================

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

#include "StudioModelRenderer.h"
#include "PhyStudioModelRenderer.h"
#include<StudioModelRenderer.h>
#include "Exports.h"


#include<PhysicsManager.h>
#include<cvarcfg.h>

//SkeletalPhysicsComponent* GetPhysComponent(cl_entity_t* ent) {
//	return *(SkeletalPhysicsComponent**)&ent->curstate.iuser1;
//}
//void AssignPhysComponent(cl_entity_t* ent, SkeletalPhysicsComponent* h) {
//	*(SkeletalPhysicsComponent**)&ent->curstate.iuser1 = h;
//}
//
//GCHandle GetPhysWeaponComponent(cl_entity_t* ent) {
//	// TODO: get from kinematic sub component
//	return *(GCHandle*)&ent->prevstate.iuser1;
//}
//void AssignPhysWeaponComponent(cl_entity_t* ent, GCHandle h) {
//	*(GCHandle*)&ent->prevstate.iuser1 = h;
//}
typedef struct {
	byte a, r, g, b;
}color32;
typedef struct ColoredPos
{
	vec3_t pos;
	color32 color;
}ColoredPos_t;
extern "C" {
	DLLEXPORT void DrawBufferedLines(ColoredPos_t* buffer, int elementCount);
	DLLEXPORT void DrawBufferedLines(ColoredPos_t* buffer, int elementCount) {
		gEngfuncs.pTriAPI->Brightness(1.0f);

		gEngfuncs.pTriAPI->Begin(TRI_LINES);
		for (size_t i = 0; i < elementCount; i ++)
		{
			gEngfuncs.pTriAPI->Color4ub(buffer[i].color.r, buffer[i].color.g, buffer[i].color.b, buffer[i].color.a);
			gEngfuncs.pTriAPI->Vertex3fv((buffer[i].pos));

		}
		gEngfuncs.pTriAPI->End();
	}
}
//
// Override the StudioModelRender virtual member functions here to implement custom bone
// setup, blending, etc.
//

// Global engine <-> studio model rendering code interface
extern engine_studio_api_t IEngineStudio;

// The renderer object, created on the stack.
PhyStudioModelRenderer g_StudioRenderer;
/*
====================
PhyStudioModelRenderer

====================
*/
PhyStudioModelRenderer::PhyStudioModelRenderer(void)
{
}
void PhyStudioModelRenderer::Init(void)
{
	CStudioModelRenderer::Init();
	gPhysicsManager.Init();
	if (!IEngineStudio.IsHardware())
	{
		gEngfuncs.Con_Printf("Please run on hardware mode.\n");
	}
}
int PhyStudioModelRenderer::StudioDrawPhysModel(int flags)
{
	alight_t lighting;
	vec3_t dir;

	IEngineStudio.GetTimes(&m_nFrameCount, &m_clTime, &m_clOldTime);
	IEngineStudio.GetViewInfo(m_vRenderOrigin, m_vUp, m_vRight, m_vNormal);
	IEngineStudio.GetAliasScale(&m_fSoftwareXScale, &m_fSoftwareYScale);

	m_pRenderModel = m_pCurrentEntity->model;
	m_pStudioHeader = (studiohdr_t*)IEngineStudio.Mod_Extradata(m_pRenderModel);
	IEngineStudio.StudioSetHeader(m_pStudioHeader);
	IEngineStudio.SetRenderModel(m_pRenderModel);
	auto component = (SkeletalPhysicsComponent*)GetPhysComponent(m_pCurrentEntity, &m_pCurrentEntity->curstate);
	component->GetWorldTransform((float(*)[4])m_protationmatrix);

	if (flags & STUDIO_RENDER)
	{
		// see if the bounding box lets us trivially reject, also sets
		// TODO: get aabb from physics component and check by myself
		if (!IEngineStudio.StudioCheckBBox())
			return 0;

		(*m_pModelsDrawn)++;
		(*m_pStudioModelCount)++; // render data cache cookie

		if (m_pStudioHeader->numbodyparts == 0)
			return 1;
	}
//	// setup bones for different kinds phys_model
//	if (m_pCurrentEntity == IEngineStudio.GetViewEntity())
//	{
//		// 因为 SetPoseViewComponent 的时候并不是最新的位置
//		CStudioModelRenderer::StudioDrawModel(0);
//		//gpPhysicsManager->SetupBonesForViewModel(*m_pbonetransform);
//	}
//	else if (m_pCurrentEntity->index>0)
//	{
//		// setup kinematic model
//		gPhysics.StudioKinematicComponentSetupBones(m_pCurrentEntity->index);
//	}
//	else if(pgCorpseMgr->IsRagdollCorpse(m_pCurrentEntity))
//	{
//#define MOVETYPE_RAGDOLL 3601
//		if (m_pCurrentEntity->curstate.movetype != MOVETYPE_RAGDOLL)
//			return 0;
//		gPhysics.SetupBones(GetPhysComponent(m_pCurrentEntity));
//	}
//	//else if (m_pCurrentEntity->baseline.iuser1 == XXXPhysType)
//	//{
//	//	gPhysics.SetupBones((GCHandle)m_pCurrentEntity->baseline.iuser2);
//	//}
//	else
//	{
//		// other temp entity
//		return CStudioModelRenderer::StudioDrawModel(flags);
//	}
	
	component->SetupBones(*m_pbonetransform);
	StudioSaveBones();

	// setup light transform
	for (int i = 0; i < m_pStudioHeader->numbones; i++)
		MatrixCopy((*m_pbonetransform)[i], (*m_plighttransform)[i]);


	if (flags & STUDIO_EVENTS)
	{
		StudioCalcAttachments();
		IEngineStudio.StudioClientEvents();
		// copy attachments into global entity array
		if (m_pCurrentEntity->index > 0)
		{
			cl_entity_t* ent = gEngfuncs.GetEntityByIndex(m_pCurrentEntity->index);

			memcpy(ent->attachment, m_pCurrentEntity->attachment, sizeof(vec3_t) * 4);
		}
	}

	if (flags & STUDIO_RENDER)
	{
		lighting.plightvec = dir;
		IEngineStudio.StudioDynamicLight(m_pCurrentEntity, &lighting);

		IEngineStudio.StudioEntityLight(&lighting);

		// model and frame independant
		IEngineStudio.StudioSetupLighting(&lighting);

		m_nTopColor = m_pCurrentEntity->curstate.colormap & 0xFF;
		m_nBottomColor = (m_pCurrentEntity->curstate.colormap & 0xFF00) >> 8;

		IEngineStudio.StudioSetRemapColors(m_nTopColor, m_nBottomColor);

		StudioRenderModel();
	}
	return 1;
}
bool PhyStudioModelRenderer::IsPhysAnimating(cl_entity_t* pent, entity_state_t* curstate)
{
	if (pent == IEngineStudio.GetViewEntity())
	{
		return false;
	}
	else if (pent->index > 0)
	{
		return gPhysicsManager.PServerEntManager->IsPhysAnimating(pent);
	}
	else
	{
		return gPhysicsManager.PRagdollManager->IsRagdoll(pent);
	}
}
void* PhyStudioModelRenderer::GetPhysComponent(cl_entity_t* pent, entity_state_t* curstate)
{
	if (pent == IEngineStudio.GetViewEntity())
	{
		return NULL;
	}
	else if (pent->index > 0)
	{
		return gPhysicsManager.PServerEntManager->GetPhysComponent(pent);
	}
	else
	{
		return gPhysicsManager.PRagdollManager->GetComponent(pent);
	}
	return NULL;
}
int PhyStudioModelRenderer::StudioDrawModelEx(int flags)
{
	if (m_pCurrentEntity->curstate.renderfx == kRenderFxDeadPlayer)
		return 0;

	// 过滤掉已经创建有布娃娃代替它了的实体,返回 0 告诉引擎当前模型没渲染，
	// 引擎通过返回值决定是否绘制当前引擎的子实体，返回 0 表示不渲染。
	if (gPhysicsManager.PCorpseManager->IsEntityAlreadyDead(m_pCurrentEntity))
	{
		CStudioModelRenderer::StudioDrawModel(flags & ~STUDIO_RENDER);
		return 0;
	}

	if (IsPhysAnimating(m_pCurrentEntity, &m_pCurrentEntity->curstate))
		return StudioDrawPhysModel(flags);
	else
		return CStudioModelRenderer::StudioDrawModel(flags);
	 
	return 0;

	 


	//// view ent
	//if (m_pCurrentEntity == IEngineStudio.GetViewEntity())
	//{
	//	// view model 会以 index == 1 的实体出现，需要提前过滤
	//	bool supportPhysAnimating = false;//TODO
	//	if (supportPhysAnimating)
	//		return CStudioModelRenderer::StudioDrawModel(flags);
	//	else
	//		return CStudioModelRenderer::StudioDrawModel(flags);
	//}
	//// index > 0 的按照约定视为服务器实体
	//else if (m_pCurrentEntity->index > 0)
	//{
	//	// setup kinematic model
	//	if (gPhysics.IsPhysAnimating(m_pCurrentEntity->curstate.modelindex))
	//		return StudioDrawPhysModel(flags);
	//	else
	//		return CStudioModelRenderer::StudioDrawModel(flags);
	//}
	//// the rest ent
	//else if (pgCorpseMgr->IsRagdollCorpse(m_pCurrentEntity))
	//{
	//	return StudioDrawPhysModel(flags);
	//}
	//else
	//{
	//	// other temp entity
	//	return CStudioModelRenderer::StudioDrawModel(flags);
	//}
}
int PhyStudioModelRenderer::StudioDrawModel(int flags)
{
	m_pCurrentEntity = IEngineStudio.GetCurrentEntity();
	m_pRenderModel = m_pCurrentEntity->model;
	m_pStudioHeader = (studiohdr_t*)IEngineStudio.Mod_Extradata(m_pRenderModel);

	return StudioDrawModelEx(flags);
}

int PhyStudioModelRenderer::StudioDrawPlayer(int flags, entity_state_s* pplayer)
{
	m_pCurrentEntity = IEngineStudio.GetCurrentEntity();
	if (gPhysicsManager.PCorpseManager->IsEntityAlreadyDead(m_pCurrentEntity))
	{
		CStudioModelRenderer::StudioDrawPlayer(flags & ~STUDIO_RENDER, pplayer);
		return 0;
	}
	if (phys_jiggle->value && m_pCurrentEntity->index > 0 && IsPhysAnimating(m_pCurrentEntity,pplayer))
	{
		alight_t lighting;
		vec3_t dir;

		IEngineStudio.GetTimes(&m_nFrameCount, &m_clTime, &m_clOldTime);
		IEngineStudio.GetViewInfo(m_vRenderOrigin, m_vUp, m_vRight, m_vNormal);
		IEngineStudio.GetAliasScale(&m_fSoftwareXScale, &m_fSoftwareYScale);

		m_nPlayerIndex = pplayer->number - 1;

		if (m_nPlayerIndex < 0 || m_nPlayerIndex >= gEngfuncs.GetMaxClients())
			return 0;

		m_pRenderModel = IEngineStudio.SetupPlayerModel(m_nPlayerIndex);
		if (m_pRenderModel == NULL)
			return 0;

		m_pStudioHeader = (studiohdr_t*)IEngineStudio.Mod_Extradata(m_pRenderModel);
		IEngineStudio.StudioSetHeader(m_pStudioHeader);
		IEngineStudio.SetRenderModel(m_pRenderModel);
		auto component = (SkeletalPhysicsComponent*)GetPhysComponent(m_pCurrentEntity, pplayer);
		component->GetWorldTransform((float(*)[4])m_protationmatrix);
		
		if (flags & STUDIO_RENDER)
		{
			// see if the bounding box lets us trivially reject, also sets
			if (!IEngineStudio.StudioCheckBBox())
				return 0;

			(*m_pModelsDrawn)++;
			(*m_pStudioModelCount)++; // render data cache cookie

			if (m_pStudioHeader->numbodyparts == 0)
				return 1;
		}

		m_pPlayerInfo = IEngineStudio.PlayerInfo(m_nPlayerIndex);
		
		component->SetupBones(*m_pbonetransform);
		StudioSaveBones();

		// setup light transform
		for (int i = 0; i < m_pStudioHeader->numbones; i++)
			MatrixCopy((*m_pbonetransform)[i], (*m_plighttransform)[i]);

		StudioSaveBones();
		m_pPlayerInfo->renderframe = m_nFrameCount;

		m_pPlayerInfo = NULL;

		if (flags & STUDIO_EVENTS)
		{
			StudioCalcAttachments();
			IEngineStudio.StudioClientEvents();
			// copy attachments into global entity array
			if (m_pCurrentEntity->index > 0)
			{
				cl_entity_t* ent = gEngfuncs.GetEntityByIndex(m_pCurrentEntity->index);

				memcpy(ent->attachment, m_pCurrentEntity->attachment, sizeof(vec3_t) * 4);
			}
		}

		if (flags & STUDIO_RENDER)
		{
			if (m_pCvarHiModels->value && m_pRenderModel != m_pCurrentEntity->model)
			{
				// show highest resolution multiplayer model
				m_pCurrentEntity->curstate.body = 255;
			}

			if (!(m_pCvarDeveloper->value == 0 && gEngfuncs.GetMaxClients() == 1) && (m_pRenderModel == m_pCurrentEntity->model))
			{
				m_pCurrentEntity->curstate.body = 1; // force helmet
			}

			lighting.plightvec = dir;
			IEngineStudio.StudioDynamicLight(m_pCurrentEntity, &lighting);

			IEngineStudio.StudioEntityLight(&lighting);

			// model and frame independant
			IEngineStudio.StudioSetupLighting(&lighting);

			m_pPlayerInfo = IEngineStudio.PlayerInfo(m_nPlayerIndex);

			// get remap colors
			m_nTopColor = m_pPlayerInfo->topcolor;
			m_nBottomColor = m_pPlayerInfo->bottomcolor;

			// bounds check
			if (m_nTopColor < 0)
				m_nTopColor = 0;
			if (m_nTopColor > 360)
				m_nTopColor = 360;
			if (m_nBottomColor < 0)
				m_nBottomColor = 0;
			if (m_nBottomColor > 360)
				m_nBottomColor = 360;

			IEngineStudio.StudioSetRemapColors(m_nTopColor, m_nBottomColor);

			StudioRenderModel();
			m_pPlayerInfo = NULL;

			if (pplayer->weaponmodel)
			{
				cl_entity_t saveent = *m_pCurrentEntity;

				model_t* pweaponmodel = IEngineStudio.GetModelByIndex(pplayer->weaponmodel);
				m_pStudioHeader = (studiohdr_t*)IEngineStudio.Mod_Extradata(pweaponmodel);
				IEngineStudio.StudioSetHeader(m_pStudioHeader);

				//// get weapon phys component from player entity
				//auto component = (SkeletalPhysicsComponent*)NULL;//TODO:
				//// setup bones for weapon
				//if (component)
				//	component->SetupBones(*m_pbonetransform);
				//else
					StudioMergeBones(pweaponmodel);

				IEngineStudio.StudioSetupLighting(&lighting);

				StudioRenderModel();

				StudioCalcAttachments();

				*m_pCurrentEntity = saveent;
			}
		}

		return 1;
	}

	return CStudioModelRenderer::StudioDrawPlayer(flags, pplayer);
}

////////////////////////////////////
// Hooks to class implementation
////////////////////////////////////

/*
====================
R_StudioDrawPlayer

====================
*/
int R_StudioDrawPlayer(int flags, entity_state_t* pplayer)
{
	return g_StudioRenderer.StudioDrawPlayer(flags, pplayer);
}

/*
====================
R_StudioDrawModel

====================
*/
int R_StudioDrawModel(int flags)
{
	return g_StudioRenderer.StudioDrawModel(flags);
}

/*
====================
R_StudioInit

====================
*/
void R_StudioInit(void)
{
	g_StudioRenderer.Init();
}

// The simple drawing interface we'll pass back to the engine
r_studio_interface_t studio =
{
	STUDIO_INTERFACE_VERSION,
	R_StudioDrawModel,
	R_StudioDrawPlayer,
};

/*
====================
HUD_GetStudioModelInterface

Export this function for the engine to use the studio renderer class to render objects.
====================
*/
int DLLEXPORT HUD_GetStudioModelInterface(int version, struct r_studio_interface_s** ppinterface, struct engine_studio_api_s* pstudio)
{
	//	RecClStudioInterface(version, ppinterface, pstudio);

	if (version != STUDIO_INTERFACE_VERSION)
		return 0;

	// Point the engine to our callbacks
	*ppinterface = &studio;

	// Copy in engine helper functions
	memcpy(&IEngineStudio, pstudio, sizeof(IEngineStudio));

	// Initialize local variables, etc.
	R_StudioInit();

	// Success
	return 1;
}

