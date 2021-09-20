/***
*
*	Copyright (c) 1996-2001, Valve LLC. All rights reserved.
*
*	This product contains software technology licensed from Id
*	Software, Inc. ("Id Technology").  Id Technology (c) 1996 Id Software, Inc.
*	All Rights Reserved.
*
*   Use, distribution, and modification of this source code and/or resulting
*   object code is restricted to non-commercial enhancements to products from
*   Valve LLC.  All other use, distribution, or modification is prohibited
*   without written permission from Valve LLC.
*
****/
#ifndef GAME_SHARED_STUDIO_STUDIOUTILS_H
#define GAME_SHARED_STUDIO_STUDIOUTILS_H

namespace studio
{
	mstudioanim_t* GetAnim(studiohdr_t* pHeader, model_t* m_pSubModel, mstudioseqdesc_t* pseqdesc);

	void CalcBoneAdj(studiohdr_t* pHeader, float dadt, float* adj, const byte* pcontroller1, const byte* pcontroller2, byte mouthopen);

	void CalcBoneQuaterion(int frame, float s, mstudiobone_t* pbone, mstudioanim_t* panim, float* adj, vec4_t& q);

	void CalcBonePosition(int frame, float s, mstudiobone_t* pbone, mstudioanim_t* panim, float* adj, Vector& vecPos);

	void CalcRotations(studiohdr_t* pHeader, Vector* vecPos, vec4_t* q, mstudioseqdesc_t* pseqdesc, mstudioanim_t* panim, float f, float dadt, const byte* pcontroller1, const byte* pcontroller2, byte mouthopen, float framerate);

	void SlerpBones(studiohdr_t* pHeader, vec4_t* q1, Vector* vecPos1, vec4_t* q2, const Vector* vecPos2, float s);
}

#endif //GAME_SHARED_STUDIO_STUDIOUTILS_H