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
#include "extdll.h"
#include "util.h"

#include "com_model.h"
#include "studio.h"
#include "r_studioint.h"

#ifdef CLIENT_DLL
//TODO: need to put this in a header. - Solokiller
extern engine_studio_api_t IEngineStudio;
#else
#include "CStudioBlending.h"
#endif

#include "studio_util.h"
#include "StudioUtils.h"

namespace studio
{
	mstudioanim_t* GetAnim(studiohdr_t* pHeader, model_t* m_pSubModel, mstudioseqdesc_t* pseqdesc)
	{
		mstudioseqgroup_t* pseqgroup = (mstudioseqgroup_t*)((byte*)pHeader + pHeader->seqgroupindex) + pseqdesc->seqgroup;

		if (pseqdesc->seqgroup == 0)
		{
			return (mstudioanim_t*)((byte*)pHeader + pseqdesc->animindex);
		}

		cache_user_t* paSequences = (cache_user_t*)m_pSubModel->submodels;

		if (paSequences == nullptr)
		{
			paSequences = (cache_user_t*)IEngineStudio.Mem_Calloc(16, sizeof(cache_user_t)); // UNDONE: leak!
			m_pSubModel->submodels = (dmodel_t*)paSequences;
		}

		if (!IEngineStudio.Cache_Check((cache_user_t*)&(paSequences[pseqdesc->seqgroup])))
		{
			IEngineStudio.LoadCacheFile(pseqgroup->name, (cache_user_t*)&paSequences[pseqdesc->seqgroup]);
		}
		return (mstudioanim_t*)((byte*)paSequences[pseqdesc->seqgroup].data + pseqdesc->animindex);
	}

	void CalcBoneAdj(studiohdr_t* pHeader, float dadt, float* adj, const byte* pcontroller1, const byte* pcontroller2, byte mouthopen)
	{
		int					i, j;
		float				value;
		mstudiobonecontroller_t* pbonecontroller;

		pbonecontroller = (mstudiobonecontroller_t*)((byte*)pHeader + pHeader->bonecontrollerindex);

		for (j = 0; j < pHeader->numbonecontrollers; j++)
		{
			i = pbonecontroller[j].index;
			if (i <= 3)
			{
				// check for 360% wrapping
				if (pbonecontroller[j].type & STUDIO_RLOOP)
				{
					if (abs(pcontroller1[i] - pcontroller2[i]) > 128)
					{
						int a, b;
						a = (pcontroller1[j] + 128) % 256;
						b = (pcontroller2[j] + 128) % 256;
						value = ((a * dadt) + (b * (1 - dadt)) - 128) * (360.0 / 256.0) + pbonecontroller[j].start;
					}
					else
					{
						value = (pcontroller1[i] * dadt + (pcontroller2[i]) * (1.0 - dadt)) * (360.0 / 256.0) + pbonecontroller[j].start;
					}
				}
				else
				{
					value = (pcontroller1[i] * dadt + pcontroller2[i] * (1.0 - dadt)) / 255.0;
					if (value < 0) value = 0;
					if (value > 1.0) value = 1.0;
					value = (1.0 - value) * pbonecontroller[j].start + value * pbonecontroller[j].end;
				}
				// Con_DPrintf( "%d %d %f : %f\n", m_pCurrentEntity->curstate.controller[j], m_pCurrentEntity->latched.prevcontroller[j], value, dadt );
			}
			else
			{
				value = mouthopen / 64.0;
				if (value > 1.0) value = 1.0;
				value = (1.0 - value) * pbonecontroller[j].start + value * pbonecontroller[j].end;
				// Con_DPrintf("%d %f\n", mouthopen, value );
			}
			switch (pbonecontroller[j].type & STUDIO_TYPES)
			{
			case STUDIO_XR:
			case STUDIO_YR:
			case STUDIO_ZR:
				adj[j] = value * (M_PI / 180.0);
				break;
			case STUDIO_X:
			case STUDIO_Y:
			case STUDIO_Z:
				adj[j] = value;
				break;
			}
		}
	}

	void CalcBoneQuaterion(int frame, float s, mstudiobone_t* pbone, mstudioanim_t* panim, float* adj, vec4_t& q)
	{
		int					j, k;
		vec4_t			q1, q2;
		Vector				angle1, angle2;
		mstudioanimvalue_t* panimvalue;

		for (j = 0; j < 3; j++)
		{
			if (panim->offset[j + 3] == 0)
			{
				angle2[j] = angle1[j] = pbone->value[j + 3]; // default;
			}
			else
			{
				panimvalue = (mstudioanimvalue_t*)((byte*)panim + panim->offset[j + 3]);
				k = frame;
				// DEBUG
				if (panimvalue->num.total < panimvalue->num.valid)
					k = 0;
				while (panimvalue->num.total <= k)
				{
					k -= panimvalue->num.total;
					panimvalue += panimvalue->num.valid + 1;
					// DEBUG
					if (panimvalue->num.total < panimvalue->num.valid)
						k = 0;
				}
				// Bah, missing blend!
				if (panimvalue->num.valid > k)
				{
					angle1[j] = panimvalue[k + 1].value;

					if (panimvalue->num.valid > k + 1)
					{
						angle2[j] = panimvalue[k + 2].value;
					}
					else
					{
						if (panimvalue->num.total > k + 1)
							angle2[j] = angle1[j];
						else
							angle2[j] = panimvalue[panimvalue->num.valid + 2].value;
					}
				}
				else
				{
					angle1[j] = panimvalue[panimvalue->num.valid].value;
					if (panimvalue->num.total > k + 1)
					{
						angle2[j] = angle1[j];
					}
					else
					{
						angle2[j] = panimvalue[panimvalue->num.valid + 2].value;
					}
				}
				angle1[j] = pbone->value[j + 3] + angle1[j] * pbone->scale[j + 3];
				angle2[j] = pbone->value[j + 3] + angle2[j] * pbone->scale[j + 3];
			}

			if (pbone->bonecontroller[j + 3] != -1)
			{
				angle1[j] += adj[pbone->bonecontroller[j + 3]];
				angle2[j] += adj[pbone->bonecontroller[j + 3]];
			}
		}

		if (angle1 != angle2)
		{
			AngleQuaternion(angle1, q1);
			AngleQuaternion(angle2, q2);
			QuaternionSlerp(q1, q2, s, q);
		}
		else
		{
			AngleQuaternion(angle1, q);
		}
	}

	void CalcBonePosition(int frame, float s, mstudiobone_t* pbone, mstudioanim_t* panim, float* adj, Vector& vecPos)
	{
		int					j, k;
		mstudioanimvalue_t* panimvalue;

		for (j = 0; j < 3; j++)
		{
			vecPos[j] = pbone->value[j]; // default;
			if (panim->offset[j] != 0)
			{
				panimvalue = (mstudioanimvalue_t*)((byte*)panim + panim->offset[j]);
				/*
				if (i == 0 && j == 0)
				Con_DPrintf("%d  %d:%d  %f\n", frame, panimvalue->num.valid, panimvalue->num.total, s );
				*/

				k = frame;
				// DEBUG
				if (panimvalue->num.total < panimvalue->num.valid)
					k = 0;
				// find span of values that includes the frame we want
				while (panimvalue->num.total <= k)
				{
					k -= panimvalue->num.total;
					panimvalue += panimvalue->num.valid + 1;
					// DEBUG
					if (panimvalue->num.total < panimvalue->num.valid)
						k = 0;
				}
				// if we're inside the span
				if (panimvalue->num.valid > k)
				{
					// and there's more data in the span
					if (panimvalue->num.valid > k + 1)
					{
						vecPos[j] += (panimvalue[k + 1].value * (1.0 - s) + s * panimvalue[k + 2].value) * pbone->scale[j];
					}
					else
					{
						vecPos[j] += panimvalue[k + 1].value * pbone->scale[j];
					}
				}
				else
				{
					// are we at the end of the repeating values section and there's another section with data?
					if (panimvalue->num.total <= k + 1)
					{
						vecPos[j] += (panimvalue[panimvalue->num.valid].value * (1.0 - s) + s * panimvalue[panimvalue->num.valid + 2].value) * pbone->scale[j];
					}
					else
					{
						vecPos[j] += panimvalue[panimvalue->num.valid].value * pbone->scale[j];
					}
				}
			}
			if (pbone->bonecontroller[j] != -1 && adj)
			{
				vecPos[j] += adj[pbone->bonecontroller[j]];
			}
		}
	}

	void CalcRotations(studiohdr_t* pHeader, Vector* vecPos, vec4_t* q, mstudioseqdesc_t* pseqdesc, mstudioanim_t* panim, float f, float dadt, const byte* pcontroller1, const byte* pcontroller2, byte mouthopen, float framerate)
	{
		int					i;
		int					frame;
		mstudiobone_t* pbone;

		float				s;
		float				adj[MAXSTUDIOCONTROLLERS];

		if (f > pseqdesc->numframes - 1)
		{
			f = 0;	// bah, fix this bug with changing sequences too fast
		}
		// BUG ( somewhere else ) but this code should validate this data.
		// This could cause a crash if the frame # is negative, so we'll go ahead
		//  and clamp it here
		else if (f < -0.01)
		{
			f = -0.01;
		}

		frame = (int)f;

		// Con_DPrintf("%d %.4f %.4f %.4f %.4f %d\n", m_pCurrentEntity->curstate.sequence, m_clTime, m_pCurrentEntity->animtime, m_pCurrentEntity->frame, f, frame );

		// Con_DPrintf( "%f %f %f\n", m_pCurrentEntity->angles[ROLL], m_pCurrentEntity->angles[PITCH], m_pCurrentEntity->angles[YAW] );

		// Con_DPrintf("frame %d %d\n", frame1, frame2 );


		s = (f - frame);

		// add in programtic controllers
		pbone = (mstudiobone_t*)((byte*)pHeader + pHeader->boneindex);

		CalcBoneAdj(pHeader, dadt, adj, pcontroller1, pcontroller2, mouthopen);

		for (i = 0; i < pHeader->numbones; i++, pbone++, panim++)
		{
			CalcBoneQuaterion(frame, s, pbone, panim, adj, q[i]);

			CalcBonePosition(frame, s, pbone, panim, adj, vecPos[i]);
			// if (0 && i == 0)
			//	Con_DPrintf("%d %d %d %d\n", m_pCurrentEntity->curstate.sequence, frame, j, k );
		}

		if (pseqdesc->motiontype & STUDIO_X)
		{
			vecPos[pseqdesc->motionbone][0] = 0.0;
		}
		if (pseqdesc->motiontype & STUDIO_Y)
		{
			vecPos[pseqdesc->motionbone][1] = 0.0;
		}
		if (pseqdesc->motiontype & STUDIO_Z)
		{
			vecPos[pseqdesc->motionbone][2] = 0.0;
		}

		s = 0 * ((1.0 - (f - (int)(f))) / (pseqdesc->numframes)) * framerate;

		if (pseqdesc->motiontype & STUDIO_LX)
		{
			vecPos[pseqdesc->motionbone][0] += s * pseqdesc->linearmovement[0];
		}
		if (pseqdesc->motiontype & STUDIO_LY)
		{
			vecPos[pseqdesc->motionbone][1] += s * pseqdesc->linearmovement[1];
		}
		if (pseqdesc->motiontype & STUDIO_LZ)
		{
			vecPos[pseqdesc->motionbone][2] += s * pseqdesc->linearmovement[2];
		}
	}

	void SlerpBones(studiohdr_t* pHeader, vec4_t* q1, Vector* vecPos1, vec4_t* q2, const Vector* vecPos2, float s)
	{
		int			i;
		vec4_t	q3;
		float		s1;

		if (s < 0) s = 0;
		else if (s > 1.0) s = 1.0;

		s1 = 1.0 - s;

		for (i = 0; i < pHeader->numbones; i++)
		{
			QuaternionSlerp(q1[i], q2[i], s, q3);
			q1[i][0] = q3[0];
			q1[i][1] = q3[1];
			q1[i][2] = q3[2];
			q1[i][3] = q3[3];
			vecPos1[i][0] = vecPos1[i][0] * s1 + vecPos2[i][0] * s;
			vecPos1[i][1] = vecPos1[i][1] * s1 + vecPos2[i][1] * s;
			vecPos1[i][2] = vecPos1[i][2] * s1 + vecPos2[i][2] * s;
		}
	}
}