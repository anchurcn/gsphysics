#include"PhysicsAssetsManager.h"
#include"../gsphysics/tinyxml2.h"
#include<cvarcfg.h>
#include<BulletCollision/Gimpact/btGImpactShape.h>
#include<ConstructionInfo.h>
#include<APIProxy.h>
#include<cl_dll.h>
#include<r_studioint.h>

#include<btBulletCollisionCommon.h>
#include<btBulletDynamicsCommon.h>
#include "hud.h"
#include "cl_util.h"
#include "const.h"
#include "com_model.h"
#include "cl_entity.h"
#include<studio.h>


extern engine_studio_api_t IEngineStudio;

PhysicsComponentConstructionInfo* LoadConstructionInfoByPath(const char* gpdPath,studiohdr_t* phdr)
{
	int length = 0;
	auto memStream = gEngfuncs.COM_LoadFile(gpdPath, 5, &length);
	if (memStream == nullptr)
	{
		return nullptr;
	}
	else
	{
		auto pInfo = new PhysicsComponentConstructionInfo();
		pInfo->Parse((char*)memStream, length, phdr);
		return pInfo;
	}
}

int* GetTriangleIndeces(model_t* mod, std::vector<int>& indexPool, int& outIndecesCount)
{
	int oldsize = indexPool.size();
	int* pbase = &indexPool.data()[oldsize];

	msurface_t* surfaces = mod->surfaces;
	int* surfedges = mod->surfedges;
	medge_t* edges = mod->edges;

	int firstface = mod->firstmodelsurface;
	int numfaces = mod->nummodelsurfaces;

	// foreach surface in model
	for (int faceIndex = 0; faceIndex < numfaces; faceIndex++)
	{
		auto& surface = surfaces[firstface + faceIndex];
		// foreach edge in surface
		// triagulate the surface
		// triangle fan to triangle
		{
			int vertexIndex[3];
			{
				int signedIndex = surfedges[surface.firstedge + 0];
				if (signedIndex > 0)
					vertexIndex[0] = edges[signedIndex].v[0];
				else
					vertexIndex[0] = edges[-signedIndex].v[1];

				signedIndex = surfedges[surface.firstedge + 1];
				if (signedIndex > 0)
					vertexIndex[1] = edges[signedIndex].v[0];
				else
					vertexIndex[1] = edges[-signedIndex].v[1];
			}
			for (int edgeIndex = 2; edgeIndex < surface.numedges; edgeIndex++)
			{
				int signedIndex = surfedges[surface.firstedge + edgeIndex];

				if (signedIndex > 0)
					vertexIndex[2] = edges[signedIndex].v[0];
				else
					vertexIndex[2] = edges[-signedIndex].v[1];

				indexPool.push_back(vertexIndex[0]);
				indexPool.push_back(vertexIndex[1]);
				indexPool.push_back(vertexIndex[2]);
				vertexIndex[1] = vertexIndex[2];
			}
		}

	}

	outIndecesCount = indexPool.size() - oldsize;
	return pbase;
}

PhysicsAssetManager::PhysicsAssetManager() 
	:_indexPool(32 * 65535),
	_lookupTable(1025)
{
	_indexPool.clear();

	int i = 0;
	model_t* pmodel = gEngfuncs.hudGetModelByIndex(1);

	_vertexPool.resize(pmodel->numvertexes);
	btScalar* pp = (btScalar*)_vertexPool.data();
	btScalar* p = (btScalar*)pmodel->vertexes;
	for (size_t i = 0; i < pmodel->numvertexes * 3; i++)
	{
		pp[i] = p[i] * G2BScale();
	}

	for (i = 1, pmodel = gEngfuncs.hudGetModelByIndex(i);
		i < _lookupTable.size() && pmodel != NULL;
		i++, pmodel = gEngfuncs.hudGetModelByIndex(i))
	{
		LoadModelByIndex(i);
	}
	// can do the player asset preloading, 
	// but will load to many unused item slowing down our search.
}

PhysicsAssetManager::~PhysicsAssetManager()
{
}
/// <summary>
/// get player model.
/// </summary>
/// <param name="modelName">playerinfo.model</param>
/// <returns></returns>
model_t* GetPlayerModel(const char* modelName)
{
	// no model
	if (modelName[0] == 0)
		return NULL;

	if (strcmp(modelName, "player") == 0)
	{
		return IEngineStudio.Mod_ForName("models/player.mdl", true);
	}
	else
	{
		char tempname[256];
		sprintf_s(tempname, sizeof(tempname), "models/player/%s/%s.mdl", modelName, modelName);
		auto mod = IEngineStudio.Mod_ForName(tempname, false);
		return mod ? mod : IEngineStudio.Mod_ForName("models/player.mdl", true);
	}
}
bool PhysicsAssetManager::IsPlayerAssetAvailable(char const* const modelName, int& outIndex)
{
	if (modelName[0] == '\0')
		return false;

	for (size_t i = 0; i < _playerAssets.size(); i++)
	{
		auto item = _playerAssets[i];
		if (strcmp(item.Name, modelName) == 0)
		{
			outIndex = i;
			return item.Asset != NULL;
		}
	}

	// load for player asset, may be NULL.
	_playerAssets.push_back({ 0,0 });
	outIndex = _playerAssets.size() - 1;
	TupleNameAsset& cacheItem = _playerAssets[outIndex];
	strcpy_s(cacheItem.Name, 64, modelName);

	char tempname[256];
	auto phdr = (studiohdr_t*)IEngineStudio.Mod_Extradata(GetPlayerModel(modelName));
	if (phdr) 
	{
		if (strcmp(modelName, "player") == 0)
		{
			cacheItem.Asset = LoadConstructionInfoByPath("models/player.gpd", phdr);
		}
		else
		{
			sprintf_s(tempname, sizeof(tempname), "models/player/%s/%s.gpd", modelName, modelName);
			cacheItem.Asset = LoadConstructionInfoByPath(tempname, phdr);
		}
	}
	if (cacheItem.Asset)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void* PhysicsAssetManager::GetPlayerPhysicsAsset(int index)
{
	return _playerAssets[index].Asset;
}

void* PhysicsAssetManager::GetPlayerPhysicsAsset(const char* modelName)
{
	for (size_t i = 0; i < _playerAssets.size(); i++)
	{
		auto item = _playerAssets[i];
		if (strcmp(item.Name, modelName) == 0)
		{
			return item.Asset;
		}
	}
	return nullptr;
}

bool PhysicsAssetManager::IsAssetAvailable(int modelIndex)
{
	if (!_lookupTable[modelIndex].Valid)
		LoadModelByIndex(modelIndex);
	return _lookupTable[modelIndex].Asset != NULL;
}

void* PhysicsAssetManager::GetPhysicsAsset(int modelIndex)
{
	return _lookupTable[modelIndex].Asset;
}

bool PhysicsAssetManager::IsAssetAvailable(char const* const modelName)
{
	int index = 0;
	if (gEngfuncs.CL_LoadModel(modelName, &index))
		return IsAssetAvailable(index);
	else
		return false;
}

void* PhysicsAssetManager::GetPhysicsAsset(char const* const modelName)
{
	int index = 0;
	gEngfuncs.CL_LoadModel(modelName, &index);
	return GetPhysicsAsset(index);
}

void PhysicsAssetManager::LoadModelByIndex(int modelIndex)
{
	void* result = nullptr;
	auto mod = gEngfuncs.hudGetModelByIndex(modelIndex);
	if (mod->type == modtype_t::mod_brush)
	{
		{// use gimpact instead?
			int indexCount = 0;
			auto pIndeces = GetTriangleIndeces(mod, _indexPool, indexCount);
			auto vertexArray = new btTriangleIndexVertexArray(indexCount / 3, pIndeces, 12, mod->numvertexes, (btScalar*)_vertexPool.data(), 12);
			btBvhTriangleMeshShape* meshShape = new btBvhTriangleMeshShape(vertexArray, true);
			result = meshShape;
		}
	}
	else if (mod->type == modtype_t::mod_studio)
	{
		char temppath[64];
		strcpy(temppath, mod->name);
		int i = strlen(temppath)-3;
		temppath[i++] = 'g';
		temppath[i++] = 'p';
		temppath[i] = 'd';

		auto phdr = (studiohdr_t*)IEngineStudio.Mod_Extradata(mod);
		result = LoadConstructionInfoByPath(temppath,phdr);
	}

	_lookupTable[modelIndex].Valid = true;
	_lookupTable[modelIndex].Asset = result;
}

