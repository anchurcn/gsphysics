#pragma once
#include<array>
#include<vector>
#include<btBulletDynamicsCommon.h>

typedef struct
{
	char Name[64];
	void* Asset;
}TupleNameAsset;

typedef struct
{
	bool Valid;
	void* Asset;
}ModelPhysicsItem;


// 對每個模型，在調用GetXXX之前至少要調用一次IsXXXAvailable。
class PhysicsAssetManager
{
public:
	PhysicsAssetManager();
	~PhysicsAssetManager();

	bool IsAssetAvailable(int modelIndex);
	// return type: PhysicsComponentConstructionInfo* or CollisionShape*
	void* GetPhysicsAsset(int modelIndex);
	// modelPath usually model_t::name
	bool IsAssetAvailable(char const* const modelPath);
	void* GetPhysicsAsset(char const* const modelPath);
	// momdelName usually player_info_t::model
	bool IsPlayerAssetAvailable(char const* const modelName, int& outIndex);
	// return type: PhysicsComponentConstructionInfo*
	void* GetPlayerPhysicsAsset(int index);
	void* GetPlayerPhysicsAsset(const char* modelName);
private:

	std::vector<btVector3>	_vertexPool;
	std::vector<int>		_indexPool;
	std::vector<ModelPhysicsItem> _lookupTable;
	std::vector<TupleNameAsset> _playerAssets;
	void LoadModelByIndex(int modelIndex);
};