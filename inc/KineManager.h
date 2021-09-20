#pragma once
#include"PhysicsAssetsManager.h"
#include"PhysicsComponent.h"

class KineManager
{
public:
	KineManager(PhysicsAssetManager& assetManager);
	~KineManager();

private:
	PhysicsAssetManager& _assetManager;
};
