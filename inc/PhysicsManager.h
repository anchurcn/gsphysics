#pragma once
#include<list>
#include<phy_corpse.h>
#include"PhysicsComponent.h"
#include"KinematicManager.h"


#pragma region PhysicsManager

typedef struct {
    cl_entity_t* pent;
    int type;
}VisEnt;

class PhysicsManager
{
public:
    PhysicsAssetManager* PAssetManager;
    RagdollManager* PRagdollManager;
    KinematicManager* PServerEntManager;
    CorpseManager* PCorpseManager;

public:
    PhysicsManager();
    ~PhysicsManager();

    void Init();
    void NewMap();
    void AddEntity(cl_entity_t* pent, int entType);
    void MapReset();
    void Update(float currentTime, float deltaTime);
    void Draw();
    
    //SkeletalPhysicsComponent* GetPhysComponent(cl_entity_t* pent);
private:
    btBroadphaseInterface* m_broadphase;
    btCollisionDispatcher* m_dispatcher;
    btConstraintSolver* m_solver;
    btDefaultCollisionConfiguration* m_collisionConfiguration;
    btDiscreteDynamicsWorld* m_dynamicsWorld;
    std::vector<VisEnt> _visEntities;
};

#pragma endregion



extern PhysicsManager gPhysicsManager;