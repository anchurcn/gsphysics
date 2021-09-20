#pragma once
#include<list>
#include"PhysicsComponent.h"


typedef struct KinematicTableItem_s
{
    /// <summary>
    /// Kinematic SkeletalPhysicsComponent for studio model,
    /// Kinematic RigidBody for brush model.
    /// </summary>
    IPhysicsComponent* KinematicComponent;
    /// <summary>
    /// The model corresponding to the component.
    /// </summary>
    int CorrespondingModel;
    bool IsInWorld;
}KinematicTableItem;


typedef struct PlayerComponentInfo_s
{
    // this is the latest infomation, updated just now
    // if not, remove the component if it is in world
    bool Updated;
    // component has already in world
    bool IsInWorld;
    // if the model support phys, not NULL
    SkeletalPhysicsComponent* Component;
    char ModelName[MAX_QPATH] = "not initialized name @a0.0! this cannot be a model name.";
}PlayerComponentInfo;


/// <summary>
/// 为服务器实体管理物理组件。因为服务器实体中的状态完全由服务器下发的信息控制，
/// 客户端无法使用实体内的字段存储信息，所以使用管理储存并提供简单的逻辑创建物理组件。
/// 通过实体索引访问其对应的物理组件。
/// </summary>
class KinematicManager
{
public:
    KinematicManager(PhysicsAssetManager& assetManager);
    ~KinematicManager();
    /// <summary>
    /// Add kinematic for server live entity 
    /// Add ragdoll for dead entity 
    /// </summary>
    /// <param name="pent"></param>
    void AddNormal(cl_entity_t* pent);
    void AddPlayer(cl_entity_t* pent);
    void BeforeSimulation(float currentTime, float deltaTime);
    void AfterSimulation();
    bool IsPhysAnimating(cl_entity_t* pent);
    SkeletalPhysicsComponent* GetPhysComponent(cl_entity_t* pent);
    btRigidBody* GetWorldSpawn();
private:

    PhysicsAssetManager& _assetManager;

    std::list<int> _added;
    std::list<int> _old;
    std::vector<KinematicTableItem*> _lookupTable;
    std::vector<PlayerComponentInfo> _playerComponents;

    IPhysicsComponent* MakeNewComponent(cl_entity_t* pent);
};
#pragma endregion