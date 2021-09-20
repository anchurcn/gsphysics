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
/// Ϊ������ʵ����������������Ϊ������ʵ���е�״̬��ȫ�ɷ������·�����Ϣ���ƣ�
/// �ͻ����޷�ʹ��ʵ���ڵ��ֶδ洢��Ϣ������ʹ�ù����沢�ṩ�򵥵��߼��������������
/// ͨ��ʵ�������������Ӧ�����������
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