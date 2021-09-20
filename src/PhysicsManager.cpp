#include<PhysicsManager.h>
#include<PreStudioModelRenderer.h>
#include<cvarcfg.h>
#include<BulletUtils.h>
#include<gl/GL.h>
#include<entity_types.h>


#pragma comment(lib,"opengl32.lib")

btDynamicsWorld* DynamicWorld;
PreStudioModelRenderer PreModelRenderer;
PhysicsManager gPhysicsManager;
float gCurrentTime;
float gDeltaTime;
model_t* GetPlayerModel(const char* modelName);
// return: Len of string.
int DrawSprString_Internal(int x, int y, const char* str)
{
	auto triapi = gEngfuncs.pTriAPI;
	int imgsize = 512;
	float charHeight = imgsize / 16;
	float charGap = 16;
	float ugap = charGap / (float)imgsize;
	float uvSize = 1 / 16.f;
	HSPRITE font = SPR_Load("sprites/font.spr");
	triapi->SpriteTexture((model_t*)gEngfuncs.GetSpritePointer(font), 0);
	triapi->RenderMode(kRenderTransAdd);
	triapi->Color4f(1, 1, 1, 1);
	triapi->Brightness(1);
	triapi->Begin(TRI_QUADS);
	int i = 0;
	for (i = 0; str[i]; i++)
	{
		float u = ((unsigned char)str[i] & 0xF) * uvSize;
		float v = ((unsigned char)str[i] >> 4) * uvSize;

		triapi->TexCoord2f(u, v);
		triapi->Vertex3f(x, y, 0);

		triapi->TexCoord2f(u, v + uvSize);
		triapi->Vertex3f(x, y + charHeight, 0);

		triapi->TexCoord2f(u + ugap, v + uvSize);
		triapi->Vertex3f(x + charGap, y + charHeight, 0);

		triapi->TexCoord2f(u + ugap, v);
		triapi->Vertex3f(x + charGap, y, 0);

		x += charGap;
	}
	triapi->End();
	return i;
}

typedef	union CoordChar_u
{
	struct
	{
		int x;
		int y;
	};
	char firstchar;
}CoordChar;
char g_strbuf[2048];
CoordChar* g_pcurchar = (CoordChar*)g_strbuf;
void DrawSprString(int x, int y, const char* str)
{
	g_pcurchar->x = x;
	g_pcurchar->y = y;
	g_pcurchar++;

	for (; *str; str++, g_pcurchar = (CoordChar*)(((int)g_pcurchar) + 1))
		g_pcurchar->firstchar = *str;

	g_pcurchar->firstchar = '\0';
	g_pcurchar = (CoordChar*)(((int)g_pcurchar) + 1);
}
void DrawSprString_Flush()
{
	auto p = (CoordChar*)g_strbuf;
	while (p != g_pcurchar)
	{
		int x = p->x;
		int y = p->y;
		p++;
		int len = DrawSprString_Internal(x, y, &p->firstchar);
		p = (CoordChar*)(((char*)p) + len+1);
	}
	g_pcurchar = (CoordChar*)g_strbuf;
}
#pragma region PhysicsManager

PhysicsManager::PhysicsManager()
	:_visEntities()
{
	PAssetManager = nullptr;
	PRagdollManager = nullptr;
	PServerEntManager = nullptr;
	PCorpseManager = nullptr;

	m_collisionConfiguration = nullptr;
	m_dispatcher = nullptr;
	m_broadphase = nullptr;
	m_solver = nullptr;
	m_dynamicsWorld = nullptr;
}
PhysicsManager::~PhysicsManager()
{
	delete PAssetManager;
	delete PRagdollManager;
	delete PServerEntManager;
	delete PCorpseManager;

	delete m_dynamicsWorld->getDebugDrawer();
	delete m_dynamicsWorld;
	m_dynamicsWorld = 0;

	delete m_solver;
	m_solver = 0;

	delete m_broadphase;
	m_broadphase = 0;

	delete m_dispatcher;
	m_dispatcher = 0;

	delete m_collisionConfiguration;
	m_collisionConfiguration = 0;
}
#pragma region MakeExplosion

btVector3 Vector_ToScaledbtVector(Vector& v)
{
	return btVector3(v.x * G2BScale(), v.y * G2BScale(), v.z * G2BScale());
}
int explmdl = 0;
void Phys_Explode()
{
	float r = phys_explode_r->value * G2BScale();
	float intensity = phys_explode_i->value;
	btVector3 pos = Vector_ToScaledbtVector(gEngfuncs.GetLocalPlayer()->origin);
	auto world = DynamicWorld;
	for (int i = 0; i < world->getNumCollisionObjects(); i++)
	{
		auto obj = world->getCollisionObjectArray()[i];
		auto point = obj->getWorldTransform().getOrigin();
		auto dir = point - pos;
		auto distsqared = dir.length2();
		if (distsqared < r * r)
		{
			if (distsqared < r / 2 * r / 2)
			{
				distsqared = r / 2 * r / 2;
			}
			auto force = intensity / distsqared;
			dir.normalize();
			auto rigid = btRigidBody::upcast(obj);
			if (rigid != nullptr)
			{
				rigid->activate();
				rigid->applyCentralImpulse(dir * force);
			}
		}
	}
	gEngfuncs.pEfxAPI->R_Explosion(gEngfuncs.GetLocalPlayer()->origin, explmdl, 2.5, phys_explode_r->value, TE_EXPLFLAG_NONE);

}
#pragma endregion

#pragma region MakeRagdoll

btRigidBody* CreateBoneRigidbody(float mass, btTransform& offset, btCollisionShape* pshape);
void Phys_CreateBox()
{
	btBoxShape* pbox = new btBoxShape(btVector3(0.5, 0.5, 0.5));
	btTransform offset, start;
	offset.setIdentity();
	btRigidBody* pbody = CreateBoneRigidbody(1, offset, pbox);
	start.setIdentity();
	start.setOrigin(Vector_ToScaledbtVector(gEngfuncs.GetLocalPlayer()->origin));
	auto motion = pbody->getMotionState();
	motion->setWorldTransform(start);
	pbody->setWorldTransform(start);
	DynamicWorld->addRigidBody(pbody);
}
void Phys_CreateRagdoll()
{
	auto pent = gEngfuncs.GetLocalPlayer();
	auto modelname = IEngineStudio.PlayerInfo(pent->index - 1)->model;
	int i = 0;
	if (gPhysicsManager.PAssetManager->IsPlayerAssetAvailable(modelname, i))
	{
		auto info = (PhysicsComponentConstructionInfo*)gPhysicsManager.PAssetManager->GetPlayerPhysicsAsset(i);
		auto tempent = gPhysicsManager.PRagdollManager->CreateRagdoll(pent, &pent->curstate, info, DynamicWorld);
		tempent->entity.model = GetPlayerModel(modelname);
		double currentTime = 0;
		double oldTime = 0;
		int frame = 0;
		float boneTransform[128][3][4];
		if (pent->player)
		{
			IEngineStudio.GetTimes(&frame, &currentTime, &oldTime);
			PreModelRenderer.SetupBonesPlayer(pent, currentTime, currentTime - oldTime, boneTransform);
			auto component = gPhysicsManager.PRagdollManager->GetComponent(&tempent->entity);
			component->SetPose(boneTransform);
			component->Enable();
		}
	}
	//Phys_CreateBox();
}

#pragma endregion

#pragma region DebugDraw

int dbg_lineCount;
Vector(*dbg_lines)[3];
void dbg_flushlines()
{
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	if (!phys_dtest->value)
		glDisable(GL_DEPTH_TEST);
	gEngfuncs.pTriAPI->Brightness(1.0f);

	gEngfuncs.pTriAPI->Begin(TRI_LINES);
	for (size_t i = 0; i < dbg_lineCount; i++)
	{
		gEngfuncs.pTriAPI->Color4f(dbg_lines[i][2].x, dbg_lines[i][2].y, dbg_lines[i][2].z, 1);
		gEngfuncs.pTriAPI->Vertex3fv(dbg_lines[i][0]);

		gEngfuncs.pTriAPI->Color4f(dbg_lines[i][2].x, dbg_lines[i][2].y, dbg_lines[i][2].z, 1);
		gEngfuncs.pTriAPI->Vertex3fv(dbg_lines[i][1]);
	}
	gEngfuncs.pTriAPI->End();
	dbg_lineCount = 0;

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	if (!phys_dtest->value)
		glEnable(GL_DEPTH_TEST);
}
void dbg_drawline(const Vector& from, const Vector& to)
{
	VectorScale(from, B2GScale(), dbg_lines[dbg_lineCount][0]);
	VectorScale(to, B2GScale(), dbg_lines[dbg_lineCount][1]);
	dbg_lines[dbg_lineCount][2] = Vector(0.6f, 0.f, 1.f);// purple
	dbg_lineCount++;
}
class BufferedDebugDrawer :public btIDebugDraw
{
public:
	BufferedDebugDrawer();
	virtual ~BufferedDebugDrawer();

	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
	{
		VectorScale(from, B2GScale(), dbg_lines[dbg_lineCount][0]);
		VectorScale(to, B2GScale(), dbg_lines[dbg_lineCount][1]);
		VectorCopy(color, dbg_lines[dbg_lineCount][2]);
		dbg_lineCount++;
	}
	virtual void flushLines()
	{
		dbg_flushlines();
	}
	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) {};

	virtual void reportErrorWarning(const char* warningString) { gEngfuncs.Con_DPrintf("Bullet Error: %s", warningString); };

	virtual void draw3dText(const btVector3& location, const char* textString) { gEngfuncs.Con_DPrintf("3d text %s", textString); };

	virtual void setDebugMode(int debugMode) {};

	virtual int getDebugMode() const { return cvarcfg_DebugDrawMode(); };

private:

};

BufferedDebugDrawer::BufferedDebugDrawer()
{
	dbg_lines = new Vector[100000][3];
	dbg_lineCount = 0;
}

BufferedDebugDrawer::~BufferedDebugDrawer()
{
	delete[] dbg_lines;
}


#pragma endregion
void PhysicsManager::Init()
{
	phys_corpse = gEngfuncs.pfnRegisterVariable("phys_corpse", "1", FCVAR_CLIENTDLL);
	phys_corpsestay = gEngfuncs.pfnRegisterVariable("phys_corpsestay", "60", FCVAR_CLIENTDLL);
	char mode[32];
	sprintf_s<32>(mode, "%d", btIDebugDraw::DBG_DrawConstraintLimits | btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawWireframe);
	phys_debugdraw = gEngfuncs.pfnRegisterVariable("phys_debugdraw", /*"63567"*/mode, FCVAR_CLIENTDLL);
	phys_drawstatic = gEngfuncs.pfnRegisterVariable("phys_drawstatic", "0", FCVAR_CLIENTDLL);
	phys_simurate = gEngfuncs.pfnRegisterVariable("phys_simurate", "60", FCVAR_CLIENTDLL);
	phys_scale = gEngfuncs.pfnRegisterVariable("phys_scale", /*"0.03"*/"0.04", FCVAR_CLIENTDLL);
	phys_gravity = gEngfuncs.pfnRegisterVariable("phys_gravity", "9.8", FCVAR_CLIENTDLL);
	phys_dtest = gEngfuncs.pfnRegisterVariable("phys_dtest", "0", FCVAR_CLIENTDLL);
	phys_jiggle = gEngfuncs.pfnRegisterVariable("phys_jiggle", "1", FCVAR_CLIENTDLL);

	phys_explode_r = gEngfuncs.pfnRegisterVariable("phys_explode_r", "270", FCVAR_CLIENTDLL);
	phys_explode_i = gEngfuncs.pfnRegisterVariable("phys_explode_i", "1000", FCVAR_CLIENTDLL);

	gEngfuncs.pfnAddCommand("phys_explode", Phys_Explode);
	gEngfuncs.CL_LoadModel("sprites/explode1.spr", &explmdl);
	gEngfuncs.pfnAddCommand("phys_createragdoll", Phys_CreateRagdoll);

	PreModelRenderer.InitEx();

#pragma region InitDynamicWorld

	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btAxisSweep3(btVector3(-10000, -10000, -10000), btVector3(10000, 10000, 10000));/*new btDbvtBroadphase()*/;

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

	m_dynamicsWorld->setDebugDrawer(new BufferedDebugDrawer);
	DynamicWorld = m_dynamicsWorld;

	// enable ccd
	DynamicWorld->getSolverInfo().m_solverMode |= btSolverMode::SOLVER_USE_2_FRICTION_DIRECTIONS | btSolverMode::SOLVER_RANDMIZE_ORDER;
	DynamicWorld->getSolverInfo().m_numIterations = 60;
	DynamicWorld->getDispatchInfo().m_useContinuous = true;
#pragma endregion

}
void PhysicsManager::NewMap()
{
	cvarcfg_NewMap();

	if (PAssetManager)
		delete PAssetManager;
	if (PRagdollManager)
		delete PRagdollManager;
	if (PServerEntManager)
		delete PServerEntManager;
	if (PCorpseManager)
		delete PCorpseManager;

	PAssetManager = new PhysicsAssetManager();
	PRagdollManager = new RagdollManager();
	PServerEntManager = new KinematicManager(*PAssetManager);
	PCorpseManager = new CorpseManager(*PAssetManager, *PRagdollManager);

	m_dynamicsWorld->setGravity(btVector3(0, 0, -phys_gravity->value));
}

void PhysicsManager::AddEntity(cl_entity_t* pent, int entType)
{
	_visEntities.push_back({ pent,entType });
}

void PhysicsManager::MapReset()
{
	// for cs-like game, clear ragdolls last round
}

void PhysicsManager::Update(float currentTime, float deltaTime)
{
	g_pcurchar = (CoordChar*)g_strbuf;
	gCurrentTime = currentTime;
	gDeltaTime = deltaTime;
	char msg[64];
	sprintf_s<64>(msg, "FPS: %f\n", floor( 1 / deltaTime));
	DrawSprString(1100, 0, msg);
	if (currentTime == deltaTime)
		return;
	for (size_t i = 0; i < _visEntities.size(); i++)
	{
		auto currentent = _visEntities[i].pent;
		auto type = _visEntities[i].type;

		// ragdoll will replace it, do not apply any phys effects
		// PhysRenderer just simply return when encounter it.
		if (currentent->curstate.renderfx == kRenderFxDeadPlayer)
			break;

		switch (type)
		{
		case ET_NORMAL:
			// if not studio model, then try to add dead entity.
			if (currentent->model->type != modtype_t::mod_studio || !PCorpseManager->AddNormalEntity(currentent))
				PServerEntManager->AddNormal(currentent);
			break;
		case ET_PLAYER:
			if (!PCorpseManager->AddPlayerEntity(currentent))
			{
				PServerEntManager->AddPlayer(currentent);
				if (currentent->curstate.weaponmodel)
				{
					// TODO:
					bool playerPoseAvailable = false;
					if (playerPoseAvailable)
					{
						//save bone
						//setup weapon bone
					}
					else
					{
						//setup player bone
						//save bone
						// setup weapon bone
					}
				}
			}
		case ET_BEAM:
		case ET_TEMPENTITY:
		case ET_FRAGMENTED:
		default:
			break;
		}
	}
	_visEntities.clear();

	auto viewent = gEngfuncs.GetViewModel();
	if (viewent != NULL && viewent->curstate.modelindex != 0)
	{
		// TODO:
	}

	// before simulation
	PServerEntManager->BeforeSimulation(currentTime, deltaTime);

	DynamicWorld->stepSimulation(deltaTime);

	PServerEntManager->AfterSimulation();
}


void PhysicsManager::Draw()
{
	btCollisionObject* worldspawn = PServerEntManager->GetWorldSpawn();
	if (phys_drawstatic->value)
		worldspawn->setCollisionFlags(worldspawn->getCollisionFlags() & ~btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
	else
		worldspawn->setCollisionFlags(worldspawn->getCollisionFlags() | btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
	
#pragma region phys debug draw

	btDiscreteDynamicsWorld* pworld = (btDiscreteDynamicsWorld*)DynamicWorld;
	if (pworld->getDebugDrawer())
	{
		pworld->getDebugDrawer()->clearLines();

		btIDebugDraw::DefaultColors defaultColors = pworld->getDebugDrawer()->getDefaultColors();

		if (pworld->getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawContactPoints)
		{
			if (pworld->getDispatcher())
			{
				int numManifolds = pworld->getDispatcher()->getNumManifolds();

				for (int i = 0; i < numManifolds; i++)
				{
					btPersistentManifold* contactManifold = pworld->getDispatcher()->getManifoldByIndexInternal(i);
					//btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
					//btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

					int numContacts = contactManifold->getNumContacts();
					for (int j = 0; j < numContacts; j++)
					{
						btManifoldPoint& cp = contactManifold->getContactPoint(j);
						pworld->getDebugDrawer()->drawContactPoint(cp.m_positionWorldOnB, cp.m_normalWorldOnB, cp.getDistance(), cp.getLifeTime(), defaultColors.m_contactPoint);
					}
				}
			}
		}

		if ((pworld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb)))
		{
			int i;

			auto arr = pworld->getCollisionObjectArray();
			for (i = 0; i < pworld->getNumCollisionObjects(); i++)
			{
				btCollisionObject* colObj = arr[i];
				if ((colObj->getCollisionFlags() & btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT) == 0)
				{
					if (pworld->getDebugDrawer() && (pworld->getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawWireframe))
					{
						btVector3 color(btScalar(0.4), btScalar(0.4), btScalar(0.4));

						switch (colObj->getActivationState())
						{
						case ACTIVE_TAG:
							color = defaultColors.m_activeObject;
							break;
						case ISLAND_SLEEPING:
							color = defaultColors.m_deactivatedObject;
							break;
						case WANTS_DEACTIVATION:
							color = defaultColors.m_wantsDeactivationObject;
							break;
						case DISABLE_DEACTIVATION:
							color = defaultColors.m_disabledDeactivationObject;
							break;
						case DISABLE_SIMULATION:
							color = defaultColors.m_disabledSimulationObject;
							break;
						default:
						{
							color = btVector3(btScalar(.3), btScalar(0.3), btScalar(0.3));
						}
						};

						colObj->getCustomDebugColor(color);

						pworld->debugDrawObject(colObj->getWorldTransform(), colObj->getCollisionShape(), color);
					}
					if (pworld->getDebugDrawer() && (pworld->getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawAabb))
					{
						btVector3 minAabb, maxAabb;
						btVector3 colorvec = defaultColors.m_aabb;
						colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb, maxAabb);
						btVector3 contactThreshold(gContactBreakingThreshold, gContactBreakingThreshold, gContactBreakingThreshold);
						minAabb -= contactThreshold;
						maxAabb += contactThreshold;

						btVector3 minAabb2, maxAabb2;

						if (pworld->getDispatchInfo().m_useContinuous && colObj->getInternalType() == btCollisionObject::CO_RIGID_BODY && !colObj->isStaticOrKinematicObject())
						{
							colObj->getCollisionShape()->getAabb(colObj->getInterpolationWorldTransform(), minAabb2, maxAabb2);
							minAabb2 -= contactThreshold;
							maxAabb2 += contactThreshold;
							minAabb.setMin(minAabb2);
							maxAabb.setMax(maxAabb2);
						}

						pworld->getDebugDrawer()->drawAabb(minAabb, maxAabb, colorvec);
					}
				}
			}
		}
	}

	bool drawConstraints = false;
	if (pworld->getDebugDrawer())
	{
		int mode = pworld->getDebugDrawer()->getDebugMode();
		if (mode & (btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits))
		{
			drawConstraints = true;
		}
	}
	if (drawConstraints)
	{
		for (int i = pworld->getNumConstraints() - 1; i >= 0; i--)
		{
			btTypedConstraint* constraint = pworld->getConstraint(i);
			pworld->debugDrawConstraint(constraint);
		}
	}

	//if (pworld->getDebugDrawer() && (pworld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawNormals)))
	//{
	//	int i;

	//	if (pworld->getDebugDrawer() && pworld->getDebugDrawer()->getDebugMode())
	//	{
	//		for (i = 0; i < m_actions.size(); i++)
	//		{
	//			m_actions[i]->debugDraw(pworld->getDebugDrawer());
	//		}
	//	}
	//}
	if (pworld->getDebugDrawer())
		pworld->getDebugDrawer()->flushLines();

#pragma endregion

}
#pragma endregion

#pragma region KinematicManager


void PreSetupBonesModel(cl_entity_t* pent, float _currentTime, float _deltaTime, float outBonesTransform[MAXSTUDIOBONES][3][4])
{
	// let target entity bone transform available if MOVETYPE_FOLLOW.
	if (pent->curstate.movetype == MOVETYPE_FOLLOW)
	{
		auto targetent = gEngfuncs.GetEntityByIndex(pent->curstate.aiment);

		if (targetent->player)
		{
			PreModelRenderer.SetupBonesPlayer(targetent, _currentTime, _deltaTime, outBonesTransform);
		}
		else
		{
			PreModelRenderer.SetupBonesModel(targetent, _currentTime, _deltaTime, outBonesTransform);
		}
		PreModelRenderer.StudioSaveBones();
	}

	PreModelRenderer.SetupBonesModel(pent, _currentTime, _deltaTime, outBonesTransform);
}

KinematicManager::KinematicManager(PhysicsAssetManager& assetManager)
	:_assetManager(assetManager)
{
	_lookupTable.resize(1024);
	_playerComponents.resize(gEngfuncs.GetMaxClients());

	auto worldspawn = gEngfuncs.GetEntityByIndex(0);
	worldspawn->curstate.modelindex = 1;
	auto c = (EntityPhysicsComponent*)MakeNewComponent(worldspawn);
	c->Disable();
	c->SetComponentType(ComponentType::Static);
	c->Enable();
}

KinematicManager::~KinematicManager()
{
	// remove all normal entity from world
	for (auto& i : _old)
	{
		if (_lookupTable[i]->KinematicComponent != null)
			_lookupTable[i]->KinematicComponent->Disable();
	}
	for (auto& i : _added)
	{
		if (_lookupTable[i]->KinematicComponent != null)
			_lookupTable[i]->KinematicComponent->Disable();
	}
	// worldspawn is not registed
	_lookupTable[0]->KinematicComponent->Disable();
	// remove all player
	for (auto& i : _playerComponents)
		if (i.IsInWorld)
			i.Component->Disable();

	// free
	for (auto& i : _lookupTable)
	{
		if (i != null)
		{
			if (i->KinematicComponent != null)
				delete i->KinematicComponent;

			delete i;
		}
	}
	for (auto& i : _playerComponents)
		if (i.Component)
			delete i.Component;
}

void KinematicManager::AddNormal(cl_entity_t* pent)
{
	auto pEntity = pent;
	auto entityIndex = pEntity->index;
	auto modelType = pEntity->model->type;
	auto modelIndex = pEntity->curstate.modelindex;
	auto iter = std::find(_old.begin(), _old.end(), entityIndex);
	static float bonesTransform[MAXSTUDIOBONES][3][4];

	// exist entity last frame
	if (iter != _old.end())
	{
		_old.erase(iter);
		_added.push_back(entityIndex);
		KinematicTableItem* record = _lookupTable[entityIndex];
		// if model changed
		if (record->CorrespondingModel != modelIndex)
		{
			// disable last component if present
			if (record->KinematicComponent != null)
				record->KinematicComponent->Disable();
			MakeNewComponent(pEntity);
		}
		else
		{
			if (record->KinematicComponent != null)
			{
				// for those studio models, gives the newest pose
				if (modelType == modtype_t::mod_studio)
				{
					auto component = (SkeletalPhysicsComponent*)record->KinematicComponent;
					PreSetupBonesModel(pEntity, gCurrentTime, gDeltaTime, bonesTransform);
					component->SetPose(bonesTransform);
				}
			}
		}
	}
	else
	{
		_added.push_back(entityIndex);
		KinematicTableItem* record = _lookupTable[entityIndex];
		if (record != null && record->CorrespondingModel == modelIndex)
		{
			if (record->KinematicComponent != null)
			{
				// for those studio models, gives the newest pose
				if (modelType == modtype_t::mod_studio)
				{
					auto component = (SkeletalPhysicsComponent*)record->KinematicComponent;
					PreSetupBonesModel(pEntity, gCurrentTime, gDeltaTime, bonesTransform);
					component->SetPose(bonesTransform);
				}
				record->KinematicComponent->Enable();
			}
		}
		else
		{
			MakeNewComponent(pEntity);
		}
	}
}
// 要渲染的 studiomodel 实体都必须经过 kinematic manager
// 不然实体未经过这里更新状态渲染时会拿到旧的数据误认为已经setup bones过了

void KinematicManager::AddPlayer(cl_entity_t* pent)
{
	auto entityIndex = pent->index;
	auto playerIndex = pent->index - 1;
	auto playerInfo = IEngineStudio.PlayerInfo(playerIndex);
	static float boneTransform[MAXSTUDIOBONES][3][4];

#pragma region experiment

	PlayerComponentInfo& info = _playerComponents[playerIndex];
	info.Updated = true;

	// because it cost too much to check if a player model support physics
	// so for no phys, just component = NULL and IsInWorld  = false but register its modelname
	// next frame it will see its modelname but not in world, so it knows it not support phys
	if (info.IsInWorld)
	{
		// model still
		if (strcmp(info.ModelName, playerInfo->model) == 0)
		{
			// calc pose for next simulation
			auto component = (SkeletalPhysicsComponent*)info.Component;
			PreModelRenderer.SetupBonesPlayer(pent, gCurrentTime, gDeltaTime, boneTransform);
			component->SetPose(boneTransform);
		}
		else
		{
			// change component
			// if no phys, apply NULL component and remove from world (IsInWorld = false).
			// register name
			info.Component->Disable();
			delete info.Component;
			int assetIndex = -1;
			if (_assetManager.IsPlayerAssetAvailable(playerInfo->model, assetIndex))
			{
				auto asset = (PhysicsComponentConstructionInfo*)_assetManager.GetPlayerPhysicsAsset(assetIndex);
				PreModelRenderer.SetupBonesPlayer(pent, gCurrentTime, gDeltaTime, boneTransform);
				auto component = info.Component = new SkeletalPhysicsComponent(true, asset, DynamicWorld, boneTransform);
				component->SetOwner(pent);
			}
			else
			{
				info.Component = nullptr;
				info.IsInWorld = false;
			}
			strcpy(info.ModelName, playerInfo->model);
		}
	}
	else
	{
		if (strcmp(info.ModelName, playerInfo->model) == 0)
		{
			// re enable its component
			if (info.Component)
			{
				auto component = (SkeletalPhysicsComponent*)info.Component;
				PreModelRenderer.SetupBonesPlayer(pent, gCurrentTime, gDeltaTime, boneTransform);
				component->SetPose(boneTransform);
				component->Enable();
				info.IsInWorld = true;
			}
			else
			{
				// no support(no model name/ no phys file/ even nomodel)
			}
		}
		else
		{
			// new component(may null if not support)
			// copy name
			if (info.Component)// compo for last model
				info.Component->Disable();
			delete info.Component;

			int assetIndex = -1;
			if (_assetManager.IsPlayerAssetAvailable(playerInfo->model, assetIndex))
			{
				auto asset = (PhysicsComponentConstructionInfo*)_assetManager.GetPlayerPhysicsAsset(assetIndex);
				PreModelRenderer.SetupBonesPlayer(pent, gCurrentTime, gDeltaTime, boneTransform);
				auto component = info.Component = new SkeletalPhysicsComponent(true, asset, DynamicWorld, boneTransform);
				component->SetOwner(pent);
				info.IsInWorld = true;
			}
			else
			{
				info.Component = nullptr;
				info.IsInWorld = false;
			}
			strcpy(info.ModelName, playerInfo->model);
		}
	}
#pragma endregion

#pragma region old procedure

	//auto iter = std::find(_old.begin(), _old.end(), entityIndex);
	//// exist entity last frame
	//if (iter != _old.end())
	//{
	//	_old.erase(iter);
	//	_added.push_back(entityIndex);
	//	KinematicTableItem* record = _lookupTable[entityIndex];
	//	// if model changed
	//	if (strcmp(_playerModel[playerIndex], playerInfo->model))
	//	{
	//		// disable last component if present
	//		if (record->KinematicComponent != null)
	//			record->KinematicComponent->Disable();
	//		MakeNewPlayerComponent(pent);
	//		// enable new if present
	//		if (record->KinematicComponent != null)
	//		{
	//			// calc init pose
	//			auto component = (SkeletalPhysicsComponent*)record->KinematicComponent;
	//			PreModelRenderer.SetupBonesPlayer(pent, gCurrentTime, gDeltaTime, boneTransform);
	//			component->SetPose(boneTransform);

	//			record->KinematicComponent->Enable();
	//		}
	//	}
	//	else
	//	{
	//		if (record->KinematicComponent != null)
	//		{
	//			// calc pose for next simulation
	//			auto component = (SkeletalPhysicsComponent*)record->KinematicComponent;
	//			PreModelRenderer.SetupBonesPlayer(pent, gCurrentTime, gDeltaTime, boneTransform);
	//			component->SetPose(boneTransform);
	//		}
	//	}
	//}
	//else
	//{
	//	_added.push_back(entityIndex);
	//	KinematicTableItem* record = _lookupTable[entityIndex];
	//	if (record != null && strcmp(_playerModel[playerIndex], playerInfo->model) == 0)
	//	{
	//		if (record->KinematicComponent != null)
	//		{
	//			// calc init pose
	//			auto component = (SkeletalPhysicsComponent*)record->KinematicComponent;
	//			PreModelRenderer.SetupBonesPlayer(pent, gCurrentTime, gDeltaTime, boneTransform);
	//			component->SetPose(boneTransform);

	//			record->KinematicComponent->Enable();
	//		}
	//	}
	//	else
	//	{
	//		MakeNewPlayerComponent(pent);
	//		// maybe null before, we make a new record just now.
	//		record = _lookupTable[entityIndex];
	//		if (record->KinematicComponent != null)
	//		{
	//			// calc init pose
	//			auto component = (SkeletalPhysicsComponent*)record->KinematicComponent;
	//			PreModelRenderer.SetupBonesPlayer(pent, gCurrentTime, gDeltaTime, boneTransform);
	//			component->SetPose(boneTransform);

	//			record->KinematicComponent->Enable();
	//		}
	//	}
	//}
#pragma endregion
}

bool KinematicManager::IsPhysAnimating(cl_entity_t* pent)
{
	if (pent->player)
		return _playerComponents[pent->index - 1].IsInWorld;
	return (_lookupTable[pent->index] && _lookupTable[pent->index]->KinematicComponent != null);
}
SkeletalPhysicsComponent* KinematicManager::GetPhysComponent(cl_entity_t* pent)
{
	if (pent->player)
		return _playerComponents[pent->index - 1].Component;
	return (SkeletalPhysicsComponent*)_lookupTable[pent->index]->KinematicComponent;
}
btRigidBody* KinematicManager::GetWorldSpawn()
{
	return ((EntityPhysicsComponent*)_lookupTable[0]->KinematicComponent)->GetRigidbody();
}
IPhysicsComponent* KinematicManager::MakeNewComponent(cl_entity_t* pent)
{
	auto entityIndex = pent->index;
	auto entityModel = pent->curstate.modelindex;
	auto record = _lookupTable[entityIndex];

	if (record != NULL)
	{
		if (record->KinematicComponent)
			delete record->KinematicComponent;
	}
	else
	{
		record = _lookupTable[entityIndex] = new KinematicTableItem;
	}

	IPhysicsComponent* result = null;
	if (_assetManager.IsAssetAvailable(entityModel))
	{
		if (pent->model->type == modtype_t::mod_brush)
		{
			auto shape = (btCollisionShape*)_assetManager.GetPhysicsAsset(entityModel);
			auto component = new EntityPhysicsComponent(pent, shape, DynamicWorld);
			component->Enable();
			result = component;
		}
		else if (pent->model->type == modtype_t::mod_studio)
		{
			static float bonesTransform[MAXSTUDIOBONES][3][4];
			auto info = (PhysicsComponentConstructionInfo*)_assetManager.GetPhysicsAsset(entityModel);
			PreSetupBonesModel(pent, gCurrentTime, gDeltaTime, bonesTransform);
			auto component = new SkeletalPhysicsComponent(true, info, DynamicWorld, bonesTransform);
			component->SetOwner(pent);
			result = component;
		}
		else
		{
			gEngfuncs.Con_Printf("ERROR: Not supported physics asset [%s] on entity index[%d].\n", pent->model->name, pent->index);
		}
	}

	record->CorrespondingModel = entityModel;
	record->KinematicComponent = result;
	return result;
}
//IPhysicsComponent* KinematicManager::MakeNewPlayerComponent(cl_entity_t* pent)
//{
//	int playerIndex = pent->index - 1;
//	auto playerInfo = IEngineStudio.PlayerInfo(playerIndex);
//	auto entityIndex = pent->index;
//	auto record = _lookupTable[entityIndex];
//	IPhysicsComponent* result = null;
//	if (record != null)
//	{
//		if (record->KinematicComponent)
//			delete record->KinematicComponent;
//	}
//	else
//	{
//		record = _lookupTable[entityIndex] = new KinematicTableItem;
//	}
//
//	int playerModelIndex = -1;
//	if (_assetManager->IsPlayerAssetAvailable(playerInfo->model, playerModelIndex))
//	{
//		auto info = (PhysicsComponentConstructionInfo*)_assetManager->GetPlayerPhysicsAsset(playerModelIndex);
//		auto component = new SkeletalPhysicsComponent(true, info, DynamicWorld);
//		result = component;
//	}
//	strcpy_s(_playerModel[playerIndex], 64, playerInfo->model);
//	record->CorrespondingModel = -2;
//	record->KinematicComponent = result;
//
//	return result;
//}

void KinematicManager::BeforeSimulation(float currentTime, float deltaTime)
{
	// before simu, remove not updated component and reset updated field
	for (int i = 0; i < gEngfuncs.GetMaxClients(); i++)
	{
		auto& info = _playerComponents[i];
		if (!info.Updated && info.IsInWorld)
		{
			info.IsInWorld = false;
			info.Component->Disable();
		}
		info.Updated = false;
	}

	for (auto& i : _old)
	{
		auto c = _lookupTable[i]->KinematicComponent;
		if (c)
			c->Disable();
	}
	_old.clear();
}

void KinematicManager::AfterSimulation()
{
	//TODO:  FillAddonBonesRelativeTransform for viewent
	_added.swap(_old);
}

#pragma endregion

#pragma region CorpseManager

bool CorpseManager::AddPlayerEntity(cl_entity_t* pent)
{
	auto playerIndex = pent->index - 1;
	auto playerState = IEngineStudio.GetPlayerState(playerIndex);
	auto playerInfo = IEngineStudio.PlayerInfo(playerIndex);

	model_t* mod = GetPlayerModel(playerInfo->model);
	auto phdr = (studiohdr_t*)IEngineStudio.Mod_Extradata(mod);

	static float bonesTransform[MAXSTUDIOBONES][3][4];
	int assetIndex = -1;
	// a ragdoll replace the player that playing death sequence
	if (_assetManager.IsPlayerAssetAvailable(playerInfo->model, assetIndex) && 
		IsPlayingDeathSequence(phdr, playerState->sequence))
	{
		// first frame of death sequnce, create a ragdoll
		if (!IsEntityAlreadyDead(pent))
		{
			auto tempent = _ragdollManager.CreateRagdoll(pent, playerState,
				(PhysicsComponentConstructionInfo*)_assetManager.GetPlayerPhysicsAsset(assetIndex), DynamicWorld);
			tempent->entity.model = mod;
			auto component = _ragdollManager.GetComponent(&tempent->entity);
			PreModelRenderer.SetupBonesPlayer(pent, gCurrentTime, gDeltaTime, bonesTransform);
			component->SetPose(bonesTransform);
			component->Enable();
			EntityDie(pent);
		}
		return true;
	}
	// live player or no phys
	else
	{
		EntityRespawn(pent);
		return false;
	}
}
bool CorpseManager::AddNormalEntity(cl_entity_t* pent)
{
	static float bonesTransform[MAXSTUDIOBONES][3][4];

	// a ragdoll replace it, return true.
	auto phdr = (studiohdr_t*)IEngineStudio.Mod_Extradata(pent->model);
	if (IsPlayingDeathSequence(phdr, pent->curstate.sequence) && _assetManager.IsAssetAvailable(pent->curstate.modelindex))
	{
		if (!IsEntityAlreadyDead(pent))
		{
			auto info = (PhysicsComponentConstructionInfo*)_assetManager.GetPhysicsAsset(pent->curstate.modelindex);
			auto tempent = _ragdollManager.CreateRagdoll(pent, &pent->curstate, info, DynamicWorld);
			auto component = _ragdollManager.GetComponent(&tempent->entity);
			PreSetupBonesModel(pent, gCurrentTime, gDeltaTime, bonesTransform);
			component->SetPose(bonesTransform);
			component->Enable();
			// since we cannot get death info on single player mode.
			if (gEngfuncs.GetMaxClients() == 1)
			{
				auto local = gEngfuncs.GetLocalPlayer();
				Vector v = (pent->origin - local->origin).Normalize();												
				v = v * 6;
				component->SetVelocity(*(btVector3*)&v);
			}
			EntityDie(pent);
		}
		return true;
	}
	else
	{
		EntityRespawn(pent);
		return false;
	}
}


#pragma endregion

void RagdollUpdate(TEMPENTITY* tempent, float delta, float time)
{
	auto component = gPhysicsManager.PRagdollManager->GetComponent(&tempent->entity);
	component->GetWorldTransform(tempent->entity.origin, tempent->entity.angles);
	// do by engine
	//tempent->entity.curstate.origin = tempent->entity.origin;
	//tempent->entity.curstate.angles = tempent->entity.angles;
	if (tempent->entity.curstate.fuser1 < time)
	{
		// die next frame on TempEntUpdate
		tempent->die = time;
		component->Disable();
		gPhysicsManager.PRagdollManager->DestroyRagdoll(component);
		// don't draw this frame, we have no phys component to control its animation.
		tempent->flags = FTENT_NOMODEL;
	}
}