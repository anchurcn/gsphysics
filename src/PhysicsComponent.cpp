#include<PhysicsComponent.h>
#include<cvarcfg.h>


void Matrix4x4_Transpose(float out[4][4], float in1[4][4])
{
	out[0][0] = in1[0][0];
	out[0][1] = in1[1][0];
	out[0][2] = in1[2][0];
	out[0][3] = in1[3][0];
	out[1][0] = in1[0][1];
	out[1][1] = in1[1][1];
	out[1][2] = in1[2][1];
	out[1][3] = in1[3][1];
	out[2][0] = in1[0][2];
	out[2][1] = in1[1][2];
	out[2][2] = in1[2][2];
	out[2][3] = in1[3][2];
	out[3][0] = in1[0][3];
	out[3][1] = in1[1][3];
	out[3][2] = in1[2][3];
	out[3][3] = in1[3][3];
}
void Matrix3x4_ToScaledbtTransform(const float matrix3x4[3][4], btTransform& trans)
{
	float matrix4x4[4][4] = {
		{1.0f, 0.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 0.0f, 1.0f}
	};
	memcpy(matrix4x4, matrix3x4, sizeof(float[3][4]));

	float matrix4x4_transposed[4][4];
	Matrix4x4_Transpose(matrix4x4_transposed, matrix4x4);

	trans.setFromOpenGLMatrix((float*)matrix4x4_transposed);
	trans.setOrigin(trans.getOrigin() * G2BScale());
}
void btTransform_ToScaledMatrix3x4(const btTransform& trans, float matrix3x4[3][4])
{
	float matrix4x4_transposed[4][4];
	
	trans.getOpenGLMatrix((float*)matrix4x4_transposed);

	float matrix4x4[4][4];
	Matrix4x4_Transpose(matrix4x4, matrix4x4_transposed);

	memcpy(matrix3x4, matrix4x4, sizeof(float[3][4]));
	matrix3x4[0][3] *= B2GScale();
	matrix3x4[1][3] *= B2GScale();
	matrix3x4[2][3] *= B2GScale();
}
void SinCos(float radians, float* sine, float* cosine)
{
	*sine = sinf(radians);
	*cosine = cosf(radians);
}
void Matrix4x4_CreateFromEntity(float out[4][4], const vec3_t angles, const vec3_t origin,const float scale)
{
	float	angle, sr, sp, sy, cr, cp, cy;


	if (angles[ROLL])
	{
		angle = angles[YAW] * (M_PI * 2 / 360.0f);
		SinCos(angle, &sy, &cy);
		angle = angles[PITCH] * (M_PI * 2 / 360.0f);
		SinCos(angle, &sp, &cp);
		angle = angles[ROLL] * (M_PI * 2 / 360.0f);
		SinCos(angle, &sr, &cr);

		out[0][0] = (cp * cy) * scale;
		out[0][1] = (sr * sp * cy + cr * -sy) * scale;
		out[0][2] = (cr * sp * cy + -sr * -sy) * scale;
		out[0][3] = origin[0];
		out[1][0] = (cp * sy) * scale;
		out[1][1] = (sr * sp * sy + cr * cy) * scale;
		out[1][2] = (cr * sp * sy + -sr * cy) * scale;
		out[1][3] = origin[1];
		out[2][0] = (-sp) * scale;
		out[2][1] = (sr * cp) * scale;
		out[2][2] = (cr * cp) * scale;
		out[2][3] = origin[2];
	}
	else if (angles[PITCH])
	{
		angle = angles[YAW] * (M_PI * 2 / 360.0f);
		SinCos(angle, &sy, &cy);
		angle = angles[PITCH] * (M_PI * 2 / 360.0f);
		SinCos(angle, &sp, &cp);

		out[0][0] = (cp * cy) * scale;
		out[0][1] = (-sy) * scale;
		out[0][2] = (sp * cy) * scale;
		out[0][3] = origin[0];
		out[1][0] = (cp * sy) * scale;
		out[1][1] = (cy)*scale;
		out[1][2] = (sp * sy) * scale;
		out[1][3] = origin[1];
		out[2][0] = (-sp) * scale;
		out[2][1] = 0.0f;
		out[2][2] = (cp)*scale;
		out[2][3] = origin[2];
	}
	else if (angles[YAW])
	{
		angle = angles[YAW] * (M_PI * 2 / 360.0f);
		SinCos(angle, &sy, &cy);

		out[0][0] = (cy)*scale;
		out[0][1] = (-sy) * scale;
		out[0][2] = 0.0f;
		out[0][3] = origin[0];
		out[1][0] = (sy)*scale;
		out[1][1] = (cy)*scale;
		out[1][2] = 0.0f;
		out[1][3] = origin[1];
		out[2][0] = 0.0f;
		out[2][1] = 0.0f;
		out[2][2] = scale;
		out[2][3] = origin[2];
	}
	else
	{
		out[0][0] = scale;
		out[0][1] = 0.0f;
		out[0][2] = 0.0f;
		out[0][3] = origin[0];
		out[1][0] = 0.0f;
		out[1][1] = scale;
		out[1][2] = 0.0f;
		out[1][3] = origin[1];
		out[2][0] = 0.0f;
		out[2][1] = 0.0f;
		out[2][2] = scale;
		out[2][3] = origin[2];
	}
}
void Matrix3x4_Invert(float const value[3][4],float result[3][4])
{    // M41,42,43 = 0, M44 = 1;
	{
		float num = value[2][0] * 0 - value[2][1] * 0;
		float num2 = value[2][0] * 0 - value[2][2] * 0;
		float num3 = value[2][3] * 0 - value[2][0] * 1;
		float num4 = value[2][1] * 0 - value[2][2] * 0;
		float num5 = value[2][3] * 0 - value[2][1] * 1;
		float num6 = value[2][2] * 1 - value[2][3] * 0;
		float num7 = value[1][1] * num6 + value[1][2] * num5 + value[1][3] * num4;
		float num8 = value[1][0] * num6 + value[1][2] * num3 + value[1][3] * num2;
		float num9 = value[1][0] * (0. - num5) + value[1][1] * num3 + value[1][3] * num;
		float num10 = value[1][0] * num4 + value[1][1] * (0 - num2) + value[1][2] * num;
		float num11 = value[0][0] * num7 - value[0][1] * num8 + value[0][2] * num9 - value[0][3] * num10;
		if (abs(num11) <= 1E-06f)
		{
			//TODO:result = Matrix3x4.Zero;
			return;
		}

		num11 = 1.f / num11;
		float num12 = value[0][0] * value[1][1] - value[0][1] * value[1][0];
		float num13 = value[0][0] * value[1][2] - value[0][2] * value[1][0];
		float num14 = value[0][3] * value[1][0] - value[0][0] * value[1][3];
		float num15 = value[0][1] * value[1][2] - value[0][2] * value[1][1];
		float num16 = value[0][3] * value[1][1] - value[0][1] * value[1][3];
		float num17 = value[0][2] * value[1][3] - value[0][3] * value[1][2];
		float num18 = value[0][1] * num6 + value[0][2] * num5 + value[0][3] * num4;
		float num19 = value[0][0] * num6 + value[0][2] * num3 + value[0][3] * num2;
		float num20 = value[0][0] * (0. - num5) + value[0][1] * num3 + value[0][3] * num;
		float num21 = value[0][0] * num4 + value[0][1] * (0.f - num2) + value[0][2] * num;
		float num22 = 0 * num17 + 0 * num16 + 1 * num15;
		float num23 = 0 * num17 + 0 * num14 + 1 * num13;
		float num24 = 0 * (0.f - num16) + 0 * num14 + 1 * num12;
		float num25 = 0 * num15 + 0 * (0.f - num13) + 0 * num12;
		float num26 = value[2][1] * num17 + value[2][2] * num16 + value[2][3] * num15;
		float num27 = value[2][0] * num17 + value[2][2] * num14 + value[2][3] * num13;
		float num28 = value[2][0] * (0.f - num16) + value[2][1] * num14 + value[2][3] * num12;
		float num29 = value[2][0] * num15 + value[2][1] * (0.f - num13) + value[2][2] * num12;
		result[0][0] = num7 * num11;
		result[0][1] = (0.f - num18) * num11;
		result[0][2] = num22 * num11;
		result[0][3] = (0.f - num26) * num11;
		result[1][0] = (0.f - num8) * num11;
		result[1][1] = num19 * num11;
		result[1][2] = (0.f - num23) * num11;
		result[1][3] = num27 * num11;
		result[2][0] = num9 * num11;
		result[2][1] = (0.f - num20) * num11;
		result[2][2] = num24 * num11;
		result[2][3] = (0.f - num28) * num11;
	}
}
void Matrix3x4_Multiply(float const lhs[3][4], float const rhs[3][4], float out[3][4])
{
	out[0][0] = lhs[0][0] * rhs[0][0] + lhs[0][1] * rhs[1][0] +
		lhs[0][2] * rhs[2][0];
	out[0][1] = lhs[0][0] * rhs[0][1] + lhs[0][1] * rhs[1][1] +
		lhs[0][2] * rhs[2][1];
	out[0][2] = lhs[0][0] * rhs[0][2] + lhs[0][1] * rhs[1][2] +
		lhs[0][2] * rhs[2][2];
	out[0][3] = lhs[0][0] * rhs[0][3] + lhs[0][1] * rhs[1][3] +
		lhs[0][2] * rhs[2][3] + lhs[0][3];
	out[1][0] = lhs[1][0] * rhs[0][0] + lhs[1][1] * rhs[1][0] +
		lhs[1][2] * rhs[2][0];
	out[1][1] = lhs[1][0] * rhs[0][1] + lhs[1][1] * rhs[1][1] +
		lhs[1][2] * rhs[2][1];
	out[1][2] = lhs[1][0] * rhs[0][2] + lhs[1][1] * rhs[1][2] +
		lhs[1][2] * rhs[2][2];
	out[1][3] = lhs[1][0] * rhs[0][3] + lhs[1][1] * rhs[1][3] +
		lhs[1][2] * rhs[2][3] + lhs[1][3];
	out[2][0] = lhs[2][0] * rhs[0][0] + lhs[2][1] * rhs[1][0] +
		lhs[2][2] * rhs[2][0];
	out[2][1] = lhs[2][0] * rhs[0][1] + lhs[2][1] * rhs[1][1] +
		lhs[2][2] * rhs[2][1];
	out[2][2] = lhs[2][0] * rhs[0][2] + lhs[2][1] * rhs[1][2] +
		lhs[2][2] * rhs[2][2];
	out[2][3] = lhs[2][0] * rhs[0][3] + lhs[2][1] * rhs[1][3] +
		lhs[2][2] * rhs[2][3] + lhs[2][3];
}
void Matrix3x4_Inverse(const float in[3][4], float result[3][4]) 
{
    float det = in[0][0] * in[1][1] * in[2][2] +
                in[1][0] * in[2][1] * in[0][2] +
                in[2][0] * in[0][1] * in[1][2] -
                in[2][0] * in[1][1] * in[0][2] -
                in[1][0] * in[0][1] * in[2][2] -
                in[0][0] * in[2][1] * in[1][2];

    float invDet = 1.0f / det;

    result[0][0] = (in[1][1] * in[2][2] - in[2][1] * in[1][2]) * invDet;
    result[0][1] = -(in[0][1] * in[2][2] - in[2][1] * in[0][2]) * invDet;
    result[0][2] = (in[0][1] * in[1][2] - in[1][1] * in[0][2]) * invDet;
    result[0][3] = -(in[0][3] * result[0][0] + in[1][3] * result[0][1] + in[2][3] * result[0][2]);
    result[1][0] = -(in[1][0] * in[2][2] - in[2][0] * in[1][2]) * invDet;
    result[1][1] = (in[0][0] * in[2][2] - in[2][0] * in[0][2]) * invDet;
    result[1][2] = -(in[0][0] * in[1][2] - in[1][0] * in[0][2]) * invDet;
    result[1][3] = -(in[0][3] * result[1][0] + in[1][3] * result[1][1] + in[2][3] * result[1][2]);
    result[2][0] = (in[1][0] * in[2][1] - in[2][0] * in[1][1]) * invDet;
    result[2][1] = -(in[0][0] * in[2][1] - in[2][0] * in[0][1]) * invDet;
    result[2][2] = (in[0][0] * in[1][1] - in[1][0] * in[0][1]) * invDet;
    result[2][3] = -(in[0][3] * result[2][0] + in[1][3] * result[2][1] + in[2][3] * result[2][2]);

}
float DegToRad(float deg)
{
	return deg / 180.f * 3.14159265357f;
}
btRigidBody* CreateEntityBody(float mass,cl_entity_t* pent, btCollisionShape* shape, btDynamicsWorld* world)
{
	btScalar m(mass);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (m != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(m, localInertia);
	
	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	EntityMotionState* myMotionState = new EntityMotionState(pent);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(m, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	if (!isDynamic)
	{
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		body->setActivationState(DISABLE_DEACTIVATION);
	}
	//add the body to the dynamics world
	world->addRigidBody(body);
	return body;
}
void DestryEntityBody(btRigidBody* body)
{
	delete body->getMotionState();
	delete body;
}


#pragma region EntityPhysicsComponent

EntityPhysicsComponent::EntityPhysicsComponent(cl_entity_t* pent, btCollisionShape* pshape, btDynamicsWorld* pworld)
	:EntityPhysicsComponent(0, pent, pshape, pworld)
{
	
}
EntityPhysicsComponent::EntityPhysicsComponent(float mass, cl_entity_t* pent, btCollisionShape* pshape, btDynamicsWorld* pworld)
{
	btScalar m(mass);
	bool isDynamic = (m != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		pshape->calculateLocalInertia(m, localInertia);

	auto motionState = new EntityMotionState(pent);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, pshape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	_pRigidbody = body;
	_pWorld = pworld;
	if(!isDynamic)
	{
		SetComponentType(ComponentType::Kinematic);
	}
}
EntityPhysicsComponent::~EntityPhysicsComponent()
{
	delete _pRigidbody->getMotionState();
	delete _pRigidbody;
}
void EntityPhysicsComponent::SetComponentType(ComponentType type)
{
	switch (type)
	{
	case ComponentType::Static:
	{
		int flag = _pRigidbody->getCollisionFlags();
		flag &= ~btCollisionObject::CF_KINEMATIC_OBJECT;
		flag |= btCollisionObject::CF_STATIC_OBJECT;
		_pRigidbody->setCollisionFlags(flag);
		_pRigidbody->setActivationState(ISLAND_SLEEPING);
		break;
	}
	case ComponentType::Kinematic:
	{
		int flag = _pRigidbody->getCollisionFlags();
		flag &= ~btCollisionObject::CF_STATIC_OBJECT;
		flag |= btCollisionObject::CF_KINEMATIC_OBJECT;
		_pRigidbody->setCollisionFlags(flag);
		_pRigidbody->setActivationState(DISABLE_DEACTIVATION);
		break;
	}
	case ComponentType::Dynamic:
	{
		int flag = _pRigidbody->getCollisionFlags();
		flag &= ~(btCollisionObject::CF_STATIC_OBJECT | btCollisionObject::CF_KINEMATIC_OBJECT);
		_pRigidbody->setCollisionFlags(flag);
		_pRigidbody->setActivationState(WANTS_DEACTIVATION);
		break;
	}
	default:
		break;
	}
}
ComponentType EntityPhysicsComponent::GetComponentType() const
{
	return ComponentType();
}
void EntityPhysicsComponent::Enable()
{
	_pWorld->addRigidBody(_pRigidbody);
}
void EntityPhysicsComponent::Disable()
{
	_pWorld->removeRigidBody(_pRigidbody);
}
#pragma endregion


#pragma region SkeletalPhysicsComponent

btRigidBody* CreateBoneRigidbody(float mass, btTransform& offset, btCollisionShape* pshape)
{
	if (mass == 0)
		return nullptr;

	btScalar m(mass);

	BoneMotionState* motionState = new BoneMotionState(offset);

	btVector3 localInertia;
	pshape->calculateLocalInertia(m, localInertia);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(m, motionState, pshape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	
	return body;
}
void DestryBoneRigidbody(btRigidBody* body)
{
	delete body->getMotionState();
	delete body;
}
void SkeletalPhysicsComponent::GetWorldTransform(float outMatrix[3][4])
{
	auto trans = _rigidbodies[0]->getWorldTransform();
	btTransform_ToScaledMatrix3x4(trans, outMatrix);
}
void ValueToMatrix3x4(float* value, float outMatrix[3][4])
{
	vec4_t q;
	AngleQuaternion(&value[3], q);
	QuaternionMatrix(q, outMatrix);
	outMatrix[0][3] = value[0];
	outMatrix[1][3] = value[1];
	outMatrix[2][3] = value[2];
}
void SetTPose(studiohdr_t* phdr, float outBoneTransform[MAXSTUDIOBONES][3][4])
{
	mstudiobone_t* pbones = (mstudiobone_t*)((byte*)phdr + phdr->boneindex);
	for (int i = 0; i < phdr->numbones; i++)
	{
		if (pbones[i].parent == -1)
		{
			ValueToMatrix3x4(pbones[i].value, outBoneTransform[i]);
		}
		else
		{
			float local[3][4];
			ValueToMatrix3x4(pbones[i].value, local);
			ConcatTransforms(outBoneTransform[pbones[i].parent], local, outBoneTransform[i]);
		}
	}
}
SkeletalPhysicsComponent::SkeletalPhysicsComponent(bool isKinematic, PhysicsComponentConstructionInfo* pConstructionInfo, btDynamicsWorld* pworld)
{
	// for construct with init pose
	float* initpose = NULL;
	static float boneTransform[MAXSTUDIOBONES][3][4];
	if (!initpose)
	{
		initpose = (float*)boneTransform;
		SetTPose(pConstructionInfo->PStudioHeader,boneTransform);
	}
	auto info = pConstructionInfo;

	for (size_t i = 0; i < info->RigidbodyInfos.size(); i++)
	{
		auto rigidinfo = info->RigidbodyInfos[i];
		auto rigidbody = CreateBoneRigidbody(1, rigidinfo.RigidOffset,info->CollisionShapes[rigidinfo.CollisionShape]);
		rigidbody->setUserIndex(rigidinfo.BoneIndex);
		rigidbody->setUserIndex2((int)rigidinfo.UserType);
		rigidbody->setUserPointer(this);
		_rigidbodies.push_back(rigidbody);
	}
	_phdr = pConstructionInfo->PStudioHeader;
	_pbones = (mstudiobone_t*)((byte*)_phdr + _phdr->boneindex);
	_enabled = false;
	SetPose(boneTransform);
	for (size_t i = 0; i < info->ConstraintInfos.size(); i++)
	{
		auto cstinfo = info->ConstraintInfos[i];
		auto pconstraint = (btTypedConstraint*)nullptr;
		switch (cstinfo.Type)
		{
		case btTypedConstraintType::POINT2POINT_CONSTRAINT_TYPE:
		{
			btPoint2PointConstraint* p2pcst = new btPoint2PointConstraint(
				*_rigidbodies[cstinfo.RigidBodyA],
				*_rigidbodies[cstinfo.RigidBodyB],
				cstinfo.LocalInA.getOrigin(),
				cstinfo.LocalInB.getOrigin()
			);
			p2pcst->setUserConstraintType((int)cstinfo.UserType);
			pconstraint = p2pcst;
			break;
		}
		case btTypedConstraintType::HINGE_CONSTRAINT_TYPE:
		{
			auto* hinge = new btHingeConstraint(
				*_rigidbodies[cstinfo.RigidBodyA],
				*_rigidbodies[cstinfo.RigidBodyB],
				cstinfo.LocalInA,
				cstinfo.LocalInB
			);
			hinge->setUserConstraintType((int)cstinfo.UserType);
			hinge->setLimit(DegToRad(cstinfo.SwingSpan1), DegToRad(cstinfo.SwingSpan2));
			pconstraint = hinge;
			break;
		}
		case btTypedConstraintType::CONETWIST_CONSTRAINT_TYPE:
		{
			btConeTwistConstraint* cone = new btConeTwistConstraint(
				*_rigidbodies[cstinfo.RigidBodyA],
				*_rigidbodies[cstinfo.RigidBodyB],
				cstinfo.LocalInA,
				cstinfo.LocalInB
			);
			cone->setUserConstraintType((int)cstinfo.UserType);
			
			cone->setLimit(DegToRad(cstinfo.SwingSpan1), DegToRad(cstinfo.SwingSpan2), DegToRad(cstinfo.TwistSpan));
			pconstraint = cone;
			break;
		}
		default:
			break;
		}
		pconstraint->setDbgDrawSize(0.2f);
		_constraints.push_back(pconstraint);
	}

	for (auto& i : _rigidbodies)
	{
		i->setDamping(0.05f, 0.85f);
		i->setDeactivationTime(0.8f);
		i->setSleepingThresholds(1.6f, 2.5f);
		i->setFriction(2.2f);
	}

	_isKinematic = !isKinematic;
	SetIsKinematic(isKinematic);

	_pConstructionInfo = info;
	_enabled = false;
	_phdr = info->PStudioHeader;
	_pbones = (mstudiobone_t*)((byte*)_phdr + _phdr->boneindex);
	pWorld = pworld;
}

SkeletalPhysicsComponent::~SkeletalPhysicsComponent()
{
	for (auto& i : _rigidbodies)
		DestryBoneRigidbody(i);

	for (auto& i : _constraints)
		delete i;
}

void SkeletalPhysicsComponent::Disable()
{
	if (!_enabled)
		return;

	_enabled = false;

	for (size_t i = 0; i < _rigidbodies.size(); i++)
	{
		getWorld()->removeRigidBody(_rigidbodies[i]);
	}
	for (size_t i = 0; i < _constraints.size(); i++)
	{
		auto c = _constraints[i];
		if (_isKinematic && c->getUserConstraintType() == (int)UserConstraintType::None)
			continue;
		getWorld()->removeConstraint(c);
	}
}

void SkeletalPhysicsComponent::Enable()
{
	if (_enabled)
		return;
	_enabled = true;

	// clear last state of rigidbody
	for (size_t i = 0; i < _rigidbodies.size(); i++)
	{
		auto rigidbody = _rigidbodies[i];
		btVector3 zero(0, 0, 0);
		rigidbody->setLinearVelocity(zero);
		rigidbody->setAngularVelocity(zero);
		rigidbody->clearForces();
		rigidbody->activate();
	}

	for (size_t i = 0; i < _rigidbodies.size(); i++)
	{
		btRigidBody* body = _rigidbodies[i];
		BoneMotionState* state = (BoneMotionState*)body->getMotionState();
		btTransform trans;
		state->getWorldTransform(trans);
		body->setWorldTransform(trans);
		body->setInterpolationWorldTransform(trans);
		
		getWorld()->addRigidBody(_rigidbodies[i]);
		if (!phys_corpse->value && !_isKinematic)
			body->setActivationState(DISABLE_SIMULATION);
	}
	for (size_t i = 0; i < _constraints.size(); i++)
	{
		auto c = _constraints[i];
		if (_isKinematic && c->getUserConstraintType() == (int)UserConstraintType::None)
			continue;
		getWorld()->addConstraint(c,true);
	}
}

void SkeletalPhysicsComponent::SetVelocity(btVector3& v)
{
	for (size_t i = 0; i < _rigidbodies.size(); i++)
	{
		_rigidbodies[i]->setLinearVelocity(v);
	}
}

void SkeletalPhysicsComponent::SetPose(float pInBonesTransform[MAXSTUDIOBONES][3][4])
{
	for (size_t i = 0; i < _rigidbodies.size(); i++)
	{
		if (!_enabled ||( _isKinematic && _rigidbodies[i]->getUserIndex2() != (int)UserRigidbodyType::Addon))
		{
			auto rigidbody = _rigidbodies[i];
			auto motionState = (BoneMotionState*)rigidbody->getMotionState();
			motionState->SetBoneTransform(pInBonesTransform[rigidbody->getUserIndex()]);

			btTransform trans;
			motionState->getWorldTransform(trans);
			rigidbody->setWorldTransform(trans);
//			rigidbody->setInterpolationWorldTransform(trans);
		}
	}
	for (size_t i = 0; i < _phdr->numbones; i++)
	{
		if (_pbones[i].parent == -1)
		{
			MatrixCopy(pInBonesTransform[i], _boneLocalTransforms[i]);
		}
		else
		{
			float worldToLocal[3][4];
			Matrix3x4_Inverse(pInBonesTransform[_pbones[i].parent], worldToLocal);
			Matrix3x4_Multiply(worldToLocal, pInBonesTransform[i], _boneLocalTransforms[i]);
		}
	}
}

void SkeletalPhysicsComponent::SetupBones(float pOutBoneTransform[MAXSTUDIOBONES][3][4])
{
	for (size_t i = 0; i < _rigidbodies.size(); i++)
	{
		auto rigidbody = _rigidbodies[i];
		auto boneTrans = ((BoneMotionState*)rigidbody->getMotionState())->bonematrix;
		auto boneIndex = rigidbody->getUserIndex();
		btTransform_ToScaledMatrix3x4(boneTrans,pOutBoneTransform[boneIndex]);
	}
	for (size_t i = 0; i < _pConstructionInfo->NonRigidbodyAnimatingBones.size(); i++)
	{
		auto boneIndex = _pConstructionInfo->NonRigidbodyAnimatingBones[i];
		if (_pbones[boneIndex].parent == -1)
		{
			MatrixCopy(_boneLocalTransforms[boneIndex], pOutBoneTransform[boneIndex]);
		}
		else
		{
			int parent = _pbones[boneIndex].parent;
			Matrix3x4_Multiply(pOutBoneTransform[parent], _boneLocalTransforms[boneIndex], pOutBoneTransform[boneIndex]);
		}
	}
}
void SkeletalPhysicsComponent::SetIsKinematic(bool value)
{
	if (_isKinematic == value)
		return;

	if (value == true)
	{
		SetKinematic();
		if (_enabled)
		{
			// remove some constraints
			for (size_t i = 0; i < _constraints.size(); i++)
			{
				auto c = _constraints[i];
				if (c->getUserConstraintType() == (int)UserConstraintType::None)
					getWorld()->removeConstraint(c);
			}
		}
	}
	else
	{
		SetDynamic();
		if (_enabled)
		{
			// add some constraints
			for (size_t i = 0; i < _constraints.size(); i++)
			{
				auto c = _constraints[i];
				if (c->getUserConstraintType() == (int)UserConstraintType::None)
					getWorld()->addConstraint(c);
			}
		}
	}
	_isKinematic = value;
}
/// <summary>
/// turn some rigidbodies to kinematic
/// </summary>
void SkeletalPhysicsComponent::SetKinematic()
{
	for (size_t i = 0; i < _rigidbodies.size(); i++)
	{
		auto rigidbody = _rigidbodies[i];
		if (rigidbody->getUserIndex2() != (int)UserRigidbodyType::Addon)
		{
			rigidbody->setCollisionFlags(rigidbody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			rigidbody->setActivationState(DISABLE_DEACTIVATION);
		}
	}
}
/// <summary>
/// turn to dynamic rigidbodies
/// </summary>
void SkeletalPhysicsComponent::SetDynamic()
{
	for (size_t i = 0; i < _rigidbodies.size(); i++)
	{
		auto rigidbody = _rigidbodies[i];
		if (rigidbody->getUserIndex2() != (int)UserRigidbodyType::Addon)
		{
			rigidbody->setCollisionFlags(rigidbody->getCollisionFlags() & (~btCollisionObject::CF_KINEMATIC_OBJECT));
			rigidbody->setActivationState(WANTS_DEACTIVATION);
		}
	}
}

#pragma endregion

#pragma region MotionState

// offset is rigid's local to the bone
void BoneMotionState::getWorldTransform(btTransform& worldTrans) const
{
	worldTrans.mult(bonematrix, offsetmatrix);
}

void BoneMotionState::setWorldTransform(const btTransform& worldTrans)
{
	bonematrix.mult(worldTrans, offsetmatrix.inverse());
}

void BoneMotionState::GetBoneTransform(float out[3][4])
{
	btTransform_ToScaledMatrix3x4(bonematrix, out);
}
void BoneMotionState::SetBoneTransform(float const in[3][4])
{
	Matrix3x4_ToScaledbtTransform(in, bonematrix);
}

void EntityMotionState::getWorldTransform(btTransform& worldTrans) const
{
	float matrix[4][4];
	Matrix4x4_CreateFromEntity(matrix, _pEntity->angles, _pEntity->origin, 1);
	Matrix3x4_ToScaledbtTransform(matrix, worldTrans);
}

void EntityMotionState::setWorldTransform(const btTransform& worldTrans)
{
	
}
#pragma endregion
