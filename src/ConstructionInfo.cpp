#include"ConstructionInfo.h"
#include"..\gsphysics\tinyxml2.h"
#include"cvarcfg.h"


void Matrix3x4_ToScaledbtTransform(const float matrix3x4[3][4], btTransform& trans);

using namespace tinyxml2;

void GetChildElements(XMLElement& xmlElement, const char* elemName, std::vector<XMLElement*>& outChildren)
{
	auto curchild = xmlElement.FirstChildElement(elemName);
	while (curchild)
	{
		outChildren.push_back(curchild);
		curchild = curchild->NextSiblingElement(elemName);
	}
}

#pragma region Get field from xml element

float GetFloatField(XMLElement& element, const char* name)
{
	auto field = element.Attribute(name);
	float result = 0;
	if (field)
	{
		result = strtof(field, NULL);
	}
	else
	{
		field = element.FirstChildElement(name)->GetText();
		result = strtof(field, NULL);
	}
	return result;
}

void GetScaledTransformField(XMLElement& element, const char* name, btTransform& outResult)
{
	float matrix[4][4];
	float* arr = (float*)matrix;

	char str[512];
	char* context = NULL;
	const char* separator = " ,\n\r\t,;";
	strcpy_s(str, 512, element.FirstChildElement(name)->GetText());
	char* token = strtok_s(str, separator, &context);
	for (size_t i = 0; i < 4 * 4; i++)
	{
		arr[i] = strtof(token, NULL);// NULL for no error checking
		token = strtok_s(NULL, separator, &context);
	}
	Matrix3x4_ToScaledbtTransform(matrix, outResult);
}

void GetScaledVector3Field(XMLElement& element, const char* name, btVector3& outResult)
{
	float arr[3];

	char str[512];
	strcpy_s(str, 512, element.FirstChildElement(name)->GetText());
	char* context = NULL;
	const char* separator = " ,\n\r\t,;";
	char* token = strtok_s(str, separator, &context);
	for (size_t i = 0; i < 3; i++)
	{
		arr[i] = strtof(token, NULL);// NULL for no error checking
		token = strtok_s(NULL, separator, &context);
	}
	outResult.setX(arr[0] * G2BScale());
	outResult.setY(arr[1] * G2BScale());
	outResult.setZ(arr[2] * G2BScale());
}

int GetInt32Field(XMLElement& element, const char* name)
{
	return atoi(element.FirstChildElement(name)->GetText());
}
#pragma endregion

#pragma region CreateShape

btCollisionShape* CreateSubShape(XMLElement& subShapeElement)
{
	btCollisionShape* result = nullptr;
	auto type = subShapeElement.Attribute("type");
	if (strcmp(type, "primitive.box") == 0)
	{
		btVector3 halfextent;
		GetScaledVector3Field(subShapeElement, "halfextent", halfextent);
		result = new btBoxShape(halfextent);
	}
	else if (strcmp(type, "primitive.capsule") == 0)
	{
		float r = GetFloatField(subShapeElement, "radius") * G2BScale();
		float h = GetFloatField(subShapeElement, "height") * G2BScale() - r - r;
		result = new btCapsuleShapeX(r, h);
	}
	else if (strcmp(type, "primitive.other") == 0)
	{

	}
	else
	{

	}
	return result;
}

#pragma endregion

#pragma region CreateConstraintInfo

ConstraintInfo CreateConstraintInfo(XMLElement& element, ConstraintInfo& result, std::vector<RigidbodyInfo>& RigidbodyInfos)
{
	auto type = element.Attribute("type");

	if (type == nullptr)
	{
	}
	else
	{
		btTransform locala, localb;
		GetScaledTransformField(element, "locala", locala);
		GetScaledTransformField(element, "localb", localb);
		int rba = atoi(element.Attribute("rba"));
		int rbb = atoi(element.Attribute("rbb"));
		int isAddon = (RigidbodyInfos[rba].UserType == UserRigidbodyType::Addon ||
			RigidbodyInfos[rbb].UserType == UserRigidbodyType::Addon) ? 1 : 0;

		
		result.UserType = (UserConstraintType)isAddon;
		result.RigidBodyA = rba;
		result.RigidBodyB = rbb;
		result.LocalInA = locala;
		result.LocalInB = localb;

		if (strcmp(type, "spherical") == 0)
		{
			//CreatePoint2Point(element,result);
			result.Type = btTypedConstraintType::POINT2POINT_CONSTRAINT_TYPE;
		}
		else if (strcmp(type, "hinge") == 0)
		{
			result.Type = btTypedConstraintType::HINGE_CONSTRAINT_TYPE;
			result.SwingSpan1 = GetFloatField(element, "low");
			result.SwingSpan2 = GetFloatField(element, "high");
		}
		else if (strcmp(type, "cone") == 0)
		{
			result.Type = btTypedConstraintType::CONETWIST_CONSTRAINT_TYPE;
			result.TwistSpan = GetFloatField(element, "twistspan");
			result.SwingSpan1 = GetFloatField(element, "swingspan1");
			result.SwingSpan2 = GetFloatField(element, "swingspan2");
		}
		else
		{

		}
	}
		
	return result;
}

#pragma endregion

#pragma region PhysicsComponentConstructionInfo

PhysicsComponentConstructionInfo::PhysicsComponentConstructionInfo()
{
	PStudioHeader = nullptr;
}
PhysicsComponentConstructionInfo:: ~PhysicsComponentConstructionInfo()
{
	for (auto& i : CollisionShapes)
	{
		delete i;
	}
}
void PhysicsComponentConstructionInfo::Parse(const char* const gpdStream, int streamLen, studiohdr_t* phdr)
{
	PStudioHeader = phdr;
	XMLDocument doc;
	doc.Parse(gpdStream, streamLen);
	auto gpd = doc.FirstChildElement("goldsrc-physics-data");

	strcpy(Checksum, gpd->Attribute("checksum"));

	// shape block
	auto shapeBlockElement = gpd->FirstChildElement("collision-shape-block");
	std::vector<XMLElement*> shapeElements;
	GetChildElements(*shapeBlockElement, "collision-shape", shapeElements);
	for (auto& i : shapeElements)
	{
		btCollisionShape* pshape = nullptr;
		std::vector<XMLElement*> subShapeElements;
		GetChildElements(*i, "sub-collision-shape", subShapeElements);
		if (subShapeElements.size() > 1)
		{
			auto pcompound = new btCompoundShape();
			btTransform childLocalTransform;
			for (auto& subelem : subShapeElements)
			{
				auto subshape = CreateSubShape(*subelem);
				GetScaledTransformField(*subelem, "local", childLocalTransform);
				pcompound->addChildShape(childLocalTransform, subshape);
			}
			pshape = pcompound;
		}
		else
		{
			pshape = CreateSubShape(*subShapeElements[0]);
		}
		CollisionShapes.push_back(pshape);
	}

	// rigidbody block
	auto block = gpd->FirstChildElement("rigidbody-block");
	std::vector<XMLElement*> elements;
	GetChildElements(*block, "rigidbody", elements);
	for (auto& i : elements)
	{
		int boneIndex = atoi(i->Attribute("bone"));
		int shapeIndex = atoi(i->Attribute("shape"));
		int userType = atoi(i->Attribute("type"));
		btTransform offset;
		GetScaledTransformField(*i, "local", offset);

		RigidbodyInfo rigidinfo =
		{
			shapeIndex,
			boneIndex,
			offset,
			(UserRigidbodyType)userType
		};

		RigidbodyInfos.push_back(rigidinfo);
	}

	elements.clear();
	// constraint block
	block = gpd->FirstChildElement("constraint-block");
	GetChildElements(*block, "constraint", elements);
	for (auto& i : elements)
	{
		ConstraintInfo cstinfo;
		CreateConstraintInfo(*i, cstinfo, RigidbodyInfos);
		ConstraintInfos.push_back(cstinfo);
	}

	std::vector<int>& boneIndeces = NonRigidbodyAnimatingBones;
	for (size_t i = 0; i < phdr->numbones; i++)
	{
		boneIndeces.push_back(i);
	}
	for (auto& i : RigidbodyInfos)
	{
		auto iter = std::find(boneIndeces.begin(), boneIndeces.end(), i.BoneIndex);
		if (iter != boneIndeces.end())
		{
			boneIndeces.erase(iter);
		}
	}
}

#pragma endregion