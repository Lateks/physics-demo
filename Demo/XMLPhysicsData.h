#ifndef XML_PHYSICS_DATA
#define XML_PHYSICS_DATA

#include <map>
#include <string>

namespace GameEngine
{
	namespace PhysicsEngine
	{
		struct MaterialData
		{
			MaterialData(float restitution, float friction)
				: m_restitution(restitution), m_friction(friction) { }
			MaterialData(MaterialData& other)
				: m_restitution(other.m_restitution), m_friction(other.m_friction) { }
			MaterialData(MaterialData&& other)
				: m_restitution(other.m_restitution), m_friction(other.m_friction) { }

			float m_restitution;
			float m_friction;
		};

		struct XMLPhysicsData
		{
			std::map<std::string, float> m_materialDensities;
			std::map<std::string, MaterialData> MaterialTable;
			
			void LoadDataFromXML(const std::string fileName);
		};
	}
}

#endif