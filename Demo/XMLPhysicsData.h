#pragma once

#include <map>
#include <string>

namespace GameEngine
{
	namespace Physics
	{
		// Describes the bounciness (restitution) and friction of
		// a physics material. Can be combined with material densities
		// to get differing amounts of gravity.
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
			std::map<std::string, float> m_densities;
			std::map<std::string, MaterialData> m_materialTable;
			
			bool LoadDataFromXML(const std::string& fileName);
			MaterialData LookupMaterial(const std::string& materialName);
			float LookupDensity(const std::string& materialName);
		};
	}
}