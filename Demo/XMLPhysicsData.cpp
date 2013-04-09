#include "XMLPhysicsData.h"
#include <stdexcept>

namespace GameEngine
{
	namespace PhysicsEngine
	{
		void XMLPhysicsData::LoadDataFromXML(const std::string& fileName)
		{
			// TODO
		}

		MaterialData XMLPhysicsData::LookupMaterial(const std::string& materialName)
		{
			auto it = m_materialTable.find(materialName);
			if (it != m_materialTable.end())
			{
				return it->second;
			}
			return MaterialData(0, 0);
		}

		float XMLPhysicsData::LookupDensity(const std::string& materialName)
		{
			auto it = m_densities.find(materialName);
			if (it != m_densities.end())
			{
				return it->second;
			}
			throw std::runtime_error("No density found for material " + materialName + ".");
		}
	}
}