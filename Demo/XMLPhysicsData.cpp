#include "XMLPhysicsData.h"
#include <rapidxml.hpp>
#include <stdexcept>
#include <fstream>
#include <string>
#include <cassert>
#include <functional>
#include <iostream>

using rapidxml::xml_document;
using rapidxml::xml_node;
using rapidxml::xml_attribute;

using std::string;
using std::stof;

using std::make_pair;

using std::ifstream;
using std::istreambuf_iterator;

namespace GameEngine
{
	namespace Physics
	{
		float ReadFloatAttribute(xml_node<> *node, const string& attributeName)
		{
			xml_attribute<> * attr = node->first_attribute(attributeName.c_str());
			assert(attr);
			return stof(attr->value());
		}

		std::pair<string, MaterialData> ReadMaterialData(xml_node<> *node)
		{
			float restitution;
			float friction;
			
			restitution = ReadFloatAttribute(node, "restitution");
			friction = ReadFloatAttribute(node, "friction");

			return make_pair(string(node->name()), MaterialData(restitution, friction));
		}

		std::pair<string, float> ReadDensity(xml_node<> *node)
		{
			float density;
			xml_node<> *n;

			n = node->first_node();
			assert(n);
			density = stof(n->value());

			return make_pair(string(node->name()), density);
		}

		void MapChildNodes(xml_node<> *inputNode,
			std::function<void(xml_node<>*)> handleNode)
		{
			xml_node<> *node;
			for (node = inputNode->first_node(); node; node = node->next_sibling())
			{
				handleNode(node);
			}
		}

		void XMLPhysicsData::LoadDataFromXML(const std::string& fileName)
		{
			ifstream matFile(fileName);
			string matData(
				(istreambuf_iterator<char>(matFile)),
				(istreambuf_iterator<char>()));
			xml_document<> doc;
			doc.parse<0>(const_cast<char*>(matData.c_str()));
			xml_node<> *root = doc.first_node();

			xml_node<> *materialNode = root->first_node("PhysicsMaterials");
			assert(materialNode);

			MapChildNodes(materialNode, [this] (xml_node<> *node)
			{
				this->m_materialTable.insert(ReadMaterialData(node));
			});

			materialNode = root->first_node("DensityTable");
			assert(materialNode);

			MapChildNodes(materialNode, [this] (xml_node<> *node)
			{
				this->m_densities.insert(ReadDensity(node));
			});
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
			assert(it != m_densities.end());
			if (it != m_densities.end())
			{
				return it->second;
			}
			return 0;
		}
	}
}