#include "XMLPhysicsData.h"
#include <rapidxml.hpp>
#include <stdexcept>
#include <fstream>
#include <string>
#include <cassert>
#include <memory>
#include <functional>

using rapidxml::xml_document;
using rapidxml::xml_node;
using rapidxml::xml_attribute;

using std::string;
using std::stof;

using std::make_pair;

using std::ifstream;
using std::istreambuf_iterator;

typedef std::shared_ptr<xml_node<>> SharedNodePtr;

namespace GameEngine
{
	namespace PhysicsEngine
	{
		float ReadFloatAttribute(SharedNodePtr node, const string& attributeName)
		{
			std::auto_ptr<xml_attribute<>> attr;
			std::auto_ptr<const char> value;
			value.reset(attributeName.c_str());

			attr.reset(node->first_attribute(value.get()));
			assert(attr.get());
			value.reset(attr->value());
			return stof(value.get());
		}

		std::pair<string, MaterialData> ReadMaterialData(SharedNodePtr node)
		{
			float restitution;
			float friction;
			std::auto_ptr<xml_attribute<>> attr;
			
			restitution = ReadFloatAttribute(node, "restitution");
			friction = ReadFloatAttribute(node, "friction");

			std::auto_ptr<char> name;
			name.reset(node->name());

			return make_pair(string(name.get()), MaterialData(restitution, friction));
		}

		std::pair<string, float> ReadDensity(SharedNodePtr node)
		{
			float density;
			std::auto_ptr<xml_node<>> n;
			std::auto_ptr<char> value;

			n.reset(node->first_node());
			assert(n.get());
			value.reset(n->value());
			density = stof(value.get());

			value.reset(node->value());
			return make_pair(string(value.get()), density);
		}

		void MapChildNodes(std::shared_ptr<xml_node<>> inputNode,
			std::function<void(SharedNodePtr)> handleNode)
		{
			std::shared_ptr<xml_node<>> node;
			for (node.reset(inputNode->first_node());
				 node.get();
				 node.reset(node->next_sibling()))
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
			std::auto_ptr<const char> content;
			content.reset(matData.c_str());
			doc.parse<0>(const_cast<char*>(content.get()));
			std::auto_ptr<xml_node<>> root;
			root.reset(doc.first_node());

			SharedNodePtr materialNode;
			materialNode.reset(root->first_node("PhysicsMaterials"));
			assert(materialNode.get());

			MapChildNodes(materialNode, [this] (SharedNodePtr node)
			{
				this->m_materialTable.insert(ReadMaterialData(node));
			});

			materialNode.reset(root->first_node("DensityTable"));
			assert(materialNode.get());

			MapChildNodes(materialNode, [this] (SharedNodePtr node)
			{
				m_densities.insert(ReadDensity(node));
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
			if (it != m_densities.end())
			{
				return it->second;
			}
			throw std::runtime_error("No density found for material " + materialName + ".");
		}
	}
}