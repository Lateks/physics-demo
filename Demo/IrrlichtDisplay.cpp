#include "IrrlichtDisplay.h"
#include "IrrlichtConversions.h"
#include "IInputState.h"
#include "MessagingWindow.h"

#include "GameActor.h"
#include "WorldTransformComponent.h"

#include "GameData.h"

#include "EventManager.h"
#include "Events.h"

#include "Vec3.h"
#include "Vec4.h"

#include <irrlicht.h>

#include <iostream>
#include <cassert>
#include <memory>

using irr::u32;

using irr::video::SColor;
using irr::video::E_DRIVER_TYPE;
using irr::video::ITexture;

using irr::core::vector3df;
using irr::core::quaternion;

using irr::scene::ISceneManager;
using irr::scene::ISceneNode;
using irr::scene::IAnimatedMesh;

using std::shared_ptr;

using GameEngine::Events::IEventData;
using GameEngine::Events::EventType;
using GameEngine::Events::EventPtr;
using GameEngine::Events::ActorMoveEvent;

namespace
{
	unsigned int TEXTURE_ID = 0;
}

namespace GameEngine
{
	namespace Display
	{
		class IrrlichtInputState : public IInputState, public irr::IEventReceiver
		{
		public:
			virtual ~IrrlichtInputState() { }

			virtual bool OnEvent(const irr::SEvent& event)
			{
				if (event.EventType == irr::EET_MOUSE_INPUT_EVENT)
				{
					m_mouseState = IInputState::MouseState(event.MouseInput.isLeftPressed(),
						event.MouseInput.isRightPressed(), event.MouseInput.X, event.MouseInput.Y);
				}
				return false;
			}
		};

		struct IrrlichtDisplayData
		{
			IrrlichtDisplayData() : m_pDevice(nullptr), m_pDriver(nullptr),
				m_pSmgr(nullptr), m_pCamera(nullptr), m_pGui(nullptr) { }

			void AddSceneNode(ActorPtr pActor, irr::scene::ISceneNode *pNode, unsigned int texture, bool lightingOn);
			void UpdateActorPosition(Events::EventPtr pEvent);
			void SetNodeTransform(irr::scene::ISceneNode *pNode, const WorldTransformComponent& worldTransform);
			void SetCursorVisible(bool value);
			unsigned int GetTime();
			irr::scene::ISceneNode *GetSceneNode(ActorID actorId);

			std::map<unsigned int, irr::video::ITexture*> textures;
			std::map<ActorID, irr::scene::ISceneNode*> sceneNodes;

			irr::IrrlichtDevice *m_pDevice;
			irr::video::IVideoDriver *m_pDriver;
			irr::scene::ISceneManager *m_pSmgr;

			irr::scene::ICameraSceneNode *m_pCamera;
			irr::gui::IGUIEnvironment *m_pGui;

			Events::EventHandlerPtr m_pMoveEventHandler;
			shared_ptr<IrrlichtInputState> m_pInputState;
			shared_ptr<MessagingWindow> m_pMessageWindow;
		};

		class IrrlichtMessagingWindow : public MessagingWindow
		{
		public:
			IrrlichtMessagingWindow(std::size_t messageBufferSize, irr::gui::IGUIEnvironment *irrlichtGUI);
			virtual void VAddMessage(const std::wstring& message) override;
			virtual void VSetPosition(unsigned int minX, unsigned int minY) override;
			virtual void VSetWidth(unsigned int width) override;
			virtual void VSetFont(const std::string& fontFileName) override;
			virtual void VSetVisible(bool visible) override;
			virtual void VRender() const override;
		private:
			irr::gui::IGUIFont *m_pFont;
			irr::gui::IGUIEnvironment *m_irrlichtGUI;
			bool m_visible;
			unsigned int m_posX;
			unsigned int m_posY;
			unsigned int m_width;
			unsigned int m_fontHeight;
			unsigned int m_maxMessages;
			std::vector<irr::core::stringw> m_messageBuffer;
			irr::video::SColor m_color;
			void SetFontHeight();
		};

		/*
		 * Implementation of the IrrlichtDisplayFactory class.
		 */

		IrrlichtDisplayFactory::IrrlichtDisplayFactory(unsigned int width, unsigned int height,
			DriverType driverType, CameraType cameraType, int messageBufferSize)
			: m_width(width), m_height(height), m_driverType(driverType), m_cameraType(cameraType),
			m_messageBufferSize(messageBufferSize) { }

		std::shared_ptr<IDisplay> IrrlichtDisplayFactory::VCreateDeviceAndOpenWindow() const
		{
			auto pDisplay = std::shared_ptr<IDisplay>(new IrrlichtDisplay());
			if (pDisplay)
			{
				if (!pDisplay->VSetupAndOpenWindow(m_width, m_height, m_driverType, m_cameraType, m_messageBufferSize))
				{
					pDisplay.reset();
				}
			}
			return pDisplay;
		}

		/*
		 * Implementation of the IrrlichtDisplay class.
		 */
		IrrlichtDisplay::IrrlichtDisplay()
			: m_pData(new IrrlichtDisplayData())
		{
			m_pData->m_pInputState.reset(new IrrlichtInputState());
			m_pData->m_pMoveEventHandler =
				Events::EventHandlerPtr(new std::function<void(EventPtr)>(
				[this] (EventPtr event) { this->m_pData->UpdateActorPosition(event); }));
		}

		Vec3 IrrlichtDisplay::VGetCameraPosition() const
		{
			vector3df pos = m_pData->m_pCamera->getAbsolutePosition();
			return ConvertVector(pos);
		}

		shared_ptr<IInputState> IrrlichtDisplay::VGetInputState() const
		{
			return m_pData->m_pInputState;
		}

		unsigned int IrrlichtDisplay::VGetDeviceTimeMs() const
		{
			return m_pData->GetTime();
		}

		void IrrlichtDisplay::VDrawScene()
		{
			assert(m_pData->m_pDriver);

			m_pData->m_pDriver->beginScene(true, true, SColor(0,0,0,0));

			m_pData->m_pSmgr->drawAll();

			m_pData->m_pGui->drawAll();
			m_pData->m_pMessageWindow->VRender();

			m_pData->m_pDriver->endScene();
		}

		inline E_DRIVER_TYPE GetDriverType(DriverType type)
		{
			switch (type)
			{
			case DriverType::DIRECT_3D9:
				return E_DRIVER_TYPE::EDT_DIRECT3D9;
			case DriverType::OPEN_GL:
				return E_DRIVER_TYPE::EDT_OPENGL;
			case DriverType::SOFTWARE:
				return E_DRIVER_TYPE::EDT_SOFTWARE;
			case DriverType::NO_WINDOW:
				return E_DRIVER_TYPE::EDT_NULL;
			default:
				return E_DRIVER_TYPE::EDT_SOFTWARE;
			}
		}

		std::unique_ptr<irr::SKeyMap[]> CreateWASDKeyMap()
		{
			std::unique_ptr<irr::SKeyMap[]> wasdKeyMap(new irr::SKeyMap[4]);
			wasdKeyMap[0].Action = irr::EKA_MOVE_FORWARD;
			wasdKeyMap[0].KeyCode = irr::KEY_KEY_W;
			wasdKeyMap[1].Action = irr::EKA_MOVE_BACKWARD;
			wasdKeyMap[1].KeyCode = irr::KEY_KEY_S;
			wasdKeyMap[2].Action = irr::EKA_STRAFE_LEFT;
			wasdKeyMap[2].KeyCode = irr::KEY_KEY_A;
			wasdKeyMap[3].Action = irr::EKA_STRAFE_RIGHT;
			wasdKeyMap[3].KeyCode = irr::KEY_KEY_D;
			return wasdKeyMap;
		}

		bool IrrlichtDisplay::VSetupAndOpenWindow(unsigned int width, unsigned int height,
			DriverType driverType, CameraType cameraType, int messageBufferSize)
		{
			if (m_pData->m_pDevice)
			{
				std::cerr << "IrrlichtDisplay: Window already exists." << std::endl;
				return false;
			}

			m_pData->m_pDevice = irr::createDevice(GetDriverType(driverType),
				irr::core::dimension2d<u32>(width, height),
				16, false, false, false, m_pData->m_pInputState.get());
			if (!m_pData->m_pDevice)
			{
				return false;
			}

			m_pData->m_pDriver = m_pData->m_pDevice->getVideoDriver();
			m_pData->m_pSmgr = m_pData->m_pDevice->getSceneManager();
			m_pData->m_pGui = m_pData->m_pDevice->getGUIEnvironment();

			if (messageBufferSize > 0)
			{
				m_pData->m_pMessageWindow.reset(new IrrlichtMessagingWindow(messageBufferSize, m_pData->m_pGui));
				m_pData->m_pMessageWindow->VSetPosition(10, 10);
			}

			switch (cameraType)
			{
			case CameraType::STATIC:
				m_pData->m_pCamera = m_pData->m_pSmgr->addCameraSceneNode();
				break;
			case CameraType::FPS_WASD:
				m_pData->m_pCamera = m_pData->m_pSmgr->addCameraSceneNodeFPS(
					0, 100.f, 0.5f, -1, CreateWASDKeyMap().get(), 4);
				break;
			default: // defaults to the Irrlicht default FPS camera
				m_pData->m_pCamera = m_pData->m_pSmgr->addCameraSceneNodeFPS();
			}

			return true;
		}

		shared_ptr<MessagingWindow> IrrlichtDisplay::VGetMessageWindow()
		{
			return m_pData->m_pMessageWindow;
		}

		IrrlichtDisplay::~IrrlichtDisplay()
		{
			if (m_pData->m_pDevice)
			{
				m_pData->m_pDevice->drop();
			}
		}

		void IrrlichtDisplay::VSetCameraPosition(Vec3& newPosition)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setPosition(ConvertVector(newPosition));
			m_pData->m_pCamera->updateAbsolutePosition();
		}

		void IrrlichtDisplay::VSetCameraTarget(Vec3& newTarget)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setTarget(ConvertVector(newTarget));
		}

		void IrrlichtDisplay::VSetCameraUpVector(Vec3& newUpVector)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setUpVector(ConvertVector(newUpVector));
		}

		void IrrlichtDisplay::VSetCameraRotation(Quaternion newRotation)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setRotation(QuaternionToEuler(newRotation) * irr::core::RADTODEG);
		}

		void IrrlichtDisplay::VSetCameraFOV(float degrees)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setFOV(irr::core::degToRad(degrees));
		}

		void IrrlichtDisplay::VSetCameraNearPlaneDistance(float distance)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setNearValue(distance);
		}

		void IrrlichtDisplay::VSetCameraFarPlaneDistance(float distance)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setFarValue(distance);
		}

		void IrrlichtDisplay::VHideCursor()
		{
			m_pData->SetCursorVisible(false);
		}

		void IrrlichtDisplay::VShowCursor()
		{
			m_pData->SetCursorVisible(true);
		}

		bool IrrlichtDisplay::VCursorVisible() const
		{
			assert(m_pData->m_pCamera);
			return m_pData->m_pDevice->getCursorControl()->isVisible();
		}

		void IrrlichtDisplayData::SetCursorVisible(bool value)
		{
			assert(m_pDevice);
			if (m_pDevice)
			{
				m_pDevice->getCursorControl()->setVisible(value);
			}
		}

		bool IrrlichtDisplay::VRunning()
		{
			assert(m_pData->m_pDevice);
			if (!m_pData->m_pDevice)
				return false;
			return m_pData->m_pDevice->run();
		}

		bool IrrlichtDisplay::VWindowActive()
		{
			assert(m_pData->m_pDevice);
			if (!m_pData->m_pDevice)
				return false;
			return m_pData->m_pDevice->isWindowActive();
		}

		void IrrlichtDisplay::VYieldDevice()
		{
			assert(m_pData->m_pDevice);
			m_pData->m_pDevice->yield();
		}

		unsigned int IrrlichtDisplay::VLoadTexture(const std::string& filePath)
		{
			assert(m_pData->m_pDriver);
			auto texture = m_pData->m_pDriver->getTexture(filePath.c_str());
			if (!texture)
				return 0;
			m_pData->textures[++TEXTURE_ID] = texture;
			return TEXTURE_ID;
		}

		void IrrlichtDisplay::VAddSphereSceneNode(float radius, ActorPtr pActor, unsigned int texture, bool lightingOn)
		{
			assert(m_pData->m_pSmgr);
			auto node = m_pData->m_pSmgr->addSphereSceneNode(radius);
			m_pData->AddSceneNode(pActor, node, texture, lightingOn);
		}

		void IrrlichtDisplay::VAddCubeSceneNode(float dim, ActorPtr pActor, unsigned int texture, bool lightingOn)
		{
			assert(m_pData->m_pSmgr);
			auto node = m_pData->m_pSmgr->addCubeSceneNode(dim);
			m_pData->AddSceneNode(pActor, node, texture, lightingOn);
		}

		void IrrlichtDisplay::VAddMeshSceneNode(const std::string& meshFilePath, ActorPtr pActor, unsigned int texture, bool lightingOn)
		{
			assert(m_pData->m_pSmgr);
			auto mesh = m_pData->m_pSmgr->getMesh(meshFilePath.c_str());
			if (mesh)
			{
				auto node = m_pData->m_pSmgr->addAnimatedMeshSceneNode(mesh);
				m_pData->AddSceneNode(pActor, node, texture, lightingOn);
			}
		}

		void IrrlichtDisplay::VAddLightSceneNode(const Vec3& position, const RGBAColor& color, float lightRadius)
		{
			assert(m_pData->m_pSmgr);
			irr::video::SColorf irrColor(color.r(), color.g(), color.b(), color.a());
			m_pData->m_pSmgr->addLightSceneNode(0, ConvertVector(position), irrColor, lightRadius);
		}

		void IrrlichtDisplay::VSetGlobalAmbientLight(const RGBAColor& color)
		{
			assert(m_pData->m_pSmgr);
			m_pData->m_pSmgr->setAmbientLight(ConvertRGBAColorToSColorf(color));
		}

		void IrrlichtDisplay::VRemoveSceneNode(ActorID actorId)
		{
			auto node = m_pData->sceneNodes[actorId];
			if (node)
			{
				node->remove();
			}
		}

		void IrrlichtDisplay::VSetSceneNodeLighting(ActorID actorId, bool lightingOn)
		{
			auto node = m_pData->sceneNodes[actorId];
			if (node)
			{
				node->setMaterialFlag(irr::video::EMF_LIGHTING, lightingOn);
			}
		}

		void IrrlichtDisplay::VSetSceneNodeLightColors(ActorID actorId, const RGBAColor& specularColor,
				const RGBAColor& ambientColor, const RGBAColor& diffuseColor)
		{
			auto node = m_pData->sceneNodes[actorId];
			if (node && node->getMaterialCount() > 0)
			{
				irr::video::SMaterial& material = node->getMaterial(0);
				material.AmbientColor = ConvertRGBAColorToSColor(ambientColor);
				material.SpecularColor = ConvertRGBAColorToSColor(specularColor);
				material.DiffuseColor = ConvertRGBAColorToSColor(diffuseColor);
			}
		}

		void IrrlichtDisplay::VSetSceneNodeShininess(ActorID actorId, float shininess)
		{
			auto node = m_pData->sceneNodes[actorId];
			if (node && node->getMaterialCount() > 0)
			{
				irr::video::SMaterial& material = node->getMaterial(0);
				material.Shininess = shininess;
			}
		}

		void IrrlichtDisplay::VLoadMap(const std::string& mapFilePath,
			const std::string& meshName, Vec3& position)
		{
			assert(m_pData->m_pDevice && m_pData->m_pSmgr);
			if (!m_pData->m_pDevice || !m_pData->m_pSmgr)
				return;
			m_pData->m_pDevice->getFileSystem()->addFileArchive(mapFilePath.c_str());

			ISceneManager *pSmgr = m_pData->m_pSmgr;
			IAnimatedMesh *mesh = pSmgr->getMesh(meshName.c_str());
			ISceneNode *node = nullptr;

			if (mesh)
			{
				node = pSmgr->addOctreeSceneNode(mesh->getMesh(0), 0, -1, 1024);
			}
			if (node)
			{
				node->setPosition(ConvertVector(position));
			}
		}

		Vec3 IrrlichtDisplay::VGetCameraTarget() const
		{
			assert(m_pData->m_pCamera);
			auto cameraLookAt = m_pData->m_pCamera->getTarget();
			return ConvertVector(cameraLookAt);
		}

		Vec3 IrrlichtDisplay::VGetCameraUpVector() const
		{
			assert(m_pData->m_pCamera);
			auto cameraUp = m_pData->m_pCamera->getUpVector();
			return ConvertVector(cameraUp);
		}

		Vec3 IrrlichtDisplay::VGetCameraRightVector() const
		{
			assert(m_pData->m_pCamera);
			auto camera = m_pData->m_pCamera;
			auto up = camera->getUpVector();
			auto forward = camera->getTarget() - camera->getAbsolutePosition();
			auto rightVector = up.crossProduct(forward).normalize();

			return ConvertVector(rightVector);
		}

		Quaternion IrrlichtDisplay::VGetCameraRotation() const
		{
			assert(m_pData->m_pCamera);
			return EulerToQuaternion(m_pData->m_pCamera->getRotation());
		}

		float IrrlichtDisplay::VGetCameraFOV() const
		{
			assert(m_pData->m_pCamera);
			return m_pData->m_pCamera->getFOV() * irr::core::RADTODEG;
		}

		float IrrlichtDisplay::VGetCameraNearPlaneDistance() const
		{
			assert(m_pData->m_pCamera);
			return m_pData->m_pCamera->getNearValue();
		}

		float IrrlichtDisplay::VGetCameraFarPlaneDistance() const
		{
			assert(m_pData->m_pCamera);
			return m_pData->m_pCamera->getFarValue();
		}

		/*
		 * Implementation of the IrrlichtDisplayData PIMPL struct (helper methods).
		 */
		ISceneNode *IrrlichtDisplayData::GetSceneNode(ActorID actorId)
		{
			auto it = sceneNodes.find(actorId);
			assert(it != sceneNodes.end());
			if (it != sceneNodes.end())
				return it->second;
			return nullptr;
		}

		unsigned int IrrlichtDisplayData::GetTime()
		{
			assert(m_pDevice);
			return m_pDevice->getTimer()->getTime();
		}

		void IrrlichtDisplayData::AddSceneNode(ActorPtr pActor, irr::scene::ISceneNode *pNode,
			unsigned int texture, bool lightingOn)
		{
			if (!pActor)
			{
				pNode->remove();
				return;
			}

			auto texturePos = textures.find(texture);
			if (texturePos != textures.end())
			{
				pNode->setMaterialTexture(0, (*texturePos).second);
				pNode->setMaterialFlag(irr::video::EMF_LIGHTING, lightingOn);
			}
			else
			{
				pNode->setMaterialFlag(irr::video::EMF_WIREFRAME, true);
				pNode->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
				pNode->setMaterialFlag(irr::video::EMF_LIGHTING, false);
			}

			SetNodeTransform(pNode, pActor->GetWorldTransform());

			sceneNodes[pActor->GetID()] = pNode;
			auto game = GameData::GetInstance();
			game->GetEventManager()->VRegisterHandler(EventType::ACTOR_MOVED,
				m_pMoveEventHandler);
		}

		void IrrlichtDisplayData::UpdateActorPosition(EventPtr pEvent)
		{
			assert(pEvent->VGetEventType() == EventType::ACTOR_MOVED);
			std::shared_ptr<ActorMoveEvent> pMoveEvent =
				std::dynamic_pointer_cast<ActorMoveEvent>(pEvent);

			auto pGame = GameData::GetInstance();
			ActorPtr pActor = pGame->GetActor(pMoveEvent->GetActorId());
			ISceneNode *pNode = GetSceneNode(pActor->GetID());

			if (pActor && pNode)
			{
				SetNodeTransform(pNode, pActor->GetWorldTransform());
			}
		}

		void IrrlichtDisplayData::SetNodeTransform(
			irr::scene::ISceneNode *pNode, const WorldTransformComponent& worldTransform)
		{
			pNode->setRotation(QuaternionToEuler(worldTransform.GetRotation()) * irr::core::RADTODEG);
			pNode->setPosition(ConvertVector(worldTransform.GetPosition()));
			pNode->setScale(ConvertVector(worldTransform.GetScale()));
		}

		/*
		 * Implementation of the IrrlichtMessagingWindow class.
		 */
		IrrlichtMessagingWindow::IrrlichtMessagingWindow(std::size_t messageBufferSize, irr::gui::IGUIEnvironment *irrlichtGUI)
			: m_color(255, 255, 255, 255), m_maxMessages(messageBufferSize),
			m_posX(0), m_posY(0), m_visible(false), m_pFont(nullptr), m_width(400),
			m_irrlichtGUI(irrlichtGUI)
		{
			if (m_irrlichtGUI)
			{
				m_pFont = m_irrlichtGUI->getBuiltInFont();
				SetFontHeight();
			}
		}

		void IrrlichtMessagingWindow::SetFontHeight()
		{
			if (m_pFont)
			{
				m_fontHeight = m_pFont->getDimension(L"I").Height + 2;
			}
		}

		void IrrlichtMessagingWindow::VAddMessage(const std::wstring& message)
		{
			if (m_messageBuffer.size() == m_maxMessages)
			{
				m_messageBuffer.erase(m_messageBuffer.begin());
			}
			m_messageBuffer.push_back(irr::core::stringw(message.c_str()));
		}

		void IrrlichtMessagingWindow::VSetPosition(unsigned int minX, unsigned int minY)
		{
			m_posX = minX;
			m_posY = minY;
		}

		void IrrlichtMessagingWindow::VSetFont(const std::string& fontFilePath)
		{
			if (m_irrlichtGUI)
			{
				m_pFont = m_irrlichtGUI->getFont(fontFilePath.c_str());
				SetFontHeight();
			}
		}

		void IrrlichtMessagingWindow::VSetVisible(bool visible)
		{
			m_visible = visible;
		}

		void IrrlichtMessagingWindow::VSetWidth(unsigned int width)
		{
			m_width = width;
		}

		void IrrlichtMessagingWindow::VRender() const
		{
			assert(m_pFont);
			if (!m_pFont || !m_visible)
				return;

			unsigned int x = m_posX;
			unsigned int y = m_posY;
			for (std::size_t i = 0; i < m_messageBuffer.size(); i++)
			{
				m_pFont->draw(
					m_messageBuffer[i], irr::core::rect<irr::s32>(
					x, y, x + m_width, y + m_fontHeight), m_color);
				y += m_fontHeight;
			}
		}
	}
}