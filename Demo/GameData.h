#ifndef GAME_DATA_H
#define GAME_DATA_H

#include "enginefwd.h"
#include <map>

namespace GameEngine
{
	class GameData
	{
	private:
		static GameData *instance;
		GameData() : m_pRenderer(nullptr), m_pPhysicsEngine(nullptr) { }
		Display::IRenderer *m_pRenderer;
		ITimer *m_pTimer;
		PhysicsEngine::IPhysicsEngine *m_pPhysicsEngine;
		std::map<ActorID, GameActor*> m_actors;
	public:
		static GameData *getInstance()
		{
			if (!instance)
			{
				instance = new GameData();
			}
			return instance;
		}
		~GameData();

		unsigned int CurrentTime();

		void AddActor(GameActor *actor);
		GameActor *GetActor(ActorID id)
		{
			return m_actors[id];
		}
		void SetPhysicsEngine(PhysicsEngine::IPhysicsEngine *physics)
		{
			m_pPhysicsEngine = physics;
		}
		PhysicsEngine::IPhysicsEngine * const GetPhysicsEngine() const
		{
			return m_pPhysicsEngine;
		}
		void SetRenderer(Display::IRenderer *renderer)
		{
			m_pRenderer = renderer;
		}
		Display::IRenderer * const GetRenderer() const
		{
			return m_pRenderer;
		}
		void setTimer(ITimer *timer)
		{
			m_pTimer = timer;
		}
		const ITimer * const Timer() const
		{
			return m_pTimer;
		}
	};
}

#endif