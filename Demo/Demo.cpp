// Demo.cpp : Defines the entry point for the console application.
//

#include "Game.h"
#include "XMLPhysicsData.h"
#include <string>
#include <iostream>

using std::string;
using std::cout;
using std::endl;

void dump_map(const std::map<string, float> map) {
  for ( std::map<string,float>::const_iterator it = map.begin(); it != map.end(); it++) {
    cout << "Key: " << it->first << endl;
    cout << "Val: " << it->second << endl;
  }
}

int main()
{
	GameEngine::PhysicsEngine::XMLPhysicsData data;
	data.LoadDataFromXML("..\\assets\\materials.xml");

	dump_map(data.m_densities);

	GameEngine::Game game;

	return game.Run();
}

