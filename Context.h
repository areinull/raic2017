#pragma once

#ifndef MYSTRATEGY_CONTEXT_H
#define MYSTRATEGY_CONTEXT_H

#include <unordered_map>

namespace model {
    class Game;
    class Player;
    class World;
    class Vehicle;
}

using VehicleById = std::unordered_map<int, model::Vehicle>;

struct Context {
    const model::Player *me;
    const model::World *world;
    const model::Game *game;
    VehicleById *vehicleById;
};


#endif //MYSTRATEGY_CONTEXT_H
