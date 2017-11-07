#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <algorithm>

using namespace model;
using namespace std;

namespace {
    enum class Ownership {
        ANY,
        ALLY,
        ENEMY
    };
}

void MyStrategy::move(const Player& me, const World& world, const Game& game, Move& move) {
    initializeStrategy(game);
    initializeTick(me, world, game, move);

    if (me.getRemainingActionCooldownTicks() > 0) {
        return;
    }

    if (executeDelayedMove(move)) {
        return;
    }

    this->move(me, world, game);

    executeDelayedMove(move);
}

void MyStrategy::initializeStrategy(const model::Game &game) {
    static bool firstTime = true;
    if (firstTime) {
        srand(game.getRandomSeed());
    }
}

void MyStrategy::initializeTick(const Player &, const World &world, const Game &, const Move &) {
    for (const auto &vehicle : world.getNewVehicles()) {
        vehicles_[vehicle.getId()] = vehicle;
        updateTickByVehicleId_[vehicle.getId()] = world.getTickIndex();
    }

    for (const auto &vehicleUpdate : world.getVehicleUpdates()) {
        const auto &vehicleId = vehicleUpdate.getId();

        if (vehicleUpdate.getDurability() == 0) {
            vehicles_.erase(vehicleId);
            updateTickByVehicleId_.erase(vehicleId);
        } else {
            vehicles_[vehicleId] = Vehicle(vehicles_[vehicleId], vehicleUpdate);
            updateTickByVehicleId_[vehicleId] = world.getTickIndex();
        }
    }
}

bool MyStrategy::executeDelayedMove(Move& move) {
    if (moveQueue_.empty())
        return false;

    move = moveQueue_.front();
    moveQueue_.pop();

    return true;
}

void MyStrategy::move(const Player& me, const World& world, const Game& game) {
    // Каждые 300 тиков ...
    if (world.getTickIndex() % 300 == 0) {
        // ... для каждого типа техники ...
        for (int vehicleType = 0; vehicleType < _VEHICLE_COUNT_; ++vehicleType) {
            VehicleType targetType = getPreferredTargetType((VehicleType)vehicleType, me, world, game);

            // ... если этот тип может атаковать ...
            if (targetType == _VEHICLE_UNKNOWN_) {
                continue;
            }

            // ... получаем центр формации ...
            double x = 0.,
                   y = 0.;
            int cnt = 0;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() == me.getId() && v.second.getType() == vehicleType) {
                    x += v.second.getX();
                    y += v.second.getY();
                    ++cnt;
                }
            }
            if (cnt) {
                x /= cnt;
                y /= cnt;
            } else {
                x = NAN;
                y = NAN;
            }

            // ... получаем центр формации противника или центр мира ...
            double targetX = 0.,
                   targetY = 0.;
            cnt = 0;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() != me.getId() && v.second.getType() == vehicleType) {
                    targetX += v.second.getX();
                    targetY += v.second.getY();
                    ++cnt;
                }
            }
            if (cnt) {
                targetX /= cnt;
                targetY /= cnt;
            } else {
                targetX = world.getWidth()/2.;
                targetY = world.getHeight()/2.;
            }

            // .. и добавляем в очередь отложенные действия для выделения и перемещения техники.
            if (!isnan(x) && !isnan(y)) {
                Move tmp;

                tmp.setAction(ACTION_CLEAR_AND_SELECT);
                tmp.setRight(world.getWidth());
                tmp.setBottom(world.getHeight());
                tmp.setVehicleType((VehicleType)vehicleType);
                moveQueue_.push(tmp);

                tmp.setAction(ACTION_MOVE);
                tmp.setX(targetX - x);
                tmp.setY(targetY - y);
                moveQueue_.push(tmp);
            }
        }

        {
            // Также находим центр формации наших БРЭМ ...
            double x = 0.,
                   y = 0.;
            int cnt = 0;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() == me.getId() && v.second.getType() == VEHICLE_ARRV) {
                    x += v.second.getX();
                    y += v.second.getY();
                    ++cnt;
                }
            }
            if (cnt) {
                x /= cnt;
                y /= cnt;
            } else {
                x = NAN;
                y = NAN;
            }

            // .. и отправляем их в центр мира.
            if (!isnan(x) && !isnan(y)) {
                Move tmp;
                tmp.setAction(ACTION_CLEAR_AND_SELECT);
                tmp.setRight(world.getWidth());
                tmp.setBottom(world.getHeight());
                tmp.setVehicleType(VEHICLE_ARRV);
                moveQueue_.push(tmp);

                tmp.setAction(ACTION_MOVE);
                tmp.setX(world.getWidth() / 2.0 - x);
                tmp.setY(world.getHeight() / 2.0 - y);
                moveQueue_.push(tmp);
            }
        }

        return;
    }

    // Если ни один наш юнит не мог двигаться в течение 60 тиков ...
    if (std::all_of(vehicles_.begin(), vehicles_.end(), [&](const std::unordered_map<int, model::Vehicle>::value_type &v) {
        return v.second.getPlayerId() != me.getId() ||
                world.getTickIndex() - updateTickByVehicleId_[v.first] > 60;
    })) {
        /// ... находим центр нашей формации ...
        double x = 0.,
                y = 0.;
        int cnt = 0;
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() == me.getId()) {
                x += v.second.getX();
                y += v.second.getY();
                ++cnt;
            }
        }
        if (cnt) {
            x /= cnt;
            y /= cnt;
        } else {
            x = NAN;
            y = NAN;
        }

        // ... и поворачиваем её на случайный угол.
        if (!isnan(x) && !isnan(y)) {
            Move tmp;
            tmp.setAction(ACTION_ROTATE);
            tmp.setX(x);
            tmp.setY(y);
            tmp.setAngle((rand()%360-180)*PI/180.);
            moveQueue_.push(tmp);
        }
    }
}

VehicleType MyStrategy::getPreferredTargetType(VehicleType vehicleType, const model::Player &me, const model::World &, const model::Game &) const {
    unsigned cnt[_VEHICLE_COUNT_] = {0};
    for (auto it = vehicles_.cbegin(); it != vehicles_.cend(); ++it) {
        if (it->second.getPlayerId() != me.getId()) {
            ++cnt[it->second.getType()];
        }
    }

    switch (vehicleType) {
        case VEHICLE_FIGHTER: {
            if (cnt[VEHICLE_FIGHTER]*1.2 > cnt[VEHICLE_HELICOPTER])
                return VEHICLE_FIGHTER;
            else
                return VEHICLE_HELICOPTER;
        }
        case VEHICLE_HELICOPTER:
            if (cnt[VEHICLE_TANK]) return VEHICLE_TANK;
            if (cnt[VEHICLE_IFV]) return VEHICLE_IFV;
            if (cnt[VEHICLE_ARRV]) return VEHICLE_ARRV;
            if (cnt[VEHICLE_HELICOPTER]) return VEHICLE_HELICOPTER;
        case VEHICLE_IFV:
            if (cnt[VEHICLE_HELICOPTER]) return VEHICLE_HELICOPTER;
            if (cnt[VEHICLE_IFV]) return VEHICLE_IFV;
            if (cnt[VEHICLE_ARRV]) return VEHICLE_ARRV;
            if (cnt[VEHICLE_FIGHTER]) return VEHICLE_FIGHTER;
            return VEHICLE_TANK;
        case VEHICLE_TANK:
            if (cnt[VEHICLE_IFV]) return VEHICLE_IFV;
            if (cnt[VEHICLE_ARRV]) return VEHICLE_ARRV;
            if (cnt[VEHICLE_TANK]) return VEHICLE_TANK;
            if (cnt[VEHICLE_FIGHTER]) return VEHICLE_FIGHTER;
            return VEHICLE_HELICOPTER;
        default:
            return _VEHICLE_UNKNOWN_;
    }
}
