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

    constexpr int ARRV1_GROUP = 1;
    constexpr int ARRV2_GROUP = 2;
    constexpr int TANK_GROUP = 3;
    constexpr int IFV_GROUP = 4;
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
    // Начальная расстановка
    if (world.getTickIndex() == 0) {
/*        {
            Move m;
            m.setAction(ACTION_CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setBottom(world.getHeight());
            m.setRight(world.getWidth());
            m.setVehicleType(VEHICLE_ARRV);
            moveQueue_.push(m);

            m.setAction(ACTION_MOVE);
            m.setX(world.getWidth()/2);
            m.setY(world.getHeight()/2);
            moveQueue_.push(m);
        }*/

        // поделить БРЭМ на 2 группы
        double arrv_x_min = world.getWidth();
        double arrv_x_max = 0.;
        std::for_each(vehicles_.cbegin(), vehicles_.cend(), [&](VehicleById::const_reference v) {
            if (v.second.getPlayerId() == me.getId() && v.second.getType() == VEHICLE_ARRV)
            {
                arrv_x_min = std::min(arrv_x_min, v.second.getX());
                arrv_x_max = std::max(arrv_x_max, v.second.getX());
            }
        });
        {
            Move m;
            m.setAction(ACTION_CLEAR_AND_SELECT);
            m.setLeft(0.);
            m.setTop(0.);
            m.setBottom(world.getHeight());
            m.setRight((arrv_x_min + arrv_x_max)/2.);
            m.setVehicleType(VEHICLE_ARRV);
            moveQueue_.push(m);

            m.setAction(ACTION_ASSIGN);
            m.setGroup(ARRV1_GROUP);
            moveQueue_.push(m);

            m.setAction(ACTION_CLEAR_AND_SELECT);
            m.setLeft(m.getRight());
            m.setRight(world.getWidth());
            m.setGroup(0);
            moveQueue_.push(m);

            m.setAction(ACTION_ASSIGN);
            m.setGroup(ARRV2_GROUP);
            moveQueue_.push(m);
        }

        {
            double tank_x_cur = world.getWidth();
            double tank_y_cur = world.getHeight();
            double plane_x_cur = tank_x_cur;
            double plane_y_cur = tank_y_cur;
            double ifv_x_cur = tank_x_cur;
            double ifv_y_cur = tank_y_cur;
            double heli_x_cur = tank_x_cur;
            double heli_y_cur = tank_y_cur;
            double tank_x_start, tank_y_start, ifv_x_start, ifv_y_start;
            std::for_each(vehicles_.begin(), vehicles_.end(), [&](VehicleById::const_reference v) {
                if (v.second.getPlayerId() == me.getId()) {
                    switch (v.second.getType()) {
                        case VEHICLE_TANK:
                            tank_x_cur = std::min(tank_x_cur, v.second.getX());
                            tank_y_cur = std::min(tank_y_cur, v.second.getY());
                            break;
                        case VEHICLE_FIGHTER:
                            plane_x_cur = std::min(plane_x_cur, v.second.getX());
                            plane_y_cur = std::min(plane_y_cur, v.second.getY());
                            break;
                        case VEHICLE_IFV:
                            ifv_x_cur = std::min(ifv_x_cur, v.second.getX());
                            ifv_y_cur = std::min(ifv_y_cur, v.second.getY());
                            break;
                        case VEHICLE_HELICOPTER:
                            heli_x_cur = std::min(heli_x_cur, v.second.getX());
                            heli_y_cur = std::min(heli_y_cur, v.second.getY());
                            break;
                        default:
                            ;
                    }
                }
            });
//            if (tank_x_cur > tank_y_cur) {
//                tank_x_start = tank_x_cur;
//                tank_y_start = 0;
//            } else {
//                tank_x_start = 0;
//                tank_y_start = tank_y_cur;
//            }
//            if (ifv_x_cur > ifv_y_cur) {
//                ifv_x_start = ifv_x_cur;
//                ifv_y_start = 0;
//            } else {
//                ifv_x_start = 0;
//                ifv_y_start = ifv_y_cur;
//            }
            tank_x_start = tank_x_cur;
            tank_y_start = tank_y_cur;
            ifv_x_start = ifv_x_cur;
            ifv_y_start = ifv_y_cur;

            // собрать вместе танки и самолёты
            Move m;
            m.setAction(ACTION_CLEAR_AND_SELECT);
            m.setLeft(0.);
            m.setTop(0.);
            m.setBottom(world.getHeight());
            m.setRight(world.getWidth());
            m.setVehicleType(VEHICLE_TANK);
            moveQueue_.push(m);

            m.setAction(ACTION_MOVE);
            m.setX(tank_x_start - tank_x_cur);
            m.setY(tank_y_start - tank_y_cur);
            moveQueue_.push(m);

            m.setAction(ACTION_CLEAR_AND_SELECT);
            m.setVehicleType(VEHICLE_FIGHTER);
            moveQueue_.push(m);

            m.setAction(ACTION_MOVE);
            m.setX(tank_x_start - plane_x_cur);
            m.setY(tank_y_start - plane_y_cur);
            moveQueue_.push(m);

            m.setAction(ACTION_ADD_TO_SELECTION);
            m.setVehicleType(VEHICLE_TANK);
            moveQueue_.push(m);

            m.setAction(ACTION_ASSIGN);
            m.setGroup(TANK_GROUP);
            moveQueue_.push(m);
            m.setGroup(0);

            // собрать вместе БТР и вертолёты
            m.setAction(ACTION_CLEAR_AND_SELECT);
            m.setVehicleType(VEHICLE_IFV);
            moveQueue_.push(m);

            m.setAction(ACTION_MOVE);
            m.setX(ifv_x_start - ifv_x_cur);
            m.setY(ifv_y_start - ifv_y_cur);
            moveQueue_.push(m);

            m.setAction(ACTION_CLEAR_AND_SELECT);
            m.setVehicleType(VEHICLE_HELICOPTER);
            moveQueue_.push(m);

            m.setAction(ACTION_MOVE);
            m.setX(ifv_x_start - heli_x_cur);
            m.setY(ifv_y_start - heli_y_cur);
            moveQueue_.push(m);

            m.setAction(ACTION_ADD_TO_SELECTION);
            m.setVehicleType(VEHICLE_IFV);
            moveQueue_.push(m);

            m.setAction(ACTION_ASSIGN);
            m.setGroup(IFV_GROUP);
            moveQueue_.push(m);
        }

        return;
    }

    if (world.getTickIndex() % 300 == 0) {
        double x[_VEHICLE_COUNT_] = {0.}, y[_VEHICLE_COUNT_] = {0.};
        int cnt[_VEHICLE_COUNT_] = {0};
        double targetX[_VEHICLE_COUNT_] = {0.}, targetY[_VEHICLE_COUNT_] = {0.};
        int targetCnt[_VEHICLE_COUNT_] = {0};
        bool planeBusy = false, heliBusy = false;
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() == me.getId()) {
                x[v.second.getType()] += v.second.getX();
                y[v.second.getType()] += v.second.getY();
                ++cnt[v.second.getType()];
            } else {
                targetX[v.second.getType()] += v.second.getX();
                targetY[v.second.getType()] += v.second.getY();
                ++targetCnt[v.second.getType()];
            }
        }
        for (int i = 0; i < _VEHICLE_COUNT_; ++i) {
            if (cnt[i]) {
                x[i] /= cnt[i];
                y[i] /= cnt[i];
            } else {
                x[i] = NAN;
                y[i] = NAN;
            }
            if (targetCnt[i]) {
                targetX[i] /= targetCnt[i];
                targetY[i] /= targetCnt[i];
            } else {
                targetX[i] = NAN;
                targetY[i] = NAN;
            }
        }

        if (!isnan(x[VEHICLE_TANK]) && !isnan(y[VEHICLE_TANK])) {
            VehicleType targetType = getPreferredTargetType(VEHICLE_TANK, me, world, game);
            double dx = (targetX[targetType] - x[VEHICLE_TANK])/2.;
            double dy = (targetY[targetType] - y[VEHICLE_TANK])/2.;
            const double dd = dx*dx + dy*dy;
            if (dd > 10000) {
                const double k = dd/10000;
                dx /= k;
                dy /= k;
            }

            Move m;

            m.setAction(ACTION_CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(world.getWidth());
            m.setBottom(world.getHeight());
            m.setVehicleType(VEHICLE_TANK);
            moveQueue_.push(m);

            m.setAction(ACTION_MOVE);
            m.setX(dx);
            m.setY(dy);
            moveQueue_.push(m);

            if (!isnan(x[VEHICLE_FIGHTER]) && !isnan(y[VEHICLE_FIGHTER])) {
                m.setAction(ACTION_CLEAR_AND_SELECT);
                m.setVehicleType(VEHICLE_FIGHTER);
                moveQueue_.push(m);

                m.setAction(ACTION_MOVE);
                m.setX(x[VEHICLE_TANK] - x[VEHICLE_FIGHTER] + dx);
                m.setY(y[VEHICLE_TANK] - y[VEHICLE_FIGHTER] + dy);
                m.setMaxSpeed(game.getTankSpeed());
                moveQueue_.push(m);

                planeBusy = true;
            }
        }

        if (!isnan(x[VEHICLE_IFV]) && !isnan(y[VEHICLE_IFV])) {
            VehicleType targetType = getPreferredTargetType(VEHICLE_IFV, me, world, game);
            double dx = (targetX[targetType] - x[VEHICLE_IFV])/2.;
            double dy = (targetY[targetType] - y[VEHICLE_IFV])/2.;
            const double dd = dx*dx + dy*dy;
            if (dd > 10000) {
                const double k = dd/10000;
                dx /= k;
                dy /= k;
            }

            Move m;

            m.setAction(ACTION_CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(world.getWidth());
            m.setBottom(world.getHeight());
            m.setVehicleType(VEHICLE_IFV);
            moveQueue_.push(m);

            m.setAction(ACTION_MOVE);
            m.setX(dx);
            m.setY(dy);
            moveQueue_.push(m);

            if (!isnan(x[VEHICLE_HELICOPTER]) && !isnan(y[VEHICLE_HELICOPTER])) {
                m.setAction(ACTION_CLEAR_AND_SELECT);
                m.setVehicleType(VEHICLE_HELICOPTER);
                moveQueue_.push(m);

                m.setAction(ACTION_MOVE);
                m.setX(x[VEHICLE_IFV] - x[VEHICLE_HELICOPTER] + dx);
                m.setY(y[VEHICLE_IFV] - y[VEHICLE_HELICOPTER] + dy);
                m.setMaxSpeed(game.getIfvSpeed());
                moveQueue_.push(m);

                heliBusy = true;
            }
        }

        if (!planeBusy && !isnan(x[VEHICLE_FIGHTER]) && !isnan(y[VEHICLE_FIGHTER])) {
            const auto targetType = getPreferredTargetType(VEHICLE_FIGHTER, me, world, game);
            if (targetType != _VEHICLE_UNKNOWN_) {
                Move m;
                m.setAction(ACTION_CLEAR_AND_SELECT);
                m.setVehicleType(VEHICLE_FIGHTER);
                moveQueue_.push(m);

                m.setAction(ACTION_MOVE);
                m.setX(targetX[targetType] - x[VEHICLE_FIGHTER]);
                m.setY(targetY[targetType] - y[VEHICLE_FIGHTER]);
                moveQueue_.push(m);
            }
        }

        if (!heliBusy && !isnan(x[VEHICLE_HELICOPTER]) && !isnan(y[VEHICLE_HELICOPTER])) {
            const auto targetType = getPreferredTargetType(VEHICLE_HELICOPTER, me, world, game);
            if (targetType != _VEHICLE_UNKNOWN_) {
                Move m;
                m.setAction(ACTION_CLEAR_AND_SELECT);
                m.setVehicleType(VEHICLE_HELICOPTER);
                moveQueue_.push(m);

                m.setAction(ACTION_MOVE);
                m.setX(targetX[targetType] - x[VEHICLE_HELICOPTER]);
                m.setY(targetY[targetType] - y[VEHICLE_HELICOPTER]);
                moveQueue_.push(m);
            }
        }

        double arrvx[2] = {0.}, arrvy[2] = {0.};
        int arrvcnt[2] = {0};
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() == me.getId() && v.second.getType() == VEHICLE_ARRV) {
                arrvx[v.second.getGroups()[0] - 1] += v.second.getX();
                arrvy[v.second.getGroups()[0] - 1] += v.second.getY();
                ++arrvcnt[v.second.getGroups()[0] - 1];
            }
        }
        for (int i = 0; i < 2; ++i) {
            if (arrvcnt[i]) {
                arrvx[i] /= arrvcnt[i];
                arrvy[i] /= arrvcnt[i];
            } else {
                arrvx[i] = NAN;
                arrvy[i] = NAN;
            }
        }

        if (!isnan(arrvx[0]) && !isnan(arrvy[0])) {
            Move m;

            if (!isnan(x[VEHICLE_TANK]) && !isnan(y[VEHICLE_TANK])) {
                m.setAction(ACTION_CLEAR_AND_SELECT);
                m.setGroup(ARRV1_GROUP);
                moveQueue_.push(m);

//                m.setAction(ACTION_MOVE);
//                m.setX((x[VEHICLE_TANK] - arrvx[0]) * 1.2);
//                m.setY((y[VEHICLE_TANK] - arrvy[0]) * 1.2);
                m.setAction(ACTION_ROTATE);
                m.setX((x[VEHICLE_TANK] + arrvx[0])/2.);
                m.setY((y[VEHICLE_TANK] + arrvy[0])/2.);
                m.setAngle((rand() % 2? 1.: -1.) * PI);
                moveQueue_.push(m);
            } else if (!isnan(x[VEHICLE_IFV]) && !isnan(y[VEHICLE_IFV])) {
                m.setAction(ACTION_CLEAR_AND_SELECT);
                m.setGroup(ARRV1_GROUP);
                moveQueue_.push(m);

//                m.setAction(ACTION_MOVE);
//                m.setX((x[VEHICLE_IFV] - arrvx[0]) * 1.2);
//                m.setY((y[VEHICLE_IFV] - arrvy[0]) * 1.2);
                m.setAction(ACTION_ROTATE);
                m.setX((x[VEHICLE_IFV] + arrvx[0])/2.);
                m.setY((y[VEHICLE_IFV] + arrvy[0])/2.);
                m.setAngle((rand() % 2? 1.: -1.) * PI);
                moveQueue_.push(m);
            }
        }

        if (!isnan(arrvx[1]) && !isnan(arrvy[1])) {
            Move m;

            if (!isnan(x[VEHICLE_IFV]) && !isnan(y[VEHICLE_IFV])) {
                m.setAction(ACTION_CLEAR_AND_SELECT);
                m.setGroup(ARRV2_GROUP);
                moveQueue_.push(m);

//                m.setAction(ACTION_MOVE);
//                m.setX((x[VEHICLE_IFV] - arrvx[1]) * 1.2);
//                m.setY((y[VEHICLE_IFV] - arrvy[1]) * 1.2);
                m.setAction(ACTION_ROTATE);
                m.setX((x[VEHICLE_IFV] + arrvx[1])/2.);
                m.setY((y[VEHICLE_IFV] + arrvy[1])/2.);
                m.setAngle((rand() % 2? 1.: -1.) * PI);
                moveQueue_.push(m);
            } else if (!isnan(x[VEHICLE_TANK]) && !isnan(y[VEHICLE_TANK])) {
                m.setAction(ACTION_CLEAR_AND_SELECT);
                m.setGroup(ARRV2_GROUP);
                moveQueue_.push(m);

//                m.setAction(ACTION_MOVE);
//                m.setX((x[VEHICLE_TANK] - arrvx[1]) * 1.2);
//                m.setY((y[VEHICLE_TANK] - arrvy[1]) * 1.2);
                m.setAction(ACTION_ROTATE);
                m.setX((x[VEHICLE_TANK] + arrvx[1])/2.);
                m.setY((y[VEHICLE_TANK] + arrvy[1])/2.);
                m.setAngle((rand() % 2? 1.: -1.) * PI);
                moveQueue_.push(m);
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
            return VEHICLE_FIGHTER;
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
