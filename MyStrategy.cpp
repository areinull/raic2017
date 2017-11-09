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

MyStrategy::MyStrategy() {
    ctx_.vehicleById = &vehicles_;
    blobs_[(int)VehicleType::FIGHTER] = Blob(VehicleType::FIGHTER);
    blobs_[(int)VehicleType::HELICOPTER] = Blob(VehicleType::HELICOPTER);
    blobs_[(int)VehicleType::TANK] = Blob(VehicleType::TANK);
    blobs_[(int)VehicleType::IFV] = Blob(VehicleType::IFV);
    blobs_[(int)VehicleType::ARRV] = Blob(VehicleType::ARRV);
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

void MyStrategy::initializeTick(const Player &me, const World &world, const Game &game, const Move &) {
    ctx_.me = &me;
    ctx_.world = &world;
    ctx_.game = &game;

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

    for (auto &b: blobs_) {
        b.update(ctx_);
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
/*
    // Начальная расстановка
    if (world.getTickIndex() == 0) {

        // поделить БРЭМ на 2 группы
        double arrv_x_min = world.getWidth();
        double arrv_x_max = 0.;
        std::for_each(vehicles_.cbegin(), vehicles_.cend(), [&](VehicleById::const_reference v) {
            if (v.second.getPlayerId() == me.getId() && v.second.getType() == VehicleType::ARRV)
            {
                arrv_x_min = std::min(arrv_x_min, v.second.getX());
                arrv_x_max = std::max(arrv_x_max, v.second.getX());
            }
        });
        {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0.);
            m.setTop(0.);
            m.setBottom(world.getHeight());
            m.setRight((arrv_x_min + arrv_x_max)/2.);
            m.setVehicleType(VehicleType::ARRV);
            moveQueue_.push(m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(ARRV1_GROUP);
            moveQueue_.push(m);

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(m.getRight());
            m.setRight(world.getWidth());
            m.setGroup(0);
            moveQueue_.push(m);

            m.setAction(ActionType::ASSIGN);
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
                        case VehicleType::TANK:
                            tank_x_cur = std::min(tank_x_cur, v.second.getX());
                            tank_y_cur = std::min(tank_y_cur, v.second.getY());
                            break;
                        case VehicleType::FIGHTER:
                            plane_x_cur = std::min(plane_x_cur, v.second.getX());
                            plane_y_cur = std::min(plane_y_cur, v.second.getY());
                            break;
                        case VehicleType::IFV:
                            ifv_x_cur = std::min(ifv_x_cur, v.second.getX());
                            ifv_y_cur = std::min(ifv_y_cur, v.second.getY());
                            break;
                        case VehicleType::HELICOPTER:
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
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0.);
            m.setTop(0.);
            m.setBottom(world.getHeight());
            m.setRight(world.getWidth());
            m.setVehicleType(VehicleType::TANK);
            moveQueue_.push(m);

            m.setAction(ActionType::MOVE);
            m.setX(tank_x_start - tank_x_cur);
            m.setY(tank_y_start - tank_y_cur);
            moveQueue_.push(m);

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setVehicleType(VehicleType::FIGHTER);
            moveQueue_.push(m);

            m.setAction(ActionType::MOVE);
            m.setX(tank_x_start - plane_x_cur);
            m.setY(tank_y_start - plane_y_cur);
            moveQueue_.push(m);

            m.setAction(ActionType::ADD_TO_SELECTION);
            m.setVehicleType(VehicleType::TANK);
            moveQueue_.push(m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(TANK_GROUP);
            moveQueue_.push(m);
            m.setGroup(0);

            // собрать вместе БТР и вертолёты
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setVehicleType(VehicleType::IFV);
            moveQueue_.push(m);

            m.setAction(ActionType::MOVE);
            m.setX(ifv_x_start - ifv_x_cur);
            m.setY(ifv_y_start - ifv_y_cur);
            moveQueue_.push(m);

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setVehicleType(VehicleType::HELICOPTER);
            moveQueue_.push(m);

            m.setAction(ActionType::MOVE);
            m.setX(ifv_x_start - heli_x_cur);
            m.setY(ifv_y_start - heli_y_cur);
            moveQueue_.push(m);

            m.setAction(ActionType::ADD_TO_SELECTION);
            m.setVehicleType(VehicleType::IFV);
            moveQueue_.push(m);

            m.setAction(ActionType::ASSIGN);
            m.setGroup(IFV_GROUP);
            moveQueue_.push(m);
        }

        return;
    }

    if (world.getTickIndex() % 300 == 0) {
        double x[_VehicleType::COUNT_] = {0.}, y[_VehicleType::COUNT_] = {0.};
        int cnt[_VehicleType::COUNT_] = {0};
        double targetX[_VehicleType::COUNT_] = {0.}, targetY[_VehicleType::COUNT_] = {0.};
        int targetCnt[_VehicleType::COUNT_] = {0};
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
        for (int i = 0; i < _VehicleType::COUNT_; ++i) {
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

        if (!isnan(x[VehicleType::TANK]) && !isnan(y[VehicleType::TANK])) {
            VehicleType targetType = getPreferredTargetType(VehicleType::TANK, me, world, game);
            double dx = (targetX[targetType] - x[VehicleType::TANK])/2.;
            double dy = (targetY[targetType] - y[VehicleType::TANK])/2.;
            const double dd = dx*dx + dy*dy;
            Move m;

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(world.getWidth());
            m.setBottom(world.getHeight());
            m.setVehicleType(VehicleType::TANK);
            moveQueue_.push(m);


            if (dd > 32*32) {
                if (dd > 150 * 150) {
                    const double k = dd/(150*150);
                    dx /= k;
                    dy /= k;
                }
                m.setAction(ActionType::MOVE);
                m.setX(dx);
                m.setY(dy);
            } else {
                m.setAction(ActionType::ROTATE);
                m.setAngle((rand()%2? 1.: -1.)*PI);
                m.setX(x[VehicleType::TANK] + dx);
                m.setY(y[VehicleType::TANK] + dy);
            }
            moveQueue_.push(m);

            if (!isnan(x[VehicleType::FIGHTER]) && !isnan(y[VehicleType::FIGHTER])) {
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setVehicleType(VehicleType::FIGHTER);
                moveQueue_.push(m);

                m.setAction(ActionType::MOVE);
                m.setX(x[VehicleType::TANK] - x[VehicleType::FIGHTER] + dx);
                m.setY(y[VehicleType::TANK] - y[VehicleType::FIGHTER] + dy);
                m.setMaxSpeed(game.getTankSpeed());
                moveQueue_.push(m);

                planeBusy = true;
            }
        }

        if (!isnan(x[VehicleType::IFV]) && !isnan(y[VehicleType::IFV])) {
            VehicleType targetType = getPreferredTargetType(VehicleType::IFV, me, world, game);
            double dx = (targetX[targetType] - x[VehicleType::IFV])/2.;
            double dy = (targetY[targetType] - y[VehicleType::IFV])/2.;
            const double dd = dx*dx + dy*dy;
            Move m;

            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(world.getWidth());
            m.setBottom(world.getHeight());
            m.setVehicleType(VehicleType::IFV);
            moveQueue_.push(m);

            if (dd > 32*32) {
                if (dd > 150 * 150) {
                    const double k = dd/(150 * 150);
                    dx /= k;
                    dy /= k;
                }
                m.setAction(ActionType::MOVE);
                m.setX(dx);
                m.setY(dy);
            } else {
                m.setAction(ActionType::ROTATE);
                m.setAngle((rand()%2? 1.: -1.)*PI);
                m.setX(x[VehicleType::IFV] + dx);
                m.setY(y[VehicleType::IFV] + dy);
            }
            moveQueue_.push(m);

            if (!isnan(x[VehicleType::HELICOPTER]) && !isnan(y[VehicleType::HELICOPTER])) {
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setVehicleType(VehicleType::HELICOPTER);
                moveQueue_.push(m);

                m.setAction(ActionType::MOVE);
                m.setX(x[VehicleType::IFV] - x[VehicleType::HELICOPTER] + dx);
                m.setY(y[VehicleType::IFV] - y[VehicleType::HELICOPTER] + dy);
                m.setMaxSpeed(game.getIfvSpeed());
                moveQueue_.push(m);

                heliBusy = true;
            }
        }

        if (!planeBusy && !isnan(x[VehicleType::FIGHTER]) && !isnan(y[VehicleType::FIGHTER])) {
            const auto targetType = getPreferredTargetType(VehicleType::FIGHTER, me, world, game);
            if (targetType != _VehicleType::UNKNOWN_) {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setVehicleType(VehicleType::FIGHTER);
                moveQueue_.push(m);

                m.setAction(ActionType::MOVE);
                m.setX(targetX[targetType] - x[VehicleType::FIGHTER]);
                m.setY(targetY[targetType] - y[VehicleType::FIGHTER]);
                moveQueue_.push(m);
            }
        }

        if (!heliBusy && !isnan(x[VehicleType::HELICOPTER]) && !isnan(y[VehicleType::HELICOPTER])) {
            const auto targetType = getPreferredTargetType(VehicleType::HELICOPTER, me, world, game);
            if (targetType != _VehicleType::UNKNOWN_) {
                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setVehicleType(VehicleType::HELICOPTER);
                moveQueue_.push(m);

                m.setAction(ActionType::MOVE);
                m.setX(targetX[targetType] - x[VehicleType::HELICOPTER]);
                m.setY(targetY[targetType] - y[VehicleType::HELICOPTER]);
                moveQueue_.push(m);
            }
        }

        double arrvx[2] = {0.}, arrvy[2] = {0.};
        int arrvcnt[2] = {0};
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() == me.getId() && v.second.getType() == VehicleType::ARRV) {
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

            if (!isnan(x[VehicleType::TANK]) && !isnan(y[VehicleType::TANK])) {
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setGroup(ARRV1_GROUP);
                moveQueue_.push(m);

                if ((x[VehicleType::TANK] - arrvx[0])*(x[VehicleType::TANK] - arrvx[0]) + (y[VehicleType::TANK] - arrvy[0])*(y[VehicleType::TANK] - arrvy[0]) > 2000.) {
                    m.setAction(ActionType::MOVE);
                    m.setX((x[VehicleType::TANK] - arrvx[0]) * 1.2);
                    m.setY((y[VehicleType::TANK] - arrvy[0]) * 1.2);
                } else {
                    m.setAction(ActionType::ROTATE);
                    m.setX((x[VehicleType::TANK] + arrvx[0]) / 2.);
                    m.setY((y[VehicleType::TANK] + arrvy[0]) / 2.);
                    m.setAngle((rand() % 2 ? 1. : -1.) * PI);
                }
                moveQueue_.push(m);
            } else if (!isnan(x[VehicleType::IFV]) && !isnan(y[VehicleType::IFV])) {
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setGroup(ARRV1_GROUP);
                moveQueue_.push(m);

                if ((x[VehicleType::IFV] - arrvx[0])*(x[VehicleType::IFV] - arrvx[0]) + (y[VehicleType::IFV] - arrvy[0])*(y[VehicleType::IFV] - arrvy[0]) > 2000.) {
                    m.setAction(ActionType::MOVE);
                    m.setX((x[VehicleType::IFV] - arrvx[0]) * 1.2);
                    m.setY((y[VehicleType::IFV] - arrvy[0]) * 1.2);
                } else {
                    m.setAction(ActionType::ROTATE);
                    m.setX((x[VehicleType::IFV] + arrvx[0]) / 2.);
                    m.setY((y[VehicleType::IFV] + arrvy[0]) / 2.);
                    m.setAngle((rand() % 2 ? 1. : -1.) * PI);
                }
                moveQueue_.push(m);
            }
        }

        if (!isnan(arrvx[1]) && !isnan(arrvy[1])) {
            Move m;

            if (!isnan(x[VehicleType::IFV]) && !isnan(y[VehicleType::IFV])) {
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setGroup(ARRV2_GROUP);
                moveQueue_.push(m);

                if ((x[VehicleType::IFV] - arrvx[1])*(x[VehicleType::IFV] - arrvx[1]) + (y[VehicleType::IFV] - arrvy[1])*(y[VehicleType::IFV] - arrvy[1]) > 50.) {
                    m.setAction(ActionType::MOVE);
                    m.setX((x[VehicleType::IFV] - arrvx[1]) * 1.2);
                    m.setY((y[VehicleType::IFV] - arrvy[1]) * 1.2);
                } else {
                    m.setAction(ActionType::ROTATE);
                    m.setX((x[VehicleType::IFV] + arrvx[1]) / 2.);
                    m.setY((y[VehicleType::IFV] + arrvy[1]) / 2.);
                    m.setAngle((rand() % 2 ? 1. : -1.) * PI);
                }
                moveQueue_.push(m);
            } else if (!isnan(x[VehicleType::TANK]) && !isnan(y[VehicleType::TANK])) {
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setGroup(ARRV2_GROUP);
                moveQueue_.push(m);

                if ((x[VehicleType::TANK] - arrvx[1])*(x[VehicleType::TANK] - arrvx[1]) + (y[VehicleType::TANK] - arrvy[1])*(y[VehicleType::TANK] - arrvy[1]) > 50.) {
                    m.setAction(ActionType::MOVE);
                    m.setX((x[VehicleType::TANK] - arrvx[1]) * 1.2);
                    m.setY((y[VehicleType::TANK] - arrvy[1]) * 1.2);
                } else {
                    m.setAction(ActionType::ROTATE);
                    m.setX((x[VehicleType::TANK] + arrvx[1]) / 2.);
                    m.setY((y[VehicleType::TANK] + arrvy[1]) / 2.);
                    m.setAngle((rand() % 2 ? 1. : -1.) * PI);
                }
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
            tmp.setAction(ActionType::ROTATE);
            tmp.setX(x);
            tmp.setY(y);
            tmp.setAngle((rand()%360-180)*PI/180.);
            moveQueue_.push(tmp);
        }
    }
*/
    if (moveQueue_.empty()) {
        double targetX[(int)VehicleType::_COUNT_] = {0.}, targetY[(int)VehicleType::_COUNT_] = {0.};
        int targetCnt[(int)VehicleType::_COUNT_] = {0};
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() != me.getId()) {
                targetX[(int)v.second.getType()] += v.second.getX();
                targetY[(int)v.second.getType()] += v.second.getY();
                ++targetCnt[(int)v.second.getType()];
            }
        }
        for (int i = 0; i < (int)VehicleType::_COUNT_; ++i) {
            if (targetCnt[i]) {
                targetX[i] /= targetCnt[i];
                targetY[i] /= targetCnt[i];
            }
        }

        for (const auto vt: {VehicleType::TANK, VehicleType::IFV, VehicleType::FIGHTER, VehicleType::HELICOPTER}) {
            if (!blobs_[(int)vt].isAlive())
                continue;

            const auto tt = getPreferredTargetType(vt, me, world, game);
            moveQueue_.push(blobs_[(int)vt].select(ctx_));
            if (tt != VehicleType::_UNKNOWN_ && targetCnt[(int)tt]) {
                moveQueue_.push(blobs_[(int)vt].move(ctx_, (targetX[(int)tt] + blobs_[(int)vt].getX())/2., (targetY[(int)tt] + blobs_[(int)vt].getY())/2.));
            } else {
                moveQueue_.push(blobs_[(int)vt].rndWalk(ctx_));
            }
        }

        if (blobs_[(int)VehicleType::ARRV].isAlive()) {
            auto tankHpDeficit = blobs_[(int)VehicleType::TANK].getHpDeficit(ctx_);
            auto ifvHpDeficit = blobs_[(int)VehicleType::IFV].getHpDeficit(ctx_);
            moveQueue_.push(blobs_[(int)VehicleType::ARRV].select(ctx_));
            if (tankHpDeficit > ifvHpDeficit) {
                moveQueue_.push(blobs_[(int)VehicleType::ARRV].move(ctx_, blobs_[(int)VehicleType::TANK].getX(), blobs_[(int)VehicleType::TANK].getY()));
            } else if (ifvHpDeficit) {
                moveQueue_.push(blobs_[(int)VehicleType::ARRV].move(ctx_, blobs_[(int)VehicleType::IFV].getX(), blobs_[(int)VehicleType::IFV].getY()));
            } else {
                double x = 0.;
                double y = 0.;
                double cnt = 0;
                for (const auto &v: vehicles_) {
                    if (v.second.getPlayerId() == me.getId()) {
                        x += v.second.getX();
                        x += v.second.getY();
                        ++cnt;
                    }
                }
                if (cnt) {
                    x /= cnt;
                    y /= cnt;
                }
                moveQueue_.push(blobs_[(int)VehicleType::ARRV].move(ctx_, x, y));
            }
        }
    }
}

VehicleType MyStrategy::getPreferredTargetType(VehicleType vehicleType, const model::Player &me, const model::World &, const model::Game &) const {
    unsigned cnt[(int)VehicleType::_COUNT_] = {0};
    for (auto it = vehicles_.cbegin(); it != vehicles_.cend(); ++it) {
        if (it->second.getPlayerId() != me.getId()) {
            ++cnt[(int)it->second.getType()];
        }
    }

    switch (vehicleType) {
        case VehicleType::FIGHTER: {
            if (cnt[(int)VehicleType::FIGHTER]*1.2 > cnt[(int)VehicleType::HELICOPTER])
                return VehicleType::FIGHTER;
            else
                return VehicleType::HELICOPTER;
        }
        case VehicleType::HELICOPTER:
            if (cnt[(int)VehicleType::TANK]) return VehicleType::TANK;
            if (cnt[(int)VehicleType::IFV]) return VehicleType::IFV;
            if (cnt[(int)VehicleType::ARRV]) return VehicleType::ARRV;
            if (cnt[(int)VehicleType::HELICOPTER]) return VehicleType::HELICOPTER;
            return VehicleType::FIGHTER;
        case VehicleType::IFV:
            if (cnt[(int)VehicleType::HELICOPTER]) return VehicleType::HELICOPTER;
            if (cnt[(int)VehicleType::FIGHTER]) return VehicleType::FIGHTER;
            if (cnt[(int)VehicleType::IFV]) return VehicleType::IFV;
            if (cnt[(int)VehicleType::ARRV]) return VehicleType::ARRV;
            return VehicleType::TANK;
        case VehicleType::TANK:
            if (cnt[(int)VehicleType::IFV]) return VehicleType::IFV;
            if (cnt[(int)VehicleType::TANK]) return VehicleType::TANK;
            if (cnt[(int)VehicleType::ARRV]) return VehicleType::ARRV;
            if (cnt[(int)VehicleType::FIGHTER]) return VehicleType::FIGHTER;
            return VehicleType::HELICOPTER;
        default:
            return VehicleType::_UNKNOWN_;
    }
}
