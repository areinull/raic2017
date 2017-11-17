#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>

using namespace model;
using namespace std;

std::ostream& operator<<(std::ostream &s, VehicleType vt) {
    switch (vt) {
        case VehicleType::_UNKNOWN_:  s << "VehicleType::_UNKNOWN_"; break;
        case VehicleType::ARRV:       s << "VehicleType::ARRV"; break;
        case VehicleType::IFV:        s << "VehicleType::IFV"; break;
        case VehicleType::TANK:       s << "VehicleType::TANK"; break;
        case VehicleType::HELICOPTER: s << "VehicleType::HELICOPTER"; break;
        case VehicleType::FIGHTER:    s << "VehicleType::FIGHTER"; break;
        case VehicleType::_COUNT_:    s << "VehicleType::_COUNT_"; break;
    }
    return s;
}

MyStrategy::MyStrategy() {
    ctx_.vehicleById = &vehicles_;
}

double MyStrategy::getWeatherVisibility(WeatherType w) const {
    switch (w) {
        case WeatherType::CLEAR: return ctx_.game->getClearWeatherVisionFactor();
        case WeatherType::CLOUD: return ctx_.game->getCloudWeatherVisionFactor();
        case WeatherType::RAIN:  return ctx_.game->getRainWeatherVisionFactor();
        default: return 0.;
    }
}

double MyStrategy::getTerrainVisibility(TerrainType t) const {
    switch (t) {
        case TerrainType::FOREST: return ctx_.game->getForestTerrainVisionFactor();
        case TerrainType::PLAIN:  return ctx_.game->getPlainTerrainVisionFactor();
        case TerrainType::SWAMP:  return ctx_.game->getSwampTerrainVisionFactor();
        default: return 0.;
    }
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
    }

    for (const auto &vehicleUpdate : world.getVehicleUpdates()) {
        const auto &vehicleId = vehicleUpdate.getId();

        if (vehicleUpdate.getDurability() == 0) {
            vehicles_.erase(vehicleId);
        } else {
            vehicles_[vehicleId] = Vehicle(vehicles_[vehicleId], vehicleUpdate);
        }
    }
}

bool MyStrategy::executeDelayedMove(Move& move) {
    if (moveQueue_.empty())
        return false;

    if (moveQueue_.begin()->first <= ctx_.world->getTickIndex()) {
        move = moveQueue_.begin()->second;
        moveQueue_.erase(moveQueue_.begin());
    }

    return true;
}

std::pair<double, double> MyStrategy::center() const {
    double x = 0.;
    double y = 0.;
    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId()) {
            x += v.second.getX();
            y += v.second.getY();
            ++cnt;
        }
    }
    if (cnt) {
        x /= cnt;
        y /= cnt;
    }
    return {x, y};
}

std::pair<double, double> MyStrategy::target() const {
    double x = 0.;
    double y = 0.;
    int cnt = 0;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId()) {
            x += v.second.getX();
            y += v.second.getY();
            ++cnt;
        }
    }
    if (cnt) {
        x /= cnt;
        y /= cnt;
    }
    return {x, y};
}

std::pair<double,double> MyStrategy::span() const {
    double x_min = ctx_.world->getWidth();
    double x_max = 0.;
    double y_min = ctx_.world->getHeight();
    double y_max = 0.;
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId()) {
            x_min = std::min(x_min, v.second.getX());
            x_max = std::max(x_max, v.second.getX());
            y_min = std::min(y_min, v.second.getY());
            y_max = std::max(y_max, v.second.getY());
        }
    }
    return {x_max-x_min, y_max-y_min};
}

void MyStrategy::congregate() {
    constexpr double angle = PI/2.;
    const auto c = center();
    const auto s = span();
    Move m;

    // сдвиг в центр
    m.setAction(ActionType::CLEAR_AND_SELECT);
    m.setLeft(0);
    m.setTop(0);
    m.setRight(c.first);
    m.setBottom(c.second);
    queueMove(0, m);
    m.setAction(ActionType::MOVE);
    m.setX(s.first/2);
    m.setY(s.second/2);
    queueMove(0, m);

    m.setAction(ActionType::CLEAR_AND_SELECT);
    m.setLeft(c.first);
    m.setTop(0);
    m.setRight(ctx_.world->getWidth());
    m.setBottom(c.second);
    queueMove(0, m);
    m.setAction(ActionType::MOVE);
    m.setX(-s.first/2);
    m.setY(s.second/2);
    queueMove(0, m);

    m.setAction(ActionType::CLEAR_AND_SELECT);
    m.setLeft(c.first);
    m.setTop(c.second);
    m.setRight(ctx_.world->getWidth());
    m.setBottom(ctx_.world->getHeight());
    queueMove(0, m);
    m.setAction(ActionType::MOVE);
    m.setX(-s.first/2);
    m.setY(-s.second/2);
    queueMove(0, m);

    m.setAction(ActionType::CLEAR_AND_SELECT);
    m.setLeft(0);
    m.setTop(c.second);
    m.setRight(c.first);
    m.setBottom(ctx_.world->getHeight());
    queueMove(0, m);
    m.setAction(ActionType::MOVE);
    m.setX(s.first/2);
    m.setY(-s.second/2);
    queueMove(0, m);

    // вращение
    m.setAction(ActionType::CLEAR_AND_SELECT);
    m.setLeft(0);
    m.setTop(0);
    m.setRight(ctx_.world->getWidth());
    m.setBottom(ctx_.world->getHeight());
    queueMove(120, m);
    m.setAction(ActionType::ROTATE);
    m.setX(c.first);
    m.setY(c.second);
    m.setAngle(angle);
    queueMove(120, m);
}

double MyStrategy::distToEnemy() const {
    double d = 2.*std::max(ctx_.world->getHeight(), ctx_.world->getWidth()) * 2.*std::max(ctx_.world->getHeight(), ctx_.world->getWidth());
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId()) {
            for (const auto &t: vehicles_) {
                if (t.second.getPlayerId() != ctx_.me->getId()) {
                    d = std::min(d, v.second.getSquaredDistanceTo(t.second));
                }
            }
        }
    }
    return d;
}

void MyStrategy::nuke() {
#ifdef MYDEBUG
    if (ctx_.me->getNextNuclearStrikeTickIndex() > 0) {
        std::cout << " strike unit is " << (vehicles_.count(ctx_.me->getNextNuclearStrikeVehicleId())? "alive": "dead");
    }
#endif
    if (ctx_.me->getRemainingNuclearStrikeCooldownTicks() > 0) {
#ifdef MYDEBUG
        std::cout << " ticks until nuke " << ctx_.me->getRemainingNuclearStrikeCooldownTicks();
#endif
        return;
    }

    const auto c = center();
    const auto s = span();
    const auto t = target();

    auto eDir = std::make_pair(t.first - c.first, t.second - c.second);
    const auto eDirL = std::sqrt(eDir.first*eDir.first + eDir.second*eDir.second);
    eDir.first /= eDirL;
    eDir.second /= eDirL;

    const auto quarterSpan = (s.first + s.second)/8.;
    const auto pointA = std::make_pair(c.first + eDir.first*quarterSpan, c.second + eDir.second*quarterSpan);

    // choose strike vehicle near A
    int vehicleId = -1;
    double best_score = std::numeric_limits<double>::max();
    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId())
            continue;

        const double visionCoeff = v.second.isAerial()?
                                   getWeatherVisibility(ctx_.world->getWeatherByCellXY()[(int)v.second.getX()%32][(int)v.second.getY()%32]):
                                   getTerrainVisibility(ctx_.world->getTerrainByCellXY()[(int)v.second.getX()%32][(int)v.second.getY()%32]);
        const double score = v.second.getSquaredDistanceTo(pointA.first, pointA.second) - v.second.getVisionRange()*visionCoeff*v.second.getVisionRange()*visionCoeff;
        if (score < best_score) {
            best_score = score;
            vehicleId = v.first;
        }
    }
    const auto &v = vehicles_[vehicleId];

    // choose strike point
    const double strikeDistance = v.getVisionRange()*0.9;
    std::pair<double, double> pointB = t;
    if (v.getSquaredDistanceTo(t.first, t.second) > strikeDistance*strikeDistance) {
        auto tDir = std::make_pair(t.first - v.getX(), t.second - v.getY());
        const auto tDirL = std::sqrt(tDir.first*tDir.first + tDir.second*tDir.second);
        tDir.first /= tDirL;
        tDir.second /= tDirL;
        pointB.first = v.getX() + tDir.first*strikeDistance;
        pointB.second = v.getY() + tDir.second*strikeDistance;
    }

    // estimate damage in strike zone
    double balanceDamage = 0.;
    for (const auto &v: vehicles_) {
        const auto d = v.second.getDistanceTo(pointB.first, pointB.second);
        if ( d > ctx_.game->getTacticalNuclearStrikeRadius())
            continue;
        const double dmg = ctx_.game->getMaxTacticalNuclearStrikeDamage() * (1 - d/ctx_.game->getTacticalNuclearStrikeRadius());
        if (v.second.getPlayerId() == ctx_.me->getId()) {
            balanceDamage -= dmg;
        } else {
            balanceDamage += dmg;
        }
    }
#ifdef MYDEBUG
    std::cout << " dmg " << balanceDamage;
#endif

    // strike!
    if (balanceDamage > 10.*ctx_.game->getMaxTacticalNuclearStrikeDamage()) {
        Move m;
        m.setAction(ActionType::TACTICAL_NUCLEAR_STRIKE);
        m.setX(pointB.first);
        m.setY(pointB.second);
        m.setVehicleId(vehicleId);
        queueMove(0, m);
#ifdef MYDEBUG
        std::cout << " strike!";
#endif
    }
}


void MyStrategy::move(const Player& me, const World& world, const Game& game) {
    if (world.getTickIndex() == 0) {
        startupFormation();
    }
    return;

    if (world.getTickIndex() == 0) {
        congregate();
    }
    if (world.getTickIndex() == 240) {
        congregate();
    }
    if (world.getTickIndex() == 480) {
        congregate();
    }

    if (world.getTickIndex() >= 720) {
#ifdef MYDEBUG
        std::cout << "tick: " << world.getTickIndex() << " arState: " << antiReconState_;
#endif

        if (antiReconState_) {
            if (--antiReconDelay_ > 0) {
#ifdef MYDEBUG
                std::cout << " antiReconDelay: " << antiReconDelay_ << std::endl;
#endif
            } else {
                detectRecon(false);
                attackRecon();
            }
#ifdef MYDEBUG
            std::cout << std::endl;
#endif
            return;
        }

        nuke();

        if (detectRecon(true))
            return;

        if (needCongregate_) {
            needCongregate_ = false;
            congregate();
        }

        if (world.getTickIndex() % 120 == 0) {
            if (distToEnemy() > 20.) {
                constexpr double dd_max = 100. * 100.;
                const auto c = center();
                const auto t = target();
                double dx = t.first - c.first;
                double dy = t.second - c.second;
                const double dd = dx * dx + dy * dy;
                if (dd > dd_max) {
                    const double k = dd_max / dd;
                    dx *= k;
                    dy *= k;
                }

                Move m;
                m.setAction(ActionType::CLEAR_AND_SELECT);
                m.setLeft(0);
                m.setTop(0);
                m.setRight(ctx_.world->getWidth());
                m.setBottom(ctx_.world->getHeight());
                queueMove(0, m);
                m.setAction(ActionType::MOVE);
                m.setX(dx);
                m.setY(dy);
                m.setMaxSpeed(game.getTankSpeed());
                queueMove(0, m);
#ifdef MYDEBUG
                std::cout << " move " << dx << ' ' << dy;
#endif
            } else {
                congregate();
#ifdef MYDEBUG
                std::cout << " congregate";
#endif
            }
        }
#ifdef MYDEBUG
        std::cout << std::endl;
#endif
    }
}

bool MyStrategy::detectRecon(bool select) {
    std::pair<double, double> c;
    {
        double x = 0.;
        double y = 0.;
        int cnt = 0;
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial()) {
                x += v.second.getX();
                y += v.second.getY();
                ++cnt;
            }
        }
        if (cnt) {
            x /= cnt;
            y /= cnt;
        }
        c.first = x;
        c.second = y;
    }
    const auto t = target();
    const auto s = span();

    const double leftBorder = c.first - s.first/2. - ctx_.game->getFighterVisionRange()*2.;
    const double rightBorder = c.first + s.first/2. + ctx_.game->getFighterVisionRange()*2.;
    const double topBorder = c.second - s.second/2. - ctx_.game->getFighterVisionRange()*2.;
    const double bottomBorder = c.second + s.second/2. + ctx_.game->getFighterVisionRange()*2.;

    enemyRecon_.clear();

    if (t.first > leftBorder && t.first < rightBorder && t.second > topBorder && t.second < bottomBorder) {
        return antiReconState_;
    }

    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() == ctx_.me->getId())
            continue;
        if (v.second.getX() > leftBorder && v.second.getX() < rightBorder && v.second.getY() > topBorder && v.second.getY() < bottomBorder) {
            enemyRecon_.insert(v.first);
        }
    }

    if (!enemyRecon_.empty() && enemyRecon_.size() < 20) {
        antiReconState_ = true;
        antiReconDelay_ = 0;

        if (select) {
            Move m;
            m.setAction(ActionType::CLEAR_AND_SELECT);
            m.setLeft(0);
            m.setTop(0);
            m.setRight(ctx_.world->getWidth());
            m.setBottom((ctx_.world->getHeight()));
            m.setVehicleType(VehicleType::FIGHTER);
            queueMove(0, m);
            m.setAction(ActionType::ADD_TO_SELECTION);
            m.setVehicleType(VehicleType::HELICOPTER);
            queueMove(0, m);
        }
    } else {
        enemyRecon_.clear();
    }
#ifdef MYDEBUG
    std::cout << " detectRecon() found units: " << enemyRecon_.size();
#endif

    return antiReconState_;
}

void MyStrategy::attackRecon() {
    int ev = -1;
    for (const int evt: enemyRecon_) {
        if (vehicles_.count(evt)) {
            ev = evt;
            break;
        }
    }

    std::pair<double, double> ac;
    {
        double x = 0.;
        double y = 0.;
        int cnt = 0;
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() == ctx_.me->getId() && v.second.isAerial()) {
                x += v.second.getX();
                y += v.second.getY();
                ++cnt;
            }
        }
        if (cnt) {
            x /= cnt;
            y /= cnt;
        }
        ac.first = x;
        ac.second = y;
    }

    if (ev < 0) {
        std::pair<double, double> gc;
        {
            double x = 0.;
            double y = 0.;
            int cnt = 0;
            for (const auto &v: vehicles_) {
                if (v.second.getPlayerId() == ctx_.me->getId() && !v.second.isAerial()) {
                    x += v.second.getX();
                    y += v.second.getY();
                    ++cnt;
                }
            }
            if (cnt) {
                x /= cnt;
                y /= cnt;
            }
            gc.first = x;
            gc.second = y;
        }

        if ((gc.first-ac.first)*(gc.first-ac.first) + (gc.second-ac.second)*(gc.second-ac.second) < 512.) {
            antiReconState_ = false;
            needCongregate_ = true;
        }

        Move m;
        m.setAction(ActionType::MOVE);
        m.setX(gc.first - ac.first);
        m.setY(gc.second - ac.second);
        queueMove(0, m);
        antiReconDelay_ = 30;
        return;
    }

#ifdef MYDEBUG
    std::cout << " attackRecon()";
#endif

    Move m;

    m.setAction(ActionType::MOVE);
    m.setX((vehicles_[ev].getX() - ac.first)*1.1);
    m.setY((vehicles_[ev].getY() - ac.second)*1.1);
    queueMove(0, m);
    antiReconDelay_ = 30;
}

void MyStrategy::startupFormation() {
    std::array<VehicleType, 3> grNum{VehicleType::TANK, VehicleType::IFV, VehicleType::ARRV};
    std::map<VehicleType, int> type2idx {
            {VehicleType::TANK, 0},
            {VehicleType::IFV,  1},
            {VehicleType::ARRV, 2}
    };
    int delay = 0;

    std::array<std::pair<double, double>, 3> grPos;
    grPos.fill(std::make_pair(0., 0.));

    for (const auto &v: vehicles_) {
        if (v.second.getPlayerId() != ctx_.me->getId())
            continue;
        switch (v.second.getType()) {
            case VehicleType::TANK:
            case VehicleType::IFV:
            case VehicleType::ARRV:
                break;
            default: continue;
        }
        auto &pos = grPos[type2idx[v.second.getType()]];
        pos.first += v.second.getX()/100.;
        pos.second += v.second.getY()/100.;
    }

    {
        int firstGr = 0;
        double minY = grPos[0].second;
        for (int i: {1, 2}) {
            if (grPos[i].second < minY) {
                minY = grPos[i].second;
                firstGr = i;
            }
        }
        double minX = grPos[firstGr].first;
        for (int i: {(firstGr+1)%3, (firstGr+2)%3}) {
            if (grPos[i].second <= minY && grPos[i].first < minX) {
                minX = grPos[i].first;
                firstGr = i;
            }
        }
        std::swap(grNum[0], grNum[firstGr]);
        std::swap(grPos[0], grPos[firstGr]);

        if (grPos[1].second > grPos[2].second || grPos[1].first < grPos[2].first) {
            std::swap(grNum[2], grNum[1]);
            std::swap(grPos[2], grPos[1]);
        }

        for (int i=0; i<3; ++i) {
            type2idx[grNum[i]] = i;
        }
    }

    for (auto vt: grNum) {
        double left = ctx_.world->getWidth();
        double right = 0.;
        for (const auto &v: vehicles_) {
            if (v.second.getPlayerId() != ctx_.me->getId() || v.second.getType() != vt)
                continue;
            left = std::min(left, v.second.getX());
            right = std::max(right, v.second.getX());
        }
        const double spacing = (right - left)/9.;

        Move m;
        m.setAction(ActionType::CLEAR_AND_SELECT);
        m.setLeft(0);
        m.setTop(0);
        m.setRight(ctx_.world->getWidth());
        m.setBottom(ctx_.world->getHeight());
        m.setVehicleType(vt);
        queueMove(delay, m);

        m.setAction(ActionType::SCALE);
        m.setX(grPos[type2idx[vt]].first);
        m.setY(grPos[type2idx[vt]].second);
        m.setFactor(5./spacing);
        queueMove(delay, m);
    }

    delay += 200;
    {
        Move m;
        m.setAction(ActionType::CLEAR_AND_SELECT);
        m.setLeft(0);
        m.setTop(0);
        m.setRight(ctx_.world->getWidth());
        m.setBottom(ctx_.world->getHeight());
        m.setVehicleType(grNum[0]);
        queueMove(delay, m);

        m.setAction(ActionType::MOVE);
        m.setX(100. - grPos[0].first);
        m.setY(100. - grPos[0].second);
        queueMove(delay, m);
        grPos[0] = {100., 100.};
    }
    {
        Move m;
        m.setAction(ActionType::CLEAR_AND_SELECT);
        m.setLeft(0);
        m.setTop(0);
        m.setRight(ctx_.world->getWidth());
        m.setBottom(ctx_.world->getHeight());
        m.setVehicleType(grNum[1]);
        queueMove(delay, m);

        m.setAction(ActionType::MOVE);
        m.setX(grPos[0].first + 50. - grPos[1].first);
        m.setY(grPos[0].second + 150. - grPos[1].second);
        queueMove(delay, m);
        grPos[1] = {grPos[0].first + 50., grPos[0].second + 150.};
    }
    {
        Move m;
        m.setAction(ActionType::CLEAR_AND_SELECT);
        m.setLeft(0);
        m.setTop(0);
        m.setRight(ctx_.world->getWidth());
        m.setBottom(ctx_.world->getHeight());
        m.setVehicleType(grNum[2]);
        queueMove(delay, m);

        m.setAction(ActionType::MOVE);
        m.setX(grPos[0].first - grPos[2].first);
        m.setY(grPos[1].second + 150. - grPos[2].second);
        queueMove(delay, m);
        grPos[2] = {grPos[0].first, grPos[1].second + 150.};
    }

    delay += 800;
    for (auto vt: grNum) {
        Move m;
        m.setAction(ActionType::CLEAR_AND_SELECT);
        m.setLeft(0);
        m.setTop(0);
        m.setRight(ctx_.world->getWidth());
        m.setBottom(ctx_.world->getHeight());
        m.setVehicleType(vt);
        queueMove(delay, m);

        m.setAction(ActionType::SCALE);
        m.setX(grPos[type2idx[vt]].first);
        m.setY(grPos[type2idx[vt]].second);
        m.setFactor(3.);
        queueMove(delay, m);
    }
}

void MyStrategy::queueMove(int delay, const Move &m){
    moveQueue_.emplace(ctx_.world->getTickIndex() + delay, m);
}
