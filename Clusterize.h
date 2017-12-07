#pragma once

#ifndef MYSTRATEGY_CLUSTERIZE_H
#define MYSTRATEGY_CLUSTERIZE_H

#include <unordered_set>
#include <deque>
#include "Context.h"

namespace Clusterize {
    struct Cluster {
        std::unordered_set<VId> set;
        model::VehicleType type;
    };
    using ClusterList = std::deque<Cluster>;
    struct ClusterLists {
        ClusterList myClusters, enemyClusters;
    };

    ClusterLists* clusterize(const Context &ctx, double dist);
}


#endif //MYSTRATEGY_CLUSTERIZE_H
