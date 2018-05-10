//
// Created by Thomas on 19.04.18.
//

#ifndef ASTARONGRID_EXPLORATIONAGENDA_H
#define ASTARONGRID_EXPLORATIONAGENDA_H

#include <queue>

class ExplorationAgenda {
private:
    // define Key Comparator for std::priority_queue:
    struct CompareByFirst {
        constexpr bool operator()(std::pair<int, int> const & a, std::pair<int, int> const & b) const noexcept {
            return a.first > b.first || (a.first == b.first && a.second < b.second); //smaller key means higher priority
        }
    };
    // define Key (nK), Value (nV) Priority Queue (PriorityQ) ordered by Key (minimal Key on top):
    typedef std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>,CompareByFirst> minK_PriorityQ;
    minK_PriorityQ* qPriorityQ;  //q prefix indicates queue data type
public:
    // member functions for simpler std::priority_queue handling:
    ExplorationAgenda();
    ~ExplorationAgenda();
    void Add(int nK, int nV);
    int VisitTop();
    bool IsEmpty();
};


#endif //ASTARONGRID_EXPLORATIONAGENDA_H
