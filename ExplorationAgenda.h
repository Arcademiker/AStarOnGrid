//
// Created by Thomas on 19.04.18.
//

#pragma once

#include <queue>

class ExplorationAgenda {
private:
    // define Key (nK), Value (nV) Priority Queue (PriorityQ) ordered by Key (minimal Key on top):
    typedef std::priority_queue<std::pair<int,int>,std::vector<std::pair<int,int>>,std::greater<std::pair<int,int>>> minK_PriorityQ;
    minK_PriorityQ* qPriorityQ;  //q prefix indicates queue data type
public:
    // member functions for simpler std::priority_queue handling:
    ExplorationAgenda();
    ~ExplorationAgenda();
    void Add(int nK, int nV);
    int VisitTop();
    bool IsEmpty();
};


