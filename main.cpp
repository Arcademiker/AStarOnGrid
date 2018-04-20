//
// Created by Thomas on 19.04.18.
//

/* Approach: A Star (A*) algorithm with some custom changes. On a very small grid (like in the examples)
 * Djikstra for path finding is probably faster than the simplest version of A*, because A* has to calculate its
 * node elapsing heuristic with every expanding step, but since Djikstra
 * //todo ausführen
 * overhead vs datausage vs speed oder so readability
 *
 *
 * */

#include <iostream>
#include <queue>
#include <unordered_map>
#include "ExplorationAgenda.h"



int FindPath(int nStartX, int nStartY,
             int nTargetX, int nTargetY,
             const unsigned char* pMap, int nMapWidth, int nMapHeight,
             int* pOutBuffer, int nOutBufferSize);

int HScore(int nNode, int nMapWidth, int nTargetX, int nTargetY);

class ExplorationAgenda;

int main() {
    // user input:
    int nStartX = 1;
    int nStartY = 0;
    int nTargetX = 5;
    int nTargetY = 7;
    unsigned char pMap[] = {1, 1, 1, 1, 1, 1,
                            1, 1, 1, 0, 0, 1,
                            1, 1, 1, 1, 0, 1,
                            0, 1, 0, 1, 1, 0,
                            1, 1, 1, 1, 0, 1,
                            1, 0, 1, 0, 0, 1,
                            1, 0, 1, 1, 1, 1,
                            1, 1, 1, 1, 0, 1,};
    int nMapWidth = 6;
    int nMapHeight = 8;
    int nOutBufferSize = 11;
    int pOutBuffer[nOutBufferSize];

    // user input check for matching nMapWidth, nMapHeight and nOutBufferSize sizes,
    if(sizeof(pMap)/sizeof(*pMap) != nMapWidth*nMapHeight) {
        std::cerr << "pMap width and height sizes not matching with given pMap array size!" << std::endl;
        return 0;
    }

    //start of the algorithm:
    int nPathLength = FindPath(nStartX, nStartY, nTargetX, nTargetY, pMap, nMapWidth, nMapHeight, pOutBuffer, nOutBufferSize);
    std::cout << "length of the path: "<< nPathLength << std::endl;

    std::cout << "path:" << "\t" <<"x" << "\t" <<"y" << std::endl;
    for(int i=0; i<nPathLength; i++) {
        std::cout<< pOutBuffer[i] << "\t\t" << pOutBuffer[i]%nMapWidth << "\t" << pOutBuffer[i]/nMapWidth << std::endl;
    }
    return 0;
}

//why not multimap? todo! key=fscore


/* A* algorithm pseudo code:
 *
 * load next agenda node
 * explore u d l r (up down left right)
 *   check u d l r for not out of border + not wall (array)
 *   + check u d l r for not already visited
 *       put GScore+1 in visit list
 *       calc HScore = Manhattan distance current node to target
 *       calc FScore = HScore + GScore+1
 *       put FScore as Key in Priority Queue
 *  repeat (break if target node reached) */
int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* pMap,
             const int nMapWidth, const int nMapHeight, int* pOutBuffer, const int nOutBufferSize) {

    bool bPath = false;
    int nPathLength = 0;
    //visit start node
    const int nMapSize = nMapHeight*nMapWidth;

    int nX = nStartX;
    int nY = nStartY;

    int nNode = nX+nY*nMapWidth;
    auto explorationAgenda = new ExplorationAgenda();
    //explore start node (which means calculating f score for this node) todo f(n) = g(n)+h(n)
    explorationAgenda->Add(HScore(nNode,nMapWidth,nTargetX,nTargetY),nNode); //todo! f(n)=h(n)+g(n) nodes grid erklärung

    auto pGScore = new std::unordered_map<int,int>();
    std::pair<std::unordered_map<int,int>::iterator, bool> itVisited; //todo fix confusing naming
    pGScore->insert(std::make_pair(nNode,0));

    //parent Tree to find the shortest path from target back to the start (key = node, value = parent)
    auto pParentNode = new std::unordered_map<int,int>();
    //define starting point of the parent tree
    pParentNode->insert({nNode,nNode});


    int nUp;
    int nDown;
    int nLeft;
    int nRight;
    int nTarget=nTargetX+nTargetY*nMapWidth;
    while(nOutBufferSize>nPathLength && !explorationAgenda->IsEmpty()) {
        nNode = explorationAgenda->VisitTop();
        nPathLength = (*pGScore)[nNode];
        if (nTarget==nNode) {

            for(int i=(*pGScore)[nNode]; i>0; i--) {
                pOutBuffer[i-1] = nNode;
                nNode = (*pParentNode)[nNode];
            }
            bPath = true;
            break;
        }
        //nX = nNode%nMapWidth;
        //nY = nNode/nMapWidth;
        //frontier
        nUp = nNode-nMapWidth;
        nDown = nNode+nMapWidth;
        nLeft = nNode-1;
        nRight = nNode+1;
        ///try to visit up. Check outer rim of the map && Check for Walls
        if( nUp >= 0 && static_cast<bool>(pMap[nUp]) ) {
            itVisited = pGScore->insert(std::make_pair(nUp,0));
            ///don't visit the same node twice
            if(itVisited.second) {
                itVisited.first->second = (*pGScore)[nNode]+1;
                pParentNode->insert(std::make_pair(nUp,nNode));
                ///visit up and write FScore (f(n)=h(n)+g(n)) as key into the priority queue
                explorationAgenda->Add(HScore(nUp, nMapWidth, nTargetX, nTargetY) + itVisited.first->second, nUp );
            }
        }

        if( nDown < nMapSize && static_cast<bool>(pMap[nDown]) ) {
            itVisited = pGScore->insert(std::make_pair(nDown, 0));
            if (itVisited.second) {
                itVisited.first->second = (*pGScore)[nNode] + 1;
                pParentNode->insert(std::make_pair(nDown,nNode));
                explorationAgenda->Add(HScore(nDown, nMapWidth, nTargetX, nTargetY) + itVisited.first->second, nDown);
            }
        }

        if( nLeft >= nNode/nMapWidth*nMapWidth && static_cast<bool>(pMap[nLeft]) ) {
            itVisited = pGScore->insert(std::make_pair(nLeft, 0));
            if (itVisited.second) {
                itVisited.first->second = (*pGScore)[nNode] + 1;
                pParentNode->insert(std::make_pair(nLeft,nNode));
                explorationAgenda->Add(HScore(nLeft, nMapWidth, nTargetX, nTargetY) + itVisited.first->second, nLeft);
            }
        }

        if( nRight < nNode/nMapWidth*nMapWidth+nMapWidth && static_cast<bool>(pMap[nRight]) ) {
            itVisited = pGScore->insert(std::make_pair(nRight, 0));
            if (itVisited.second) {
                itVisited.first->second = (*pGScore)[nNode] + 1;
                pParentNode->insert(std::make_pair(nRight,nNode));
                explorationAgenda->Add(HScore(nRight, nMapWidth, nTargetX, nTargetY) + itVisited.first->second, nRight);
            }
        }
    }

    if(nMapWidth<10 && nMapHeight<10) {
        std::cout << "traversal tree ( g(n) values of a* ):" << std::endl;
        for (int i = 0; i < nMapSize; i++) {
            if (i % nMapWidth == 0) { std::cout << std::endl; }
            if (pGScore->count(i)) {
                std::cout << (*pGScore)[i] << " ";
            } else {
                std::cout << "X ";
            }
        }
        std::cout << std::endl << std::endl;
    }


    delete explorationAgenda;
    delete pParentNode;
    delete pGScore;
    if(!bPath) {
        nPathLength = -1;
        if(nOutBufferSize>nPathLength) {
            std::cout << std::endl << "Pathfinding was aborted, because there is no way to the target within the distance of "
                      << nOutBufferSize << " (nOutBufferSize)" << std::endl;
        }
    }
    return nPathLength;
}

// A* heuristic: Manhattan distance between a point (nX,nY) on the grid and the target (nTargetX, nTargetY)
int HScore(int nNode, int nMapWidth, int nTargetX, int nTargetY) {
    return std::abs(nNode%nMapWidth-nTargetX)+std::abs(nNode/nMapWidth-nTargetY);
}