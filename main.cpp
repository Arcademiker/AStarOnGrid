//
// Created by Thomas on 19.04.18.
//

/* Approach: A Star (A*) algorithm with some custom changes. On a very small grid (like in the examples)
 * the Dijkstra Algorithm for path finding is probably faster than the simplest version of A*, because A* has to calculate its
 * node elapsing heuristic with every expanding step, but since Dijkstra spreads uninformed into all directions A* is on bigger maps faster.
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
    int nOutBufferSize = 15;
    int pOutBuffer[nOutBufferSize];

    // user input check for matching nMapWidth, nMapHeight and nOutBufferSize sizes,
    if(sizeof(pMap)/sizeof(*pMap) != nMapWidth*nMapHeight) {
        std::cerr << "pMap width and height sizes not matching with given pMap array size!" << std::endl;
        return 0;
    }

    //start of the algorithm:
    int nPathLength = FindPath(nStartX, nStartY, nTargetX, nTargetY, pMap, nMapWidth, nMapHeight, pOutBuffer, nOutBufferSize);

    //print of the array positions and (x,y) coordinates on the shortest path
    std::cout << "length of the path: "<< nPathLength << std::endl;
    std::cout << "path:" << "\t" <<"x" << "\t" <<"y" << std::endl;
    for(int i=0; i<nPathLength; i++) {
        std::cout<< pOutBuffer[i] << "\t\t" << pOutBuffer[i]%nMapWidth << "\t" << pOutBuffer[i]/nMapWidth << std::endl;
    }
    return 0;
}

/* A* algorithm pseudo code:
 *
 * load next agenda node
 * explore u d l r (up down left right)
 *   check u d l r for not out of border + not wall (array)
 *     check u d l r for not already visited
 *       put GScore+1 in visit list
 *       remember parent node
 *       calc HScore = Manhattan distance current node to target
 *       calc FScore = HScore + GScore+1
 *       put FScore as Key in Priority Queue
 *  repeat (break if target node is reached)
 *       follow the parent nodes back to the start
 *  return path length or -1 if no such path exists*/
int FindPath(const int nStartX, const int nStartY, const int nTargetX, const int nTargetY, const unsigned char* pMap,
             const int nMapWidth, const int nMapHeight, int* pOutBuffer, const int nOutBufferSize) {

    //exist a path variable
    bool bPath = false;

    int nPathLength = 0;
    //visit start node
    const int nMapSize = nMapHeight*nMapWidth;

    //translate array position on map into x and y values. nNode is a vertex on the grid with four neighbours up down left right
    int nNode = nStartX+nStartY*nMapWidth;

    //ExplorationAgenda is a Priority Queue with key value pairs. The pair with the smallest key is on top.
    //key is the value for the f(n) score of A*.
    //why priority queue, why not multimap?
    // the std-priority_queue is faster than the std-multimap, because:
    //  The typical underlying implementation of a multimap is a balanced red/black tree.
    //  Repeated element removals from one of the extreme ends of a multimap has a good chance of skewing the tree,
    //  requiring frequent rebalancing of the entire tree. This is going to be an expensive operation.
    auto explorationAgenda = new ExplorationAgenda();
    //explore start node (which means calculating f(n) score for this node): f(n) = h(n)+g(n)
    //g(n) := dijsktra distance from start to current node. for the start node = 0
    //h(n) := manhattan distance form target to current node ignoring all possible walls on the way. h(n) the heuristic for this A*
    explorationAgenda->Add(HScore(nNode,nMapWidth,nTargetX,nTargetY),nNode);

    //pGScore = g(n)
    //why hash map, why not a vector?
    // a vector has a slightly faster access to its elements in many cases, however maps to do pathfinding on can be very big,
    // especially in modern games. The big advantage of unordered maps (hashmaps) is, they just keep track of the visited elements
    // If A* has to operate just in a small corner of a big map the whole map vector for the g(n) score has to be set to 0 at first
    // and would consume storage the whole time the algorithm is in use.
    auto pGScore = new std::unordered_map<int,int>();
    //iterator to work with unordered map (hash map)
    std::pair<std::unordered_map<int,int>::iterator, bool> itVisited;
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

    //main loop of the A* algorithm
    while(nOutBufferSize>nPathLength && !explorationAgenda->IsEmpty()) {

        //set the current node (nNode) to the node with the highest exploration priority (lowest f(n) Score)
        nNode = explorationAgenda->VisitTop();
        nPathLength = (*pGScore)[nNode];

        //abort if target node is reached
        if (nTarget==nNode) {

            //find the shortest way back from the target to the start
            for(int i=nPathLength; i>0; i--) {
                pOutBuffer[i-1] = nNode;
                nNode = (*pParentNode)[nNode];
            }
            bPath = true;
            break;
        }

        //calculate the adjacent nodes of nNode (current Node)
        nUp = nNode-nMapWidth;
        nDown = nNode+nMapWidth;
        nLeft = nNode-1;
        nRight = nNode+1;

        //try to visit up. Check outer rim of the map && Check for Walls
        if( nUp >= 0 && static_cast<bool>(pMap[nUp]) ) {
            //if insertion is successful "itVisited.second" in the if-statement becomes true
            itVisited = pGScore->insert(std::make_pair(nUp,0));
            //don't visit the same node twice
            if(itVisited.second) {
                //set the g(n) Score (without doing a second hashmap traversal) by raising the old g(n) by 1
                itVisited.first->second = (*pGScore)[nNode]+1;
                //keep track of the last visited node to find the shortest way back
                pParentNode->insert(std::make_pair(nUp,nNode));
                //visit up and write FScore (f(n)=h(n)+g(n)) as key into the priority queue
                explorationAgenda->Add(HScore(nUp, nMapWidth, nTargetX, nTargetY) + itVisited.first->second, nUp );
            }
        }

        //try to visit adjacent node down... (redundant code but easy readable and efficient)
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

    //print of the map
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

    //return -1 when there is no way or the nOutBufferSize is too short
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