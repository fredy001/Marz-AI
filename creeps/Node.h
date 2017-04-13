/*------------------------------------------------------------------------------
* Header:       Node.h
*
* Date:         February 1, 2017
*
* Revisions:
* Edited By :   Yiaoping Shu- Style guide
*
* Designer:     Fred Yang
*
* Author:       Fred Yang
*
* Notes:
* Used for the A* algorithm in navigating the map.
------------------------------------------------------------------------------*/
#ifndef NODE_H
#define NODE_H
#include <cmath>
#include <queue>
#include "../map/Map.h"
#include "../log/log.h"

// 8 possible directions
static constexpr int DIR_CAP = 8;

// horizontal/vertical & diagonal cost
static constexpr int BASE_COST   = 10;
static constexpr int EXTEND_COST = 14;

static int closedNodes[M_HEIGHT][M_WIDTH]; // array of closed nodes (evaluated)
static int openNodes[M_HEIGHT][M_WIDTH];   // array of open nodes (to be evaluated)
static int dirMap[M_HEIGHT][M_WIDTH];      // array of directions
static std::array<std::array<bool, M_WIDTH>, M_HEIGHT> gameMap; // boolean map of obstacles

/**
 * 8 possible movements
 * 0 - right, 1 - right down, 2 - down, 3 - left down
 * 4 - left, 5 - left up, 6 - up, 7 - right up
 */
static constexpr int MX[DIR_CAP]={1, 1, 0, -1, -1, -1, 0, 1};
static constexpr int MY[DIR_CAP]={0, 1, 1, 1, 0, -1, -1, -1};

class Node {
public:
    explicit Node(const int xPos = 0, const int yPos = 0, const int lv = 0,
            const int pri = 0) : xPos(xPos), yPos(yPos), lv(lv), pri(pri) {}

    virtual ~Node() = default;

    // X coordinate of current node
    int getXPos() const {
        return xPos;
    }

    // Y coordinate of current node
    int getYPos() const {
        return yPos;
    }

    // Get distance travelled so far
    int getLevel() const {
        return lv;
    }

    // Get priority of current node
    int getPriority() const {
        return pri;
    }

    // current level plus remaining cost
    void updatePriority(const int xDest, const int yDest) {
         pri = lv + estimate(xDest, yDest) * BASE_COST;
    }

    // calculate next level based on direction
    void nextLevel(const int dir) {
         lv += (dir % 2 ==0 ? BASE_COST : EXTEND_COST);
    }

    // calculate cost per the remaining distance to the destination
    const int estimate(const int xDest, const int yDest) const {
        const int xDist = xDest - xPos;
        const int yDist = yDest - yPos;

        // Euclidian Distance
        return static_cast<int>(sqrt(xDist * xDist + yDist * yDist));

    }

private:
    // current position
    int xPos;
    int yPos;

    // level = total distance already travelled to reach the node
    int lv;

    // priority = level+remaining distance estimated
    // smaller one with higher priority
    int pri;
};

// determine priority in priority queue
inline bool operator<(const Node& node1, const Node& node2) {
    return node1.getPriority() > node2.getPriority();
}

#endif
