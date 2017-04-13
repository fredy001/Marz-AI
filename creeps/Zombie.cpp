/*------------------------------------------------------------------------------
* Source: Zombie.cpp
*
* Functions:
*       void zAttack();
*       void onCollision();
*       void generateMove();
*       bool isMoving() const;
*       int detectObj() const;
*       ZombieDirection getMoveDir();
*       ZombieDirection getMonteDir();
*       void validateBlockMatrix();
*       void collidingProjectile(int damage);
*       string generatePath(const Point& start);
*       string generatePath(const Point& start, const Point& dest);
*       void move(float moveX, float moveY, CollisionHandler& ch);
*
* Date: February 12, 2017
*
* Revisions: April 8, 2017
* Edited By : Yiaoping Shu- Style guide
*
* Designer:    GM Team, AI Team
*
* Programmer:  AI Team
*
* Notes:
* This class wraps A* / Monte Carlo algorithms and other basic methods for Zombie.
*
------------------------------------------------------------------------------*/
#include <math.h>
#include <random>
#include <cassert>
#include <utility>
#include "Node.h"
#include "Zombie.h"
#include "../game/GameManager.h"
#include "../log/log.h"
#include "../map/Map.h"

using namespace std;

/**
 * Date:    February 14, 2017
 *
 * Author:  AI Team, GM Team
 *
 * Function Interface: Zombie()
 * Parameters:
 *      const int32_t id: Zombie's ID
 *      const SDL_Rect& dest: Zombie's size
 *      const SDL_Rect& movementSize: movement range (for collision checks)
 *      const SDL_Rect& projectileSize: weapon size
 *      const SDL_Rect& damageSize: damage range
 *      const int health: health points
 *      const ZombieState state: state. 0- idle, 1- move, 2- attack, 3- die
 *      const int step: steps that the zombie has moved
 *      const ZombieDirection dir: moving direction
 *      const int frame: used to make sure the zombie doesn't move through the path too quickly/slowly
 *
 * Note:
 * Zombie object Constructor, creates a zombie object based on the parameters specified.
 */
Zombie::Zombie(const int32_t id, const SDL_Rect& dest, const SDL_Rect& movementSize, const SDL_Rect& projectileSize,
        const SDL_Rect& damageSize, const int health, const ZombieState state, const int step,
        const ZombieDirection dir, const int frame) : Entity(id, dest, movementSize, projectileSize,
        damageSize), Movable(id, dest, movementSize, projectileSize, damageSize, ZOMBIE_VELOCITY),
        health(health), state(state), step(step), dir(dir), frame(frame) {
    logv("Create Zombie\n");
    inventory.initZombie();
}

/**
 * Date:    February 14, 2017
 *
 * Author:  AI Team, GM Team
 *
 * Function Interface: ~Zombie()
 *
 * Note:
 * Zombie object destructor.
 */
Zombie::~Zombie() {
    logv("Destroy Zombie\n");
}

/**
 * Date:    February 14, 2017
 *
 * Author:  Fred Yang
 *
 * Function Interface: ZombieDirection getMoveDir()
 * Returns: returns zombie moving direction
 *
 * Note:
 * Get zombie's moving direction by calling A star / Monte Carlo algos.
 * A Star time complexity: O(logN).
 * Monte Carlo time complexity: O(1).
 */
ZombieDirection Zombie::getMoveDir() {
    if (frame > 0) {
        return dir;
    }

    /* 
     * calculate move direction using Monte Carlo method if the
     * zombie already arrived at the base zone. Save cpu time
     */
    if (targeted) {
        return getMonteDir();
    }
    
    // A* algo
    string pth = generatePath(Point(getX(),getY()));

    return static_cast<ZombieDirection>(pth.length() > 0 ? stoi(pth.substr(0,1)) : -1);
}

/**
 * Date:    March 20, 2017
 *
 * Author:  Fred Yang
 *
 * Function Interface: ZombieDirection getMonteDir()
 * Returns: returns zombie moving direction
 *
 * Note:
 * Get zombie's moving direction by calling Monte Carlo algo.
 * Monte Carlo time complexity: O(1).
 */
ZombieDirection Zombie::getMonteDir() {
    // how many choices we have
    int best[2] = {0};
    int bestCount = 0;
    int distance = INT_MAX;
    
    // current row & column
    const int row = getY() / T_SIZE;
    const int col = getX() / T_SIZE;
    
    // current moving direction
    const int d = static_cast<int>(dir);
    
    // backward direction - avoid blocking
    const int backDir = (d > -1 ? (d + DIR_CAP / 2) % DIR_CAP : d);
    
    for (int i = 0; i < DIR_CAP; i++){
        int tempCol = col + MX[i];
        int tempRow = row + MY[i];

        int zx = tempCol * T_SIZE + T_SIZE / 2;
        int zy = tempRow * T_SIZE + T_SIZE / 2;
        
        //calculates Monte Carlo distance
        int mcDistance = abs(zx - M_WIDTH / 2) + abs(zy - M_HEIGHT / 2);
        
        if (mcDistance < distance) {
            best[0] = i;
            distance = mcDistance;
            bestCount = 1;
        } else if (mcDistance == distance){
            best[1] = i;
            bestCount = 2;
        }
    }
    
    if (bestCount == 1) {
        return static_cast<ZombieDirection>(best[0]);
    } else if (bestCount == 2) {
        if (backDir == -1 || (best[0] != backDir && best[1] != backDir)) {
            return static_cast<ZombieDirection>(best[rand() % 2]);
        } else{
            return static_cast<ZombieDirection>(best[(best[0] == backDir ? 1 : 0)]);
        }
    }
    
    return static_cast<ZombieDirection>(backDir);
}

/**
 * Date:    February 12, 2017
 *
 * Author:  GM Team
 *
 * Function Interface: void onCollision()
 * Returns: void
 *
 * Note:
 * Do nothing for now.
 */
void Zombie::onCollision() {
    // Do nothing for now
}

/**
 * Date:    February 12, 2017
 *
 * Author:  GM Team
 *
 * Function Interface: collidingProjectile(int damage)
 * Paramters
 *      int damage: the damage value that a projectile causes
 *
 * Returns: void
 *
 * Note:
 * Zombie's HP declines when collided with a projectile.
 */
void Zombie::collidingProjectile(int damage) {
    health -= damage;
    if (health <= 0) {
        GameManager::instance()->deleteZombie(getId());
    }
}

/**
 * Date:    March 7, 2017
 *
 * Author:  Robert Arendac
 *
 * Function Interface: bool isMoving() const
 *
 * Returns: true if the zombie is moving; false otherwise
 *
 * Note:
 * Returns if the zombie is moving.
 */
bool Zombie::isMoving() const {
    return (state == ZombieState::ZOMBIE_MOVE);
}

/**
 * Date:    March 28, 2017
 *
 * Author:  Robert Arendac, Fred Yang
 *
 * Function Interface:  int detectObj() const
 * 
 * Returns: 0- nothing, 1- zombie, 2- wall, 3- marine,
 *          4- turret, 5- barricade, 6- other objects(base tower)
 *
 * Note:
 * Zombie detects objects in vicinity.
 * In theory, zombies will only have a movement collision with a target
 * as their pathfinding should walk around obstacles.
 */
int Zombie::detectObj() const {
    int objTypeId = 0;
    auto ch = GameManager::instance()->getCollisionHandler();

    if (ch.detectMovementCollision(ch.getQuadTreeEntities(ch.quadtreeZombie, this), this)) {
        objTypeId = 1;
    } else if (ch.detectMovementCollision(ch.getQuadTreeEntities(ch.quadtreeWall, this), this)) {
        objTypeId = 2;
    } else if (ch.detectMovementCollision(ch.getQuadTreeEntities(ch.quadtreeMarine, this), this)) {
        objTypeId = 3;
    } else if (ch.detectMovementCollision(ch.getQuadTreeEntities(ch.quadtreeTurret, this), this)) {
        objTypeId = 4;
    } else if (ch.detectMovementCollision(ch.getQuadTreeEntities(ch.quadtreeBarricade,this),this)) {
        objTypeId = 5;
    } else if (ch.detectMovementCollision(ch.getQuadTreeEntities(ch.quadtreeObj,this),this)) {
        objTypeId = 6;
    }

    return objTypeId;
}

/**
 * Date:    March 15, 2017
 *
 * Author:  Fred Yang, Robert Arendac
 *
 * Function Interface: void move(float moveX, float moveY, CollisionHandler& ch)
 * Paramters
 *      float moveX: move span along x
 *      flaot moveY: move span along y
 *      CollisionHandler& ch: references to CollisionHanlder
 *
 * Returns: void
 *
 * Note:
 * overriden move method, preventing zombies from blocking.
 */
void Zombie::move(float moveX, float moveY, CollisionHandler& ch) {
    // new moving direction to be calulated
    ZombieDirection newDir = ZombieDirection::DIR_INVALID;
    
    // backup moving direction retrieved from the path
    ZombieDirection nextDir = ZombieDirection::DIR_INVALID;

    const float oldX = getX();
    const float oldY = getY();

    // Move the Movable left or right
    setX(getX() + moveX);

    if (detectObj() > 0) {
        setX(getX() - moveX);
    }

    // Move the Movable up or down
    setY(getY() + moveY);

    if (detectObj() > 0) {
        setY(getY() - moveY);
    }

    const float curX = getX();
    const float curY = getY();
    const float distBlock = sqrt((curX - oldX)*(curX - oldX) 
        + (curY - oldY)*(curY - oldY));
    const float distBase = sqrt((curX - M_WIDTH * T_SIZE / 2)*(curX - M_WIDTH * T_SIZE / 2) 
        + (curY - M_HEIGHT * T_SIZE / 2)*(curY - M_HEIGHT * T_SIZE / 2));
    
    // check if the zombie moves into the base zone
    if (distBase <= BASE_ZONE * T_SIZE) {
        targeted = true;
    } else {
        targeted = false;
    }

    // zombie blocked
    if (distBlock < BLOCK_THRESHOLD) {
        string pth = getPath();
        size_t found = pth.find_first_not_of('0' + static_cast<int>(dir));

        if (found != string::npos) {
            nextDir = static_cast<ZombieDirection>(stoi(pth.substr(found, 1)));
        }

        // If blocked, searching for better direction
        switch (dir) {
            case ZombieDirection::DIR_R:
                newDir = (nextDir == ZombieDirection::DIR_RD ?
                          ZombieDirection::DIR_D : ZombieDirection::DIR_U);
                break;
            case ZombieDirection::DIR_RD:
                newDir = (nextDir == ZombieDirection::DIR_D ?
                          ZombieDirection::DIR_LD : ZombieDirection::DIR_RU);
                break;
            case ZombieDirection::DIR_D:
                newDir = (nextDir == ZombieDirection::DIR_LD ?
                          ZombieDirection::DIR_L : ZombieDirection::DIR_R);
                break;
            case ZombieDirection::DIR_LD:
                newDir = (nextDir == ZombieDirection::DIR_L ?
                          ZombieDirection::DIR_LU : ZombieDirection::DIR_RD);
                break;
            case ZombieDirection::DIR_L:
                newDir = (nextDir == ZombieDirection::DIR_LU ?
                          ZombieDirection::DIR_U : ZombieDirection::DIR_D);
                break;
            case ZombieDirection::DIR_LU:
                newDir = (nextDir == ZombieDirection::DIR_U ?
                          ZombieDirection::DIR_RU : ZombieDirection::DIR_LD);
                break;
            case ZombieDirection::DIR_U:
                newDir = (nextDir == ZombieDirection::DIR_RU ?
                          ZombieDirection::DIR_R : ZombieDirection::DIR_L);
                break;
            case ZombieDirection::DIR_RU:
                newDir = (nextDir == ZombieDirection::DIR_R ?
                          ZombieDirection::DIR_RD : ZombieDirection::DIR_LU);
                break;
            case ZombieDirection::DIR_INVALID:
                break;
        }

        // set current direction
        setCurDir(newDir);
    }
}

/**
 * Date:    March 28, 2017
 *
 * Author:  Fred Yang
 *
 * Function Interface: void generateMove()
 *
 * Returns: void
 *
 * Note:
 * Get the direction of the zombie and take a step in the appropriate direction.
 */
void Zombie::generateMove() {
     // Direction zombie is moving
    const ZombieDirection direction = getMoveDir();

    // detect surroundings
    const int collisionObjId = detectObj();

    // path is empty, prepared to switch state to IDLE
    if (direction == ZombieDirection::DIR_INVALID) {
        if (frame > 0) {
            --frame;
        }

        setState(ZombieState::ZOMBIE_IDLE);

        return;
    }

    // preferrable objects appear in vicinity, prepared to attack
    if (collisionObjId > 0) {
        if (frame > 0) {
            --frame;
        }

        if (collisionObjId > 2) { // base, marine, turret, or barricade
            setState(ZombieState::ZOMBIE_ATTACK);
        }

        return;
    }

    // Each case will set direction and angle based on the next step in the path
    switch (direction) {
        case ZombieDirection::DIR_R:
            setDX(ZOMBIE_VELOCITY);
            setDY(0);
            setAngle(static_cast<double>(ZombieAngles::EAST));
            break;
        case ZombieDirection::DIR_RD:
            setDX(ZOMBIE_VELOCITY);
            setDY(ZOMBIE_VELOCITY);
            setAngle(static_cast<double>(ZombieAngles::SOUTHEAST));
            break;
        case ZombieDirection::DIR_D:
            setDX(0);
            setDY(ZOMBIE_VELOCITY);
            setAngle(static_cast<double>(ZombieAngles::SOUTH));
            break;
        case ZombieDirection::DIR_LD:
            setDX(-ZOMBIE_VELOCITY);
            setDY(ZOMBIE_VELOCITY);
            setAngle(static_cast<double>(ZombieAngles::SOUTHWEST));
            break;
        case ZombieDirection::DIR_L:
            setDX(-ZOMBIE_VELOCITY);
            setDY(0);
            setAngle(static_cast<double>(ZombieAngles::WEST));
            break;
        case ZombieDirection::DIR_LU:
            setDX(-ZOMBIE_VELOCITY);
            setDY(-ZOMBIE_VELOCITY);
            setAngle(static_cast<double>(ZombieAngles::NORTHWEST));
            break;
        case ZombieDirection::DIR_U:
            setDX(0);
            setDY(-ZOMBIE_VELOCITY);
            setAngle(static_cast<double>(ZombieAngles::NORTH));
            break;
        case ZombieDirection::DIR_RU:
            setDX(ZOMBIE_VELOCITY);
            setDY(-ZOMBIE_VELOCITY);
            setAngle(static_cast<double>(ZombieAngles::NORTHEAST));
            break;
        case ZombieDirection::DIR_INVALID:  // Shouldn't ever happens, gets rid of warning
            break;
    }
    
    zAttack();
    
    /* 
     * Frames are used to make sure the zombie doesn't move through
     * the path too quickly/slowly
     */
    if (frame > 0) {
        --frame;
    } else {
        setCurFrame(ZOMBIE_FRAMES);
        ++step;
    }

    setCurDir(direction);
    setState(ZombieState::ZOMBIE_MOVE);
}

/**
 * Date:    March 15, 2017
 *
 * Author:  Fred Yang
 *
 * Function Interface: string generatePath(const Point& start)
 *
 * Returns: a string of direction digits
 *
 * Note:
 * Overloaded generatePath(const Point& start, const Point& dest) method.
 */
string Zombie::generatePath(const Point& start) {
    return generatePath(start, Point(M_WIDTH * T_SIZE / 2, M_HEIGHT * T_SIZE / 2));
}

/**
 * Date:    March 15, 2017
 *
 * Author:  Fred Yang
 *
 * Function Interface: string generatePath(const Point& start, const Point& dest)
 *
 * Returns: a string of direction digits
 *
 * Note:
 * A star algo generates a string of direction digits.
 * A Star time complexity: O(logN).
 */
string Zombie::generatePath(const Point& start, const Point& dest) {
    // temp index
    int i, j;

    // priority queue index
    int index = 0;

    // row & column index
    int curRow, curCol;
    int newRow, newCol;

    // path to be generated
    string path;

    // priority queue
    static array<priority_queue<Node>, 2> pq;
   
    // reset the node maps
    memset(closedNodes, 0, sizeof(int) * M_WIDTH * M_HEIGHT);
    memset(openNodes, 0, sizeof(int) * M_WIDTH * M_HEIGHT);
    memset(dirMap, 0, sizeof(int) * M_WIDTH * M_HEIGHT);

    // boolean map of obstacles
    validateBlockMatrix();
    
    // calculate start & end nodes
    const int xNodeStart = static_cast<int>(start.second) / T_SIZE;
    const int yNodeStart = static_cast<int>(start.first) / T_SIZE;
    const int xNodeDest = static_cast<int>(dest.second) / T_SIZE - 1;
    const int yNodeDest = static_cast<int>(dest.first) / T_SIZE - 1;

    // create the start node and push into open list
    Node curNode(xNodeStart, yNodeStart);
    curNode.updatePriority(xNodeDest, yNodeDest);
    pq[index].push(curNode);

    // A* path finding
    while (!pq[index].empty()) {
        // get the current node with the highest priority from open list
        curNode = pq[index].top();

        curRow = curNode.getXPos();
        curCol = curNode.getYPos();

        // remove the node from the open list
        pq[index].pop();

        // mark it on open/close map
        openNodes[curRow][curCol] = 0;
        closedNodes[curRow][curCol] = 1;

        // quit searching when the destination is reached
        if (curRow == xNodeDest && curCol == yNodeDest) {
            // generate the path from destination to start
            // by following the directions
            path = "";
            while (!(curRow == xNodeStart && curCol == yNodeStart)) {
                j = dirMap[curRow][curCol];
                path = static_cast<char>('0' + (j + DIR_CAP / 2) % DIR_CAP) + path;
                curRow += MY[j];
                curCol += MX[j];
            }

            // empty the leftover nodes
            pq[index] = priority_queue<Node>();

            setPath(path);
            return path;
        }

        // traverse neighbors
        for (i = 0; i < DIR_CAP; i++) {
            // neighbor coordinates
            newRow = curRow + MY[i];
            newCol = curCol + MX[i];

            // not evaluated & not outside (bound checking)
            if (!(newRow < 0 || newRow > M_HEIGHT - 1 || newCol < 0 || newCol > M_WIDTH - 1
                || gameMap[newRow][newCol] >= 1 || closedNodes[newRow][newCol] == 1)) {

                // generate a child node
                Node childNode(newRow, newCol, curNode.getLevel(), curNode.getPriority());
                childNode.nextLevel(i);
                childNode.updatePriority(xNodeDest, yNodeDest);

                // if it is not in the open list then add into that
                if (openNodes[newRow][newCol] == 0) {
                    openNodes[newRow][newCol] = childNode.getPriority();
                    pq[index].push(childNode);

                    // update the parent direction info
                    dirMap[newRow][newCol] = (i + DIR_CAP / 2) % DIR_CAP;
                } else if (openNodes[newRow][newCol] > childNode.getPriority()) {
                    // update the priority info
                    openNodes[newRow][newCol] = childNode.getPriority();

                    // update the parent direction info
                    dirMap[newRow][newCol] = (i + DIR_CAP / 2) % DIR_CAP;

                    // use a backup queue to put the best node (with highest priority)
                    // onto the top of the queue, which can be chosen later to build the path.
                    while (!(pq[index].top().getXPos() == newRow &&
                           pq[index].top().getYPos() == newCol)) {
                        pq[1 - index].push(pq[index].top());
                        pq[index].pop();
                    }

                    pq[index].pop();

                    if (pq[index].size() > pq[1 - index].size()) {
                        index = 1 - index;
                    }

                    while (!pq[index].empty()) {
                        pq[1 - index].push(pq[index].top());
                        pq[index].pop();
                    }

                    index = 1 - index;
                    pq[index].push(childNode);
                }
            }
        }
    }

    return ""; // no route found
}

/**
 * Date:    March 20, 2017
 *
 * Author:  Fred Yang
 *
 * Function Interface: void validateBlockMatrix()
 *
 * Returns: void
 *
 * Note:
 * Called to validate blocking matrix.
 */
void Zombie::validateBlockMatrix() {
    if (gameMap[0][0] == 0) {
        gameMap = GameManager::instance()->getAiMap();
    }
}

/**
 * Date:    March 28, 2017
 *
 * Author:  Mark Tattrie
 *
 * Function Interface: void zAttack()
 *
 * Returns: void
 *
 * Note:
 * Calls the zombies current weapon "ZombieHands" to fire
 */
void Zombie::zAttack(){
    Weapon* w = inventory.getCurrent();
    if (w) {
        w->fire(*this);
    } else {
        logv("Zombie Slot Empty\n");
    }
}

/**
* Date: March 27, 2017
*
* Designer: Trista Huang
*
* Programmer: Trista Huang
*
* Interface: void Marine::updateImageDirection()
*
* Description:
*       This function changes the direction that the character is facing.
*       It is called from GameStateMatch::handle after every frame and
*       updates the direction of the sprite according to the angle of the
*       mouse from the center of the screen.
*/
void Zombie::updateImageDirection() {
    const double radians = getRadianAngle();

    //order: start from ~0 rad, counter clockwise
    if (radians > ZOMBIE_SPRITE_ANGLE1 && radians < ZOMBIE_SPRITE_ANGLE2) {
        setSrcRect(getSrcRect().x, SPRITE_FRONT, SPRITE_SIZE_X, SPRITE_SIZE_Y);
    } else if (radians > ZOMBIE_SPRITE_ANGLE2 && radians < ZOMBIE_SPRITE_ANGLE3) {
        setSrcRect(getSrcRect().x, SPRITE_FRONT_RIGHT, SPRITE_SIZE_X, SPRITE_SIZE_Y);
    } else if (radians > ZOMBIE_SPRITE_ANGLE3 && radians < ZOMBIE_SPRITE_ANGLE4) {
        setSrcRect(getSrcRect().x, SPRITE_RIGHT, SPRITE_SIZE_X, SPRITE_SIZE_Y);
    } else if (radians > ZOMBIE_SPRITE_ANGLE4 && radians < ZOMBIE_SPRITE_ANGLE5) {
        setSrcRect(getSrcRect().x, SPRITE_BACK_RIGHT, SPRITE_SIZE_X, SPRITE_SIZE_Y);
    } else if (radians > ZOMBIE_SPRITE_ANGLE5 && radians < ZOMBIE_SPRITE_ANGLE6) {
        setSrcRect(getSrcRect().x, SPRITE_BACK, SPRITE_SIZE_X, SPRITE_SIZE_Y);
    } else if (radians > ZOMBIE_SPRITE_ANGLE6 && radians < ZOMBIE_SPRITE_ANGLE7) {
        setSrcRect(getSrcRect().x, SPRITE_BACK_LEFT, SPRITE_SIZE_X, SPRITE_SIZE_Y);
    } else if (radians > ZOMBIE_SPRITE_ANGLE7 && radians < ZOMBIE_SPRITE_ANGLE9) {
        setSrcRect(getSrcRect().x, SPRITE_LEFT, SPRITE_SIZE_X, SPRITE_SIZE_Y);
    } else if (radians > ZOMBIE_SPRITE_ANGLE8 && radians < ZOMBIE_SPRITE_ANGLE9) {
        setSrcRect(getSrcRect().x, SPRITE_FRONT_LEFT, SPRITE_SIZE_X, SPRITE_SIZE_Y);
    }
}
