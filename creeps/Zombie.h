/*------------------------------------------------------------------------------
* Header:     Zombie.h
*
* Date:       February 1, 2017
*
* Revisions:
* Edited By:  Yiaoping Shu- Style guide
*
* Designer:   GM Team, AI Team
*
* Author:     GM Team, AI Team
*
* Notes:
* This header file includes the macro definitions and function prototypes
* for Zombie class.
------------------------------------------------------------------------------*/
#ifndef ZOMBIE_H
#define ZOMBIE_H

#include <string>
#include <cmath>
#include <random>
#include <vector>
#include <utility>
#include <SDL2/SDL.h>

#include "../collision/HitBox.h"
#include "../basic/Entity.h"
#include "../collision/CollisionHandler.h"
#include "../inventory/Inventory.h"
#include "../collision/Quadtree.h"
#include "../buildings/Base.h"
#include "../view/Window.h"
#include "../basic/Movable.h"

typedef std::pair<float, float> Point;

// zombie properties
static constexpr int ZOMBIE_HEIGHT   = 125;
static constexpr int ZOMBIE_WIDTH    = 75;
static constexpr int ZOMBIE_INIT_HP  = 100;
static constexpr int ZOMBIE_VELOCITY = 250;
static constexpr int ZOMBIE_FRAMES   = 100;

// block threshold - check if zombie is blocked
static constexpr float BLOCK_THRESHOLD = 0.5;

// tower zone - check if zombie appears in the vicinity of the base
static constexpr int BASE_ZONE = 5;

/**
 * Date:    February 14, 2017
 * Author:  Fred Yang
 * Note:
 * 8 possible directions combining left, right, up, down.
 */
enum class ZombieDirection : int {
    DIR_R,
    DIR_RD,
    DIR_D,
    DIR_LD,
    DIR_L,
    DIR_LU,
    DIR_U,
    DIR_RU,
    DIR_INVALID = -1
};

/**
 * Date:    March 14, 2017
 * Author:  Robert Arendac
 * Note:
 * Cardinal directions for setting angles, one angle for each movement direction.
 */
enum class ZombieAngles : int {
    NORTH = 0,
    NORTHEAST = 45,
    EAST = 90,
    SOUTHEAST = 135,
    SOUTH = 180,
    SOUTHWEST = 225,
    WEST = 270,
    NORTHWEST = 315
};

/**
 * Date:    March 14, 2017
 * Author:  Fred Yang
 * Note:
 * Zombie states, change when you want zombie to take a different action.
 * Eg. go from moving to attacking
 */
enum class ZombieState {
    ZOMBIE_IDLE,
    ZOMBIE_MOVE,
    ZOMBIE_ATTACK,
    ZOMBIE_DIE
};

class Zombie : public Movable {
public:
    // constructor
    Zombie(const int32_t id, const SDL_Rect& dest, const SDL_Rect& movementSize, const SDL_Rect& projectileSize,
        const SDL_Rect& damageSize, const int health = ZOMBIE_INIT_HP,
        const ZombieState state = ZombieState::ZOMBIE_IDLE,
        const int step = 0, const ZombieDirection dir = ZombieDirection::DIR_INVALID,
        const int frame = ZOMBIE_FRAMES);

    virtual ~Zombie();                      // destructor

    void onCollision();                     // collision checks

    void collidingProjectile(int damage);   // projectile damage method

    void move(float moveX, float moveY, CollisionHandler& ch) override;  // move method
    
    int getHealth() const {return health;}  // hp getter
    
    void setHealth(const int h) {health = h;} // hp setter

    void generateMove();                    // A* movement method

    bool isMoving() const;                  // Returns if the zombie should be moving

    int detectObj() const;                  // detect objects in vicinity

    void validateBlockMatrix();             // validate blocking matrix

    ZombieDirection getMoveDir();           // get move direction
    
    ZombieDirection getMonteDir();          // get Monte Carlo move direction

    std::string generatePath(const Point& start); // A* method
    std::string generatePath(const Point& start, const Point& dest); // A* method

    /**
     * Date:    February 14, 2017
     * Author:  Fred Yang 
     * Note:    steps setter
     */
    void setStep(const int sp) {
        step = sp;
    }

    /**
     * Date:    February 14, 2017
     * Author:  Fred Yang 
     * Note:    steps getter
     */
    int getStep() const {
        return step;
    }

    /**
     * Date:    February 14, 2017
     * Author:  Fred Yang 
     * Note:    state setter
     */
    void setState(const ZombieState newState) {
        state = newState;
    }

    /**
     * Date:    February 14, 2017
     * Author:  Fred Yang 
     * Note:    state setter
     */
    ZombieState getState() const {
        return state;
    }

    /**
     * Date:    February 14, 2017
     * Author:  Fred Yang 
     * Note:    A* path getter
     */
    string getPath() const {
        return path;
    }

    /**
     * Date:    February 14, 2017
     * Author:  Fred Yang 
     * Note:    A* path setter
     */
    void setPath(const string pth) {
        path = pth;
    }

    /**
     * Date:    February 14, 2017
     * Author:  Fred Yang 
     * Note:    set current moving direction
     */
    void setCurDir(const ZombieDirection d) {
        dir = d;
    }

    /**
     * Date:    February 14, 2017
     * Author:  Fred Yang 
     * Note:    get current moving direction
     */
    ZombieDirection getCurDir() const {
        return dir;
    }

    /**
     * Date:    March 14, 2017
     * Author:  Fred Yang 
     * Note:    frame setter
     */
    void setCurFrame(const int frm) {
        frame = frm;
    }

    /**
     * Date:    March 14, 2017
     * Author:  Fred Yang 
     * Note:    frame getter
     */
    int getCurFrame() const {
        return frame;
    }

private:
    int health;         // health points of zombie
    std::string path;   // A* path zombie should follow
    ZombieState state;  // 0 - idle, 1 - move, 2 - attack, 3 - die
    int step;           // Number of steps zombie has taken in path
    ZombieDirection dir;// moving direction
    int frame;          // frames per tile
    bool targeted;      // indicate if zombie appears in base zone
    Inventory inventory;//inventory holds a weapon used to attack

    void zAttack();
};

#endif
