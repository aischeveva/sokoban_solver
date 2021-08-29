#ifndef PUSHER_HPP
#define PUSHER_HPP

#include "Object.hpp"

/** 
 * \class Pusher
 * \brief Player that pushes the boxes.
 * This class represents player that pushes boxes.
 * Currently it is used fairly limited, 
 * but it is important to check that the player can actually perform moves 
 * (move from one location to another to access a box for push or pull).
 * 
 * The block at player's coordinates is marked as not occupied, because it is assumed that the player could move
 * elsewhere if needed to push/pull a box on their current location. Might be necessary to change this set up in the future.
 * 
 * \author A. SHCHEVYEVA
 * \version 1.1 
 * 
 * Created on: 10/06/2021
 *
 */ 

class Pusher: public Object {
    public:
        Pusher(){}
        Pusher(unsigned int x, unsigned int y): Object(x, y){}
};

#endif