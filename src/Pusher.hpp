#ifndef PUSHER_HPP
#define PUSHER_HPP

#include "Object.hpp"
/** 
 * \class Pusher
 * \brief Player that pushes the boxes.
 * This class represents player that pushes boxes.
 * 
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