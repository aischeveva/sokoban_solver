#ifndef BOX_HPP
#define BOX_HPP

#include "Object.hpp"

/** 
 * \class Box
 * \brief A box that needs to be delivered to the goal zone.
 * This class manages box interface.
 * 
 * 
 * \author A. SHCHEVYEVA
 * \version 1.1 
 * 
 * Created on: 02/06/2021
 *
 */ 

class Box: public Object {
    private:
        bool isDelivered_;
    
    public:
        Box(unsigned int x, unsigned int y): Object(x, y){ isDelivered_ = false; }
        Box(unsigned int x, unsigned int y, bool isDelivered): Object(x, y){ isDelivered_ = isDelivered;}

        /* Getters */
        bool isDelivered() const {return isDelivered_;}

        /* Setters */
        void deliver() {isDelivered_ = true;}
        void unDeliver() {isDelivered_ = false;}
};

#endif