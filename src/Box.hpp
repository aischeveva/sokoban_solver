#ifndef BOX_HPP
#define BOX_HPP


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

/* enumeration types for directions */
enum Direction {
    North,
    East,
    South,
    West
};

class Box {
    private:
        int xCoordinate_;
        int yCoordinate_;
        bool isDelivered_;
    
    public:
        Box(int xCoord, int yCoord):xCoordinate_(xCoord), yCoordinate_(yCoord), isDelivered_(false){}

        /* Getters */
        int GetX() const {return xCoordinate_;}
        int GetY() const {return yCoordinate_;}
        bool isDelivered() const {return isDelivered_;}

        /* Setters */
        void deliver() {isDelivered_ = true;}
        void push(Direction dir);
};

#endif