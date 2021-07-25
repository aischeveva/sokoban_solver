#ifndef OBJECT_HPP
#define OBJECT_HPP


/** 
 * \class Object
 * \brief Movable object on the board.
 * This class represents either player or the boxes.
 * 
 * 
 * \author A. SHCHEVYEVA
 * \version 1.1 
 * 
 * Created on: 10/06/2021
 *
 */ 

/* enumeration types for directions */
enum Direction {
    North,
    East,
    South,
    West
};


class Object {
    protected:
        unsigned int xCoordinate_;
        unsigned int yCoordinate_;

    public:
        Object(){}
        Object(unsigned int x, unsigned int y): xCoordinate_(x), yCoordinate_(y){}

        /* Getters */
        unsigned int GetX() const {return xCoordinate_;}
        unsigned int GetY() const {return yCoordinate_;}

        void Move(Direction dir);

        bool operator< (const Object& right) const { return xCoordinate_ < right.GetX();}
        friend bool operator==(const Object& o1, const Object& o2);
};


#endif