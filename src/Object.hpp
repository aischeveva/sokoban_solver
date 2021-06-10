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
        int xCoordinate_;
        int yCoordinate_;

    public:
        Object(int x, int y): xCoordinate_(x), yCoordinate_(y){}

        /* Getters */
        int GetX() const {return xCoordinate_;}
        int GetY() const {return yCoordinate_;}

        void move(Direction dir);
};


#endif