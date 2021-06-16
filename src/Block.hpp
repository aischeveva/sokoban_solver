#ifndef BLOCK_HPP
#define BLOCK_HPP
#include <iostream>

/** 
 * \class Block
 * \brief Generic block.
 * This class manages generic map block interface.
 * 
 * 
 * \author A. SHCHEVYEVA
 * \version 1.1 
 * 
 * Created on: 02/06/2021
 *
 */ 

/* enumeration types for different kind of blocks */
enum BlockType {
    Wall,
    Floor,
    Goal,
    Outer
};


class Block {
    private:
        int xCoordinate_; ///< x-coordinate of the block
        int yCoordinate_; ///< y-coordinate of the block
        BlockType type_;  ///< block type
        bool occupied_;   ///< whether the block is occupied by a box or a player or not
    
    public:
        /** \fn Block(int xCoord, int yCoord, BlockType type, bool occupied)
         *  \brief Constructor
         *  \param xCoord x-coordinate of the block
         *  \param yCoord y-coordinate of the block
         *  \param type block type: Wall, Floor, Goal or Outer
         *  \param occupied whether the block is occupied by a box or a player or not
         * 
         */ 
        Block(int xCoord, int yCoord, BlockType type, bool occupied):xCoordinate_(xCoord), yCoordinate_(yCoord), type_(type), occupied_(occupied){}

        /* Getters*/

        /** \fn int GetX() const
         *  \brief Get x-coordinate of the block
         *  \return x-coordinate of the block
         */
        int GetX() const {return xCoordinate_;}

        /** \fn int GetY() const
         *  \brief Get y-coordinate of the block
         *  \return y-coordinate of the block
         */
        int GetY() const {return yCoordinate_;}

        /** \fn BlockType GetType() const
         *  \brief Get the block type
         *  \return block type: Wall, Floor, Goal or Outer
         */
        BlockType GetType() const {return type_;}

        /** \fn bool isOccupied() const
         *  \brief Check if the block is currently occupied
         *  \return True, if the block is occupied, false otherwise.
         */
        bool isOccupied() const {return occupied_;}

        /* Setters */

        /** \fn void occupy()
         *  \brief Occupy this block by setting occupied_ to true
         */
        void occupy() {occupied_ = true;}

        /** \fn void free()
         *  \brief Free this block by setting occupied_ to false
         */
        void free() {occupied_ = false;}

        void print() {std::cout<<"Block of type "<<type_<<" at ("<<xCoordinate_<<","<<yCoordinate_<<"), occupied: "<<occupied_<<std::endl;}
};

#endif