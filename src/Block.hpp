#ifndef BLOCK_HPP
#define BLOCK_HPP

#include <iostream>
#include <vector>

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
        std::vector<Block> neighbours_; ///< vector of Block neighbours
        //int neighbour_count_;
    
    public:
        Block(){};
        /** \fn Block(int xCoord, int yCoord, BlockType type, bool occupied)
         *  \brief Constructor
         *  \param xCoord x-coordinate of the block
         *  \param yCoord y-coordinate of the block
         *  \param type block type: Wall, Floor, Goal or Outer
         *  \param occupied whether the block is occupied by a box or a player or not
         * 
         */ 
        Block(int xCoord, int yCoord, BlockType type, bool occupied):xCoordinate_(xCoord), yCoordinate_(yCoord), type_(type), occupied_(occupied) /*, neighbours_(4), neighbour_count_(0)*/{}

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

        std::vector<Block> GetNeighbours() const {return neighbours_;}

        /** \fn bool isOccupied() const
         *  \brief Check if the block is currently occupied
         *  \return True, if the block is occupied, false otherwise.
         */
        bool IsOccupied() const {return occupied_;}

        /* Setters */

        /** \fn void occupy()
         *  \brief Occupy this block by setting occupied_ to true
         */
        void Occupy() {occupied_ = true;}

        /** \fn void free()
         *  \brief Free this block by setting occupied_ to false
         */
        void Free() {occupied_ = false;}

        void ChangeType(BlockType type) {type_ = type;}

        void AddNeighbour(Block block) {neighbours_.push_back(block); /*[neighbour_count_] = block; neighbour_count_++;*/}

        void Print() {std::cout<<"Block of type "<<type_<<" at ("<<xCoordinate_<<","<<yCoordinate_<<"), occupied: "<<occupied_<<std::endl;}

        friend std::ostream& operator<<(std::ostream& os, const Block& b){
            switch (b.GetType())
            {
            case 0: os<<"#"; break;
            case 1: os<<" "; break;
            case 2: os<<"."; break;
            case 3: os<<"_"; break;
            default: break;
            }

            return os;
        }
        friend bool operator==(const Block& b1, const Block& b2){
            if(b1.GetX() == b2.GetX() && b1.GetY() == b2.GetY()) return true;
            else return false;
        }
};

#endif