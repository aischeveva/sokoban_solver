#ifndef FEATURESPACECELL_HPP
#define FEATURESPACECELL_HPP

#include <algorithm>
#include <functional>
#include <map>
#include <queue>
#include <stack>
#include <vector>
#include <utility>

#include "BoardState.hpp"
#include "PackingPlanFeatureSpaceCell.hpp"

/** 
 * \class FeatureSpaceCell
 * \brief Feature space cell used to solve sokoban problem.
 * Feature space is the key element of the FESS algorithm.
 * Following the Festival solver, this implementation also utilizes a four-dimensional feature space.
 * Feature space cell is defined by packing (order), connectivity, room connectivity and out-of-plan features.
 * In the paper feature space is represented as a grid or coordinate system, 
 * but it seems that for implementation purposes a vector of active feature space cells is sufficient.
 * 
 * \author A. SHCHEVYEVA
 * \version 1.1 
 * 
 * Created on: 16/06/2021
 *
 */ 

/*TODO:
    * change boardState to a vector of projected domain states
    * packing plan: keep the order of the targets (i.e., which targets should be filled in first)
    *               packing advisor prioritizes moves that pack a box on a target that is first on the packing order list
    *               once box is packed there, the target is removed from the list to make way for the next top priority target

* How feature space works:     initiate with a root and corresponding active feature cell
    *                          expand the root by finding all possible moves
    *                          pick the best move and find a feature cell corresponding to the domain state after this move
    *                               it can be viewed as a vector of feature cells, each cell has a vector of domain states corresponding to it, each domain state has a reference to its parent state
    *                               root parent reference is null
    *                               once you found solution, rebuild the path by parent links
    *                          after that pick a feature cell from the active cells
    *                          and expand all unexpanded moves that correspond to this cell -- repeat with finding the best move 
    *                          expanding unexpanded moves means to find all moves (that are not in the tree yet) from the corresponding states

* About macro moves: pick a box, and treat it as an only moving object as long as you can move it
    *                    that would create a macro move basically
    * 
* Computing rooms correctly:
    * Traverse level map with 2x3 and 3x2 rectangles
    * if all tiles are floor tiles, that's a minimum room
    * if two minimum room overlap in at least two blocks, it's actually one room
    * if two rooms overlap in one block only, it's actually two separate rooms and the block belongs to none of them
    * keep the room info as a map: block coordinate (x, y) -> room number
    * if the block coordinates are not in a map, block doesn't belong to any room
*/
class FeatureSpaceCell{
    private:
        BoardState board_; //change it to vector of boards
        std::vector<BoardState*> corresponding_boards_;
        std::map<std::pair<int, int>, int> room_by_coord_; // find the room number by the block's coordinates
        std::map<std::pair<int, int>, int> basin_by_coord_; // find the basin number by the block's coordinates
        std::map<int, std::vector<Block>> blocks_by_room_; // find all blocks belonging to the i_th room
        std::map<int, std::vector<Block>> basins_;  // find all blocks belonging to the i_th basin
        std::map<int, int> basin_by_room_;  // find to which basin a room belongs
        std::vector<std::vector<bool>> adjacency_; // adjacency matrix for rooms when there are no boxes on board. Two rooms are connected if there is a path between them that doesn't cross another room
        int room_number_; // total number of rooms
        int sink_room_; // the number of the sink room
        int sink_room_basin_; // the number of the basin that corresponds to the sink room
        int packing_number_; // how many boxes have been packed
        std::vector<Block> packing_order_; // the order in which boxes ought to be packes (currently represented by the order of goal blocks, but it doesn't take into account parking)
        int connectivity_; // how many areas of the board are currently disconnected. When the level is solved, connectivity has to be minimized
        int room_connectivity_; // how many rooms are currently disconnected. Rooms counted as disconnected if there is currently no path between them that doesn't cross another room, but they should be connected according to the adjacency matrix.
        int out_of_plan_; // how many boxes are not in a basin/packed or parked.

    public:
        FeatureSpaceCell(){}
        FeatureSpaceCell(BoardState *boardState): board_(*boardState), connectivity_(0), room_connectivity_(0), out_of_plan_(0) { corresponding_boards_.push_back(boardState); }

        /* getters */
        std::vector<Block> GetPackingOrder() const {return packing_order_;}
        int GetConnectivity() {ComputeConnectivity(); return connectivity_;}
        int GetRoomConnectivity() {ComputeRoomConnectivity(); return room_connectivity_;}
        int GetOutOfPlan() {ComputeOutOfPlan(); return out_of_plan_;}
        int GetRoomNumber() {FindRooms(); return room_number_;}
        std::vector<BoardState*> GetStates() const {return corresponding_boards_;}

        /* compute heuristics */
        void ComputePackingNumber();
        void ComputePackingOrder();
        void ComputeConnectivity();
        void ComputeRoomConnectivity();
        void ComputeOutOfPlan();

        /* preparational functions and misc*/
        void AddBoard(BoardState* board);
        std::map<std::pair<int,int>, int> FindRooms();
        void FindBasins();
        std::vector<std::vector<bool>> ComputeAdjacency();
        void PrintRooms();

};

#endif