#ifndef FEATURESPACE_HPP
#define FEATURESPACE_HPP

#include "Board.hpp"

/** 
 * \class FeatureSpace
 * \brief Feature space to solve sokoban problem.
 * Feature space is the key element of the FESS algorithm.
 * Following the Festival solver, this implementation also utilizes a four-dimensional feature space.
 * Feature space includes packing, connectivity, room connectivity and out-of-plan features.
 * 
 * 
 * \author A. SHCHEVYEVA
 * \version 1.1 
 * 
 * Created on: 16/06/2021
 *
 */ 

class FeatureSpace{
    private:
        Board board_;
        int packing_;
        int connectivity_;
        int room_connectivity_;
        int out_of_plan_;

    public:
        FeatureSpace(Board board): board_(board) {}

        /* getters */
        int GetPacking() const {return packing_;}
        int GetConnectivity() const {return connectivity_;}
        int GetRoomConnectivity() const {return room_connectivity_;}
        int GetOutOfPlan() const {return out_of_plan_;}

        /* compute heuristics */
        void ComputePacking();
        void ComputeConnectivity();
        void ComputeRoomConnectivity();
        void ComputeOutOfPlan();

};

#endif