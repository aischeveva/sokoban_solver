#include "Solver.hpp"

bool PackingAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);

bool Solver::ConnectivityAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state){
    if(next_state.GetConnectivity() < current_state.GetConnectivity()) return true;
    else return false;
}
bool Solver::RoomConnectivityAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state){
    if(next_state.GetRoomConnectivity() < current_state.GetRoomConnectivity()) return true;
    else return false;
}

bool HotspotsAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
bool ExplorerAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);
bool OpenerAdvisor(FeatureSpaceCell current_state, FeatureSpaceCell next_state);