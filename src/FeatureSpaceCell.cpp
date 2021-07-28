#include "FeatureSpaceCell.hpp"

void FeatureSpaceCell::ComputeConnectivity(){
    std::stack<std::pair<int, int>> stack;
    std::vector<std::vector<int>> visited(board_.GetRows(), std::vector<int>(board_.GetColumns()));
    std::vector<std::vector<int>> room(board_.GetRows(), std::vector<int>(board_.GetColumns()));
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();
    std::vector<Box> boxes = board_.GetBoxes();
    int nRows = board_.GetRows();
    int nCols = board_.GetColumns();

    for(unsigned int i = 1; i < nRows - 1; i++){
        for(unsigned int j = 1; j < nCols - 1; j++){
            BlockType type = blocks[i][j].GetType();
            if(visited[i][j] == 0 && (type == Floor || type == Goal) && !blocks[i][j].IsOccupied()){
                connectivity_++;
                stack.push(std::make_pair(i,j));
                while(!stack.empty()){
                    int x = stack.top().first;
                    int y = stack.top().second;
                    room[x][y] = connectivity_;
                    stack.pop();
                    if(visited[x][y] == 0){
                        visited[x][y] = 1;
                        std::vector<Block> neighbours = blocks[x][y].GetNeighbours();
                        for(auto neighbour = neighbours.begin(); neighbour != neighbours.end(); neighbour++){
                            type = (*neighbour).GetType();
                            if ((type == Floor || type == Goal) && !(*neighbour).IsOccupied()){
                                stack.push(std::make_pair((*neighbour).GetX(), (*neighbour).GetY()));
                            }
                        }
                    }
                }
            }
        }
    }

    rooms_ = room;
}

void FeatureSpaceCell::FindSinkRoom(){
    std::vector<int> candidate_rooms(connectivity_, 0);
    std::vector<Box> boxes = board_.GetBoxes();

    // find a sink room -- a basin with the most boxes in it at the beginning of the level
    for(auto box = boxes.begin(); box != boxes.end(); box++){
        int x = (*box).GetX();
        int y = (*box).GetY();
        std::vector<int> checked_rooms;
        if(rooms_[x-1][y] > 0) {
            checked_rooms.push_back(rooms_[x-1][y]);
            candidate_rooms[rooms_[x-1][y] - 1]++;
            }
        if(rooms_[x+1][y] > 0 && 
        std::find(checked_rooms.begin(), checked_rooms.end(), rooms_[x+1][y]) == checked_rooms.end()) {
            checked_rooms.push_back(rooms_[x+1][y]);
            candidate_rooms[rooms_[x+1][y] - 1]++;
            }
        if(rooms_[x][y-1] > 0 && 
        std::find(checked_rooms.begin(), checked_rooms.end(), rooms_[x][y-1]) == checked_rooms.end()) {
            checked_rooms.push_back(rooms_[x][y-1]);
            candidate_rooms[rooms_[x][y-1] - 1]++;
            }
        if(rooms_[x][y+1] > 0 && 
        std::find(checked_rooms.begin(), checked_rooms.end(), rooms_[x][y+1]) == checked_rooms.end()) {
            candidate_rooms[rooms_[x][y+1] - 1]++;
            }
    }
    int sink_room = 1 + std::max_element(candidate_rooms.begin(), candidate_rooms.end()) - candidate_rooms.begin();

    //update room numbers for box positions -- initially set to 0
    for(auto box = boxes.begin(); box != boxes.end(); box++){
        int x = (*box).GetX();
        int y = (*box).GetY();
        std::vector<int> rooms_around(connectivity_, 0);
        if(rooms_[x-1][y] > 0) {
            rooms_around[rooms_[x-1][y] - 1] = candidate_rooms[rooms_[x-1][y] - 1];
            }
        if(rooms_[x+1][y] > 0) {
            rooms_around[rooms_[x+1][y] - 1] = candidate_rooms[rooms_[x+1][y] - 1];
            }
        if(rooms_[x][y-1] > 0) {
            rooms_around[rooms_[x][y-1] - 1] = candidate_rooms[rooms_[x][y-1] - 1];
            }
        if(rooms_[x][y+1] > 0) {
            rooms_around[rooms_[x][y+1] - 1] = candidate_rooms[rooms_[x][y+1] - 1];
            }
        rooms_[x][y] = 1 + std::max_element(rooms_around.begin(), rooms_around.end()) - rooms_around.begin();
    }
    std::cout<<"Our suggested sink room is room number "<<sink_room<<std::endl;
    sink_room_ = sink_room;
}

void FeatureSpaceCell::PrintRooms(){
    for(unsigned int i = 0; i < rooms_.size(); i++){
        for(unsigned int j = 0; j < rooms_[i].size(); j++){
            std::cout<<rooms_[i][j];
        }
        std::cout<<std::endl;
    }
}

void FeatureSpaceCell::ComputePacking(){
    //set up initial position, all boxes on targets
    std::vector<Block> goals = board_.GetGoals();
    std::vector<Box> boxes_on_goals;
    //TODO: REMEMBER TO UPDATE BLOCKS OCCUPANCY FOR THE INITIAL POSITION AS WELL
    for (auto goal : goals){
        boxes_on_goals.push_back(Box(goal.GetX(), goal.GetY(), true));
    }
    BoardState initial_board = BoardState(board_.GetBlocks(), boxes_on_goals);
    //set up list of feature cells in feature space
    std::vector<PackingPlanFeatureSpaceCell> feature_space;
    //compute initial feature space cell and add it to the list
    PackingPlanFeatureSpaceCell initial_cell = PackingPlanFeatureSpaceCell(initial_board);
    feature_space.push_back(initial_cell);

    // set up the move "tree" -- a map (state -> weight) should be sufficient probably
    // push back the initial domain state
    std::map<BoardState, int> move_tree;
    move_tree[initial_board] = 0;

    BoardState current_state = initial_board;
    PackingPlanFeatureSpaceCell current_cell = initial_cell;
    std::vector<Box> boxes_on_board = boxes_on_goals;

    //custom compare function to keep track of best moves
	auto cmp = [](std::pair<BoardState, PackingPlanFeatureSpaceCell> left, std::pair<BoardState, PackingPlanFeatureSpaceCell> right) { 
		if(left.second.GetBoxesOnBoard() < right.second.GetBoxesOnBoard()) return false; 
		else if(left.second.GetBoxesOnTarget() < right.second.GetBoxesOnTarget()) return false;
		else if(left.second.GetDistance() > right.second.GetDistance()) return false;
		return true;

	 };

    //search feature space while all the boxes are not deleted
    //or the maximum number of iterations is not reached (NOT ADDED YET)
    while(current_cell.GetBoxesOnBoard() != 0){
        // TODO: have to add cycling through all states that correspond to the feature cell :(((
        // for the initial state it's just one, but it get worse pretty fast oh well

        //locally explored moves
        std::set<BoardState> explored_moves;
        //find all possible moves from the current state
        std::priority_queue<std::pair<BoardState, PackingPlanFeatureSpaceCell>, std::vector<std::pair<BoardState, PackingPlanFeatureSpaceCell>>, decltype(cmp)> possible_moves(cmp);
        //look at moving one box at a time
        //TODO: check that local current state and current state are used correctly
        for(unsigned int i = 0; i < boxes_on_board.size(); i++){
            // moves available for i_th box only
            // tries to find all available macro moves for this box

            // TODO: should take into account if the box can be pushed in this position or if it's its starting position
            std::stack<BoardState> available_moves;
            BoardState current_local_state = current_state;
            available_moves.push(current_local_state);
            while(!available_moves.empty()){
                current_local_state = available_moves.top();
                available_moves.pop();
                //if the move hasn't been explored yet
                if(explored_moves.find(current_local_state) == explored_moves.end()){
                    // mark move as checked
                    explored_moves.insert(current_local_state);
                    
                    int x = current_local_state.GetBoxes()[i].GetX();
                    int y = current_local_state.GetBoxes()[i].GetY();
                    // if a box reached a sink room, remove it from the board and stop exploration for this box
                    if(rooms_[x][y] == sink_room_){
                        std::vector<Box> new_boxes = current_local_state.GetBoxes();
                        std::vector<std::vector<Block>> new_blocks = current_local_state.GetBlocks();
                        new_boxes.erase(new_boxes.begin() + i);
                        new_blocks[x][y].Free();
                        current_local_state = BoardState(new_blocks, new_boxes, &current_state);
                        // add it to the possible moves
                        possible_moves.push(std::make_pair(current_local_state, PackingPlanFeatureSpaceCell(current_local_state)));
                        while(!available_moves.empty()) available_moves.pop();
                    } else {
                        // add current move to moves possible from the current state
                        possible_moves.push(std::make_pair(current_local_state, PackingPlanFeatureSpaceCell(current_local_state)));
                        //otherwise check if there is enough space to pull the box in each four directions
                        std::vector<std::vector<Block>> blocks = current_local_state.GetBlocks();
                        std::vector<Box> new_boxes;
                        std::vector<std::vector<Block>> new_blocks;
                        //check North
                        if(!blocks[x-1][y].IsOccupied() && !blocks[x-2][y].IsOccupied()){
                            new_boxes = current_local_state.GetBoxes();
                            new_blocks = blocks;
                            new_blocks[x][y].Free();
                            new_blocks[x-1][y].Occupy();
                            new_boxes[i].Move(North);
                            available_moves.push(BoardState(new_blocks, new_boxes, &current_state));
                        }
                        //check South
                        if(!blocks[x+1][y].IsOccupied() && !blocks[x+2][y].IsOccupied()){
                            new_boxes = current_state.GetBoxes();
                            new_blocks = blocks;
                            new_blocks[x][y].Free();
                            new_blocks[x+1][y].Occupy();
                            new_boxes[i].Move(South);
                            available_moves.push(BoardState(new_blocks, new_boxes, &current_state));
                        }
                        //check East
                        if(!blocks[x][y+1].IsOccupied() && !blocks[x][y+2].IsOccupied()){
                            new_boxes = current_state.GetBoxes();
                            new_blocks = blocks;
                            new_blocks[x][y].Free();
                            new_blocks[x][y+1].Occupy();
                            new_boxes[i].Move(East);
                            available_moves.push(BoardState(new_blocks, new_boxes, &current_state));
                        }
                        //check West
                        if(!blocks[x][y-1].IsOccupied() && !blocks[x][y-2].IsOccupied()){
                            new_boxes = current_state.GetBoxes();
                            new_blocks = blocks;
                            new_blocks[x][y].Free();
                            new_blocks[x][y-1].Occupy();
                            new_boxes[i].Move(West);
                            available_moves.push(BoardState(new_blocks, new_boxes, &current_state));
                        }
                    }
                }
            }
        }
        
        //pick the best move according to features and distance:
        //1) if the box can be removed, pick this move. If more than one box can be removed, pick the one with higher distance
        //2) otherwise if a box can be moved from target, move it. Same as before, with several moves pick the one with higher distance
        //3) if 1) and 2) are not possible, just try to maximize the distance.
        // if the queue was defined correctly (hopefully) the best move is the first one in the queue of possible moves
        BoardState best_move = possible_moves.top().first;
        PackingPlanFeatureSpaceCell best_cell = possible_moves.top().second;

        //add a move to the tree
        //TODO: have to determine somehow if the move was picked by advisor or not
        move_tree[best_move] = move_tree[current_state] + 1;
        //check if the cell in feature space was already discovered:
        //if yes, add the move to the list of the domain states that project on this cell
        //if no, add new feature cell to the feature space
        auto it = std::find(feature_space.begin(), feature_space.end(), best_cell);
        PackingPlanFeatureSpaceCell cell = (*it);
        if(it != feature_space.end()){
            it->AddBoard(best_move);
        } else {
            feature_space.push_back(best_cell);
        }

        //pick a new cell
        //pick an unexplored move that projects onto that cell

        //add weights to the moves somehow
    }
    /// TODO
}

void FeatureSpaceCell::ComputeOutOfPlan(){
    /// TODO
}