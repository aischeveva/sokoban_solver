#include "FeatureSpaceCell.hpp"

void FeatureSpaceCell::ComputePackingNumber(){
    auto goals = board_.GetGoals();
    packing_number_ = 0;
    for(auto goal : goals){
        if(goal.IsOccupied()) packing_number_++;
    }
}

void FeatureSpaceCell::ComputePackingOrder(){
    //set up initial position, all boxes on targets
    std::vector<Block> goals = board_.GetGoals();
    std::vector<Box> boxes_on_goals;
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();
    // free initially occupied blocks
    Pusher pusher = board_.GetPusher();
    blocks[pusher.GetX()][pusher.GetY()].Free();
    for(auto box : board_.GetBoxes()){
        blocks[box.GetX()][box.GetY()].Free();
    }
    // put new boxes on goal squares, occupy those squares
    for (auto goal : goals){
        boxes_on_goals.push_back(Box(goal.GetX(), goal.GetY(), true));
        blocks[goal.GetX()][goal.GetY()].Occupy();
    }
    BoardState initial_board = BoardState(blocks, boxes_on_goals);
    //set up list of feature cells in feature space
    std::vector<PackingPlanFeatureSpaceCell> feature_space;
    //compute initial feature space cell and add it to the list
    PackingPlanFeatureSpaceCell initial_cell = PackingPlanFeatureSpaceCell(&initial_board);
    feature_space.push_back(initial_cell);

    // set up the move "tree" -- a map (state -> weight) should be sufficient probably
    // push back the initial domain state
    std::vector<BoardState> move_tree;
    move_tree.push_back(initial_board);

    // vector to hold all states corresponding to current cell in feature space
    std::vector<BoardState*> current_states; 
    current_states = initial_cell.GetStates();
    PackingPlanFeatureSpaceCell current_cell = initial_cell;

    //custom compare function to keep track of best moves:
    //1) weight
    //2) if the box can be removed, pick this move. If more than one box can be removed, pick the one with higher distance
    //3) otherwise if a box can be moved from target, move it. Same as before, with several moves pick the one with higher distance
    //4) if 1) and 2) are not possible, just try to maximize the distance.
	auto cmp = [](std::pair<BoardState*, PackingPlanFeatureSpaceCell> left, std::pair<BoardState*, PackingPlanFeatureSpaceCell> right) { 
		if(left.first->GetWeight() > right.first->GetWeight()) return true;
        else if(left.second.GetBoxesOnBoard() < right.second.GetBoxesOnBoard()) return false; 
		else if(left.second.GetBoxesOnTarget() < right.second.GetBoxesOnTarget()) return false;
		else if(left.second.GetDistance() > right.second.GetDistance()) return false;
		return true;

	 };

    // TODO: have better definition on when the situation is solved
    //search feature space while all the boxes are not deleted
    //or the maximum number of iterations is not reached (NOT ADDED YET)
    int iteration_count = 0;
    while(current_cell.GetBoxesOnBoard() != 0 && iteration_count < 150){
        iteration_count++;
        // get all states that correspond to the current cell of the feature space
        current_states = current_cell.GetStates();
        //find all possible moves from the current states
        std::priority_queue<std::pair<BoardState*, PackingPlanFeatureSpaceCell>, std::vector<std::pair<BoardState*, PackingPlanFeatureSpaceCell>>, decltype(cmp)> possible_moves(cmp);
        
        //TODO: check that local current state and current state are used correctly
        for(auto &current_state : current_states){
            //look at moving one box at a time
            //auto parent_state = std::find(move_tree.begin(), move_tree.end(), current_state);
            unsigned int number_of_boxes = current_state->GetBoxes().size();
            for(unsigned int i = 0; i < number_of_boxes; i++){
                // moves available for i_th box only
                // tries to find all available macro moves for this box
                //keep locally explored moves
                std::vector<BoardState> explored_moves;
                bool box_removed = false;
                // TODO: should take into account if the box can be pushed in this position or if it's its starting position
                std::stack<BoardState*> available_moves;
                BoardState* current_macro_move = current_state;
                available_moves.push(current_macro_move);
                while(!available_moves.empty() && !box_removed){
                    current_macro_move = available_moves.top();
                    available_moves.pop();
                    //if the move hasn't been explored yet
                    if(std::find(explored_moves.begin(), explored_moves.end(), *current_macro_move) == explored_moves.end()){
                        // mark move as checked
                        explored_moves.push_back(*current_macro_move);
                        blocks = current_macro_move->GetBlocks();
                        int x = current_macro_move->GetBoxes()[i].GetX();
                        int y = current_macro_move->GetBoxes()[i].GetY();
                        // if a box reached a sink room, remove it from the board and stop exploration for this box
                        // for now let's assume that is the only move that would be picked by advisor
                        // TODO: but in reality need to add connectivity check
                        if(rooms_[x][y] == sink_room_){
                            std::vector<Box> new_boxes = current_macro_move->GetBoxes();
                            std::vector<std::vector<Block>> new_blocks = current_macro_move->GetBlocks();
                            new_boxes.erase(new_boxes.begin() + i);
                            new_blocks[x][y].Free();
                            current_macro_move = new BoardState(new_blocks, new_boxes, current_state, true);
                            // add it to the possible moves if it's not in the move tree already
                            if(std::find(move_tree.begin(), move_tree.end(), *current_macro_move) == move_tree.end()){
                                possible_moves.push(std::make_pair(current_macro_move, PackingPlanFeatureSpaceCell(current_macro_move)));
                            }
                            box_removed = true;
                        } else {
                            // add current move to moves possible from the current state if it's not in the move tree already
                            if(std::find(move_tree.begin(), move_tree.end(), *current_macro_move) == move_tree.end()){
                                possible_moves.push(std::make_pair(current_macro_move, PackingPlanFeatureSpaceCell(current_macro_move)));
                            }
                            //otherwise check if there is enough space to pull the box in each four directions
                            std::vector<Box> new_boxes;
                            std::vector<std::vector<Block>> new_blocks;
                            //check North
                            if(!blocks[x-1][y].IsOccupied() && !blocks[x-2][y].IsOccupied()){
                                new_boxes = current_macro_move->GetBoxes();
                                new_blocks = current_macro_move->GetBlocks();
                                new_blocks[x][y].Free();
                                new_blocks[x-1][y].Occupy();
                                new_boxes[i].Move(North);
                                available_moves.push(new BoardState(new_blocks, new_boxes, current_state));
                            }
                            //check South
                            if(!blocks[x+1][y].IsOccupied() && !blocks[x+2][y].IsOccupied()){
                                new_boxes = current_macro_move->GetBoxes();
                                new_blocks = current_macro_move->GetBlocks();
                                new_blocks[x][y].Free();
                                new_blocks[x+1][y].Occupy();
                                new_boxes[i].Move(South);
                                available_moves.push(new BoardState(new_blocks, new_boxes, current_state));
                            }
                            //check East
                            if(!blocks[x][y+1].IsOccupied() && !blocks[x][y+2].IsOccupied()){
                                new_boxes = current_macro_move->GetBoxes();
                                new_blocks = current_macro_move->GetBlocks();
                                new_blocks[x][y].Free();
                                new_blocks[x][y+1].Occupy();
                                new_boxes[i].Move(East);
                                available_moves.push(new BoardState(new_blocks, new_boxes, current_state));
                            }
                            //check West
                            if(!blocks[x][y-1].IsOccupied() && !blocks[x][y-2].IsOccupied()){
                                new_boxes = current_macro_move->GetBoxes();
                                new_blocks = current_macro_move->GetBlocks();
                                new_blocks[x][y].Free();
                                new_blocks[x][y-1].Occupy();
                                new_boxes[i].Move(West);
                                available_moves.push(new BoardState(new_blocks, new_boxes, current_state));
                            }
                        }
                    }
                }
            }
        }
        //pick the best move according to features and distance:
        // if the queue was defined correctly (hopefully) the best move is the first one in the queue of possible moves
        BoardState* best_move = possible_moves.top().first;
        PackingPlanFeatureSpaceCell best_cell = possible_moves.top().second;

        //add a move to the tree and parent connection
        move_tree.push_back(*best_move);
 
        //check if the cell in feature space was already discovered:
        //if yes, add the move to the list of the domain states that project on this cell
        //if no, add new feature cell to the feature space
        auto existing_cell = std::find(feature_space.begin(), feature_space.end(), best_cell);
        if(existing_cell != feature_space.end()){
            existing_cell->AddBoard(best_move);
        } else {
            feature_space.push_back(best_cell);
        }

        //pick a new cell
        //if the current cell is the last cell in the array, start from the beginning
        //otherwise, move on to the next cell
        auto it = std::find(feature_space.begin(), feature_space.end(), current_cell);
        if(it == feature_space.end()-1) current_cell = feature_space[0];
        else current_cell = *(it+1);
    }

    if(iteration_count == 150){
        std::cout<<"Failed to finish computations"<<std::endl;
        return;
    }
    //retrieve the path from move tree
    std::vector<BoardState> path;
    BoardState next = move_tree.back();
    while(!(next == initial_board)){
        next = next.GetPreviousState();
        path.push_back(next);
    }

    //get the order of goal squares from the path of board states
    for(auto state : path){
        for(auto goal : goals){
            int x = goal.GetX();
            int y = goal.GetY();
            auto b = state.GetBlocks();
            if(b[x][y].IsOccupied() && (std::find(packing_order_.begin(), packing_order_.end(), goal) == packing_order_.end())){
                packing_order_.push_back(goal);
            }
        }
    }
}

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

void FeatureSpaceCell::ComputeRoomConnectivity(){
    //go room by room through the upper triangle of the adjacency matrix
    //check if any edges that are supposed to be there obstructed by boxes
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();
    int broken_edges = 0;
    for(int i = 0; i < room_number_; i++){
        for(int j = i + 1; j < room_number_; j++){
            if(adjacency_[i][j]){
                std::vector<Block> room_blocks = blocks_by_room_[i];
                std::set<std::pair<int, int>> visited;
                bool reached = false;
                //check paths from all the blocks if necessary
                for(auto block : room_blocks){
                    std::pair<int, int> coord = std::make_pair(block.GetX(), block.GetY());
                    if(!reached && !block.IsOccupied() && visited.find(coord) == visited.end()){
                        std::queue<std::pair<int, int>> bfs_queue;
                        bfs_queue.push(coord);
                        while(!reached && !bfs_queue.empty()){
                            std::pair<int, int> current_block = bfs_queue.front();
                            bfs_queue.pop();
                            if(visited.find(current_block) == visited.end()){
                                visited.insert(current_block);
                                if(room_by_coord_.find(current_block) == room_by_coord_.end() || room_by_coord_[current_block] == i){
                                    std::vector<Block> neighbours = blocks[current_block.first][current_block.second].GetNeighbours();
                                    for(Block neighbour : neighbours){
                                        if( !(neighbour.GetType() == Wall || neighbour.GetType() == Outer || neighbour.IsOccupied()) ){
                                            bfs_queue.push(std::make_pair(neighbour.GetX(), neighbour.GetY()));
                                        }
                                    }
                                } else if (room_by_coord_[current_block] == j){
                                    reached = true;
                                }
                            }
                        }
                    }
                }
                if(!reached) {
                    broken_edges++;
                    std::cout<<"Broken edge between rooms "<<i<<" and "<<j<<std::endl;
                }
            }
        }
    }
    room_connectivity_ = broken_edges;
}

void FeatureSpaceCell::ComputeOutOfPlan(){
    /// TODO
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

std::map<std::pair<int,int>, int> FeatureSpaceCell::FindRooms(){
    //TODO: add that if only 1 square overlaps in two rooms, mark this square not belonging to any room
    std::map<std::pair<int,int>, int> rooms;
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();

    int current_room = 0;

    //first go through the level map with 2x3 rectangles
    for(unsigned int i = 0; i < blocks.size()-1; i++){
        for(unsigned int j = 0; j < blocks[i].size()-2; j++){
            bool one_room = true;
            std::vector<std::pair<int,int>> potential_room;

            int counter_height = 0;
            while(counter_height < 2){
                int counter_width = 0;
                while(counter_width < 3){
                    //if one of the blocks is not a floor block, exit the check for the current rectangle
                    if(blocks[i+counter_height][j+counter_width].GetType() != Floor && blocks[i+counter_height][j+counter_width].GetType() != Goal){
                        one_room = false;
                        counter_height = 2;
                        counter_width = 3;
                    } else {
                        potential_room.push_back(std::make_pair(blocks[i+counter_height][j+counter_width].GetX(), blocks[i+counter_height][j+counter_width].GetY()));
                    }
                    counter_width++;
                }
                counter_height++;
            }
            //if all blocks in rectangle are floor, mark them as one room
            if(one_room){
                //first check if two (or more) blocks from the upper row/left-most column overlap with previously found room
                //don't have to check below, because we go top down left to right
                int room = current_room;
                std::pair<int, int> coord1 = potential_room[0];
                std::pair<int, int> coord2 = potential_room[1];
                std::pair<int, int> coord3 = potential_room[2];
                std::pair<int, int> coord4 = potential_room[3];
                if(rooms.find(coord1) != rooms.end() && rooms.find(coord2) != rooms.end() && rooms[coord1] == rooms[coord2]){
                    room = rooms[coord1];
                } else if (rooms.find(coord2) != rooms.end() && rooms.find(coord3) != rooms.end() && rooms[coord2] == rooms[coord3]){
                    room = rooms[coord2];
                } else if (rooms.find(coord1) != rooms.end() && rooms.find(coord4) != rooms.end() && rooms[coord1] == rooms[coord4]){
                    room = rooms[coord1];
                }
                for(auto coordinate : potential_room){
                    rooms[coordinate] = room;
                }
                if(room == current_room) current_room++;
            }

        }
    }

    // go through the level map with 3x2 rectangles
    for(unsigned int i = 0; i < blocks.size()-2; i++){
        for(unsigned int j = 0; j < blocks[i].size()-1; j++){
            bool one_room = true;
            std::vector<std::pair<int,int>> potential_room;

            int counter_height = 0;
            while(counter_height < 3){
                int counter_width = 0;
                while(counter_width < 2){
                    //if one of the blocks is not a floor block, exit the check for the current rectangle
                    if(blocks[i+counter_height][j+counter_width].GetType() != Floor && blocks[i+counter_height][j+counter_width].GetType() != Goal){
                        one_room = false;
                        counter_height = 3;
                        counter_width = 2;
                    } else {
                        potential_room.push_back(std::make_pair(blocks[i+counter_height][j+counter_width].GetX(), blocks[i+counter_height][j+counter_width].GetY()));
                    }
                    counter_width++;
                }
                counter_height++;
            }
            //if all blocks in rectangle are floor, mark them as one room
            if(one_room){
                // save coordinates for easier checking for overlapping rooms
                int room = current_room;
                std::set<int> overlapping_rooms;

                auto end = rooms.end();
                //check by row
                for(int i = 0; i < 6; i += 2){
                    std::pair<int, int> coord1 = potential_room[i];
                    std::pair<int, int> coord2 = potential_room[i+1];
                    if(rooms.find(coord1) != end && rooms.find(coord2) != end && rooms[coord1] == rooms[coord2]){
                        overlapping_rooms.insert(rooms[coord1]);
                    }
                }
                //check pairs by columns
                for(int j = 0; j < 4; j += 2){
                    for(int i = 0; i < 2; i++){
                        std::pair<int, int> coord1 = potential_room[j+i];
                        std::pair<int, int> coord2 = potential_room[j+2+i];
                        if(rooms.find(coord1) != end && rooms.find(coord2) != end && rooms[coord1] == rooms[coord2]){
                            overlapping_rooms.insert(rooms[coord1]);
                        }
                    }
                }

                //if the resulting set has 1 room or more, update those
                if(overlapping_rooms.size() > 0){
                    //select the room with smallest number
                    room = *overlapping_rooms.begin();
                    //if there are more than one room, update it to be equal to the first room
                    for(auto it = ++overlapping_rooms.begin(); it != overlapping_rooms.end(); it++){
                        for(auto& r : rooms){
                            if(r.second == *it){
                                r.second = room;
                            }
                        }
                    }
                }

                for(auto coordinate : potential_room){
                    rooms[coordinate] = room;
                }
                if(room == current_room) current_room++;
            }

        }
    }

    for(auto room : rooms){
        blocks_by_room_[room.second].push_back(blocks[room.first.first][room.first.second]);
    }

    //adjusting correct room numbering, so there wouldn't be, for example, rooms 0,1,3 instead of 0,1,2 
    int correct_room = 0;
    for(auto& room : blocks_by_room_){
        if (room.first != correct_room){
            for(auto block : room.second){
                std::pair<int, int> coord = std::make_pair(block.GetX(), block.GetY());
                rooms[coord] = correct_room;
            }
        }
        correct_room++;
    }

    blocks_by_room_.clear();
    for(auto room : rooms){
        blocks_by_room_[room.second].push_back(blocks[room.first.first][room.first.second]);
    }

    room_by_coord_ = rooms;
    room_number_ = blocks_by_room_.size();
    return rooms;
}

std::vector<std::vector<bool>> FeatureSpaceCell::ComputeAdjacency(){
    std::vector<std::vector<bool>> adjacency(room_number_, std::vector<bool>(room_number_, false));
     
    std::vector<std::vector<Block>> blocks = board_.GetBlocks();

    //compute connectivity from the perspective of each room
    for(int i = 0; i < room_number_; i++){
        std::vector<Block> room_blocks = blocks_by_room_[i];
        std::set<std::pair<int, int>> visited;
        std::queue<std::pair<int, int>> bfs_queue;
        bfs_queue.push(std::make_pair(room_blocks[0].GetX(), room_blocks[0].GetY()));
        while(!bfs_queue.empty()){
            std::pair<int, int> current_block = bfs_queue.front();
            bfs_queue.pop();
            if(visited.find(current_block) == visited.end()){
                visited.insert(current_block);
                if(room_by_coord_.find(current_block) == room_by_coord_.end() || room_by_coord_[current_block] == i){
                    std::vector<Block> neighbours = blocks[current_block.first][current_block.second].GetNeighbours();
                    for(Block neighbour : neighbours){
                        if( !(neighbour.GetType() == Wall || neighbour.GetType() == Outer) ){
                            bfs_queue.push(std::make_pair(neighbour.GetX(), neighbour.GetY()));
                        }
                    }
                } else {
                    int j = room_by_coord_[current_block];
                    adjacency[i][j] = true;
                    adjacency[j][i] = true;
                }
            }

        }
    }

    adjacency_ = adjacency;
    return adjacency;
}

void FeatureSpaceCell::PrintRooms(){
    for(unsigned int i = 0; i < rooms_.size(); i++){
        for(unsigned int j = 0; j < rooms_[i].size(); j++){
            std::cout<<rooms_[i][j];
        }
        std::cout<<std::endl;
    }
}