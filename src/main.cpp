#include <chrono>
#include <iostream>
#include <string>

#include "BoardState.hpp"
#include "FeatureSpaceCell.hpp"

int main(int argc, char *argv[])
{
	BoardState testBoard = BoardState();
	std::cout<<"Loading file. "<<std::endl;

	std::string filename = "../src/levels/xsokoban/screen.1";
	
	//takes filename as the first argument
	//if none were passed, tries to load the first level located at "./src/levels/xsokoban/screen.1"
	if(argc>1){
		filename = argv[1];
	}
	
	testBoard.ReadBoardState(filename);
	std::cout<<"Printing Board: "<<std::endl;
	testBoard.PrintBoardState();
	std::cout<<"Total number of rows: "<<testBoard.GetRows()<<", total number of columns: "<<testBoard.GetColumns()<<std::endl;
	testBoard.AddNeighbours();
	FeatureSpaceCell testSpace = FeatureSpaceCell(testBoard);
	testSpace.ComputeConnectivity();
	std::cout<<"The current number of disconnected rooms is: "<<testSpace.GetConnectivity()<<std::endl;
	std::cout<<"The current number of boxes blocking tunnels is: "<<testSpace.GetRoomConnectivity()<<std::endl;
	testSpace.FindSinkRoom();
	testSpace.PrintRooms();
}