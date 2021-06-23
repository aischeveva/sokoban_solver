#include <chrono>
#include <iostream>

#include "Board.hpp"
#include "FeatureSpace.hpp"

int main(int argc, char *argv[])
{
	Board testBoard = Board();
	std::cout<<"Loading file. "<<std::endl;
	testBoard.ReadBoard(".\\src\\levels\\xsokoban\\screen.1");
	std::cout<<"Printing Board: "<<std::endl;
	testBoard.PrintBoard();
	std::cout<<"Total number of rows: "<<testBoard.GetRows()<<", total number of columns: "<<testBoard.GetColumns()<<std::endl;
	testBoard.AddNeighbours();
	FeatureSpace testSpace = FeatureSpace(testBoard);
	testSpace.ComputeConnectivity();
	std::cout<<"The current number of disconnected rooms is: "<<testSpace.GetConnectivity()<<std::endl;
}