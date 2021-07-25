#include "Object.hpp"

void Object::Move(Direction dir){
    switch(dir){
        case North: yCoordinate_++; break;
        case South: yCoordinate_--; break;
        case East: xCoordinate_++; break;
        case West: xCoordinate_--; break;
    }
}

bool operator==(const Object& o1, const Object& o2){
    return ((o1.GetX() == o2.GetX()) && (o1.GetY() == o2.GetY()));
}