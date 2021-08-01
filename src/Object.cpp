#include "Object.hpp"

void Object::Move(Direction dir){
    switch(dir){
        case North: xCoordinate_--; break;
        case South: xCoordinate_++; break;
        case East: yCoordinate_++; break;
        case West: yCoordinate_--; break;
    }
}

bool operator==(const Object& o1, const Object& o2){
    return ((o1.GetX() == o2.GetX()) && (o1.GetY() == o2.GetY()));
}