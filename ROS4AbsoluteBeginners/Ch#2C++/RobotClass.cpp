#include <iostream>

using namespace std;

class RobotClass {
    public:
    int id;
    int wheelsNum;
    string name;
    void move();
    void greet();    
};

void RobotClass::move() {
    cout<<RobotClass::name<<" is moving..."<<endl;
}
void RobotClass::greet() {
    cout<<RobotClass::name<<" 🤖 greets you"<<endl;
}

int main() {
    // RobotClass myRobot;
    // myRobot.id = 231;
    // myRobot.wheelsNum = 0;
    // myRobot.name = "Humanoid";
    // myRobot.greet();
    // myRobot.move();
    RobotClass *newBot;
    newBot = new RobotClass;
    newBot->name = "Dr_Strange";
    newBot->id = 23;
    newBot->move();
    newBot->greet();
}
