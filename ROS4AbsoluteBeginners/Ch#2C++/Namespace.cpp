#include <iostream>
#include <string>

using namespace std;

namespace robot {
    void process() {
        cout<<"Processing from the robot namespace."<<endl;
    }
}

namespace machine {
    void process() {
        cout<<"Processing from the machine namespace."<<endl;
    }
}


int main() {
    robot::process();
    machine::process();
}