#include <iostream>
#include <string>

using namespace std;

int main() {
    int a = 1;
    int b = 0;
    try {
        if (b==0) {
            throw b;
        }
    } catch (int e) {
        cout<<"Exception found: "<<e<<endl;
    }

    return 0;
}