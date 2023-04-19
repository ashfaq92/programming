class Robot:
    def __init__(self):
        print('Started robot')
    def move_forward(self, distance):
        print('moving forward', distance, 'm')
    def move_backward(self, distance):
        print('moving backward', distance, 'm')
    def move_right(self, distance):
        print('moving right', distance, 'm')
    def move_left(self, distance):
        print('moving left', distance, 'm')
    def __del__(self):
        print("Robot stopped.")


def main():
    myRob = Robot()
    myRob.move_backward(5)
    myRob.move_forward(2)
    myRob.move_right(8)
    myRob.move_left(1)

if __name__ == "__main__":
    main()