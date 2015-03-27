#ifndef __A2_AI__
#define __A2_AI__

using namespace std;

class AI {
public:
    AI();
    string board;
    int move;
    int findNewMove();
    int findNewMoveHelper();
    void printBoard();
    void aiPlay();
    void oppPlay();
    void receiveBoard(string newBoard);
    bool isBoardEmpty();
    int twoD2oneD(int x, int y);
    pair<int, int> oneD2twoD(int x);
    bool isValidCoord2D(int x, int y);
    bool checkEnd();

};

#endif