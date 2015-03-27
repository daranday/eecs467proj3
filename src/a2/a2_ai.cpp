#include <iostream>
#include <vector>
#include <string>
#include <utility>
#include <cmath>
#include "a2_ai.h"

AI::AI() {
    board = ".........";
}

// returns 0 if winner not determined, 1 if red wins, 2 if green wins, 3 if tie
bool AI::checkEnd() {
    int wins[8][3] = {{0,1,2},{3,4,5},{6,7,8},{0,3,6},{1,4,7},{2,5,8},{0,4,8},{2,4,6}};

    int redCount, greenCount;
    bool emptyCell = false;

    for(int i = 0; i < 8; i++) {
        redCount = 0;
        greenCount = 0;
        for (int j = 0; j < 3; j++) {
            if(board[wins[i][j]] == 'R') redCount++;
            if(board[wins[i][j]] == 'G') greenCount++;
            if(board[wins[i][j]] == '.') emptyCell = true;
        }

        if(redCount == 3) {
            cout << "We Win!" << endl;
            return true;
        } else if(greenCount == 3) {
            cout << "We Lost!" << endl;
            return true;
        }
    }

    if(emptyCell) {
        return false;
    } else {
        cout << "It's a tie!" << endl;
        return true;
    }
}

int AI::twoD2oneD(int x, int y) {
    return y*3 + x;
}

pair<int, int> AI::oneD2twoD(int x) {
    return pair<int, int> (abs(x % 3 - 2), abs(x/3 - 2));
}

bool AI::isValidCoord2D(int x, int y) {
    if (x < 0 || x > 2 || y < 0 || y > 2) {
        return false;
    } else {
        return true;
    }
}

bool AI::isBoardEmpty() {
    for(int i = 0; i < 9; i++) {
        if (board[i] != '.') {
            return false;
        }
    }

    return true;
}

void AI::receiveBoard(string newBoard) {
    board = newBoard;
}


void AI::aiPlay() {

    move = findNewMove();
    pair<int, int> coord = oneD2twoD(move);
    cout << "Computer played a move at " << coord.first << ", " << coord.second << '(' << move << ')' << endl;
    if(board[move] != '.') {
        cout << "spot taken. lose a turn" << endl;
        return;
    }

    board[move] = 'R';

}

void AI::oppPlay() {

    int oppMove;

    cout << "Enter move: ";
    cin >> oppMove;

    cout << "Oppenent entered: " << oppMove << endl;

    oppMove = abs(oppMove - 8);

    cout << "After conversion: " << oppMove << endl;

    pair<int, int> coord = oneD2twoD(oppMove);
    cout << "Opponent played a move at " << coord.first << ", " << coord.second << '(' << oppMove << ')' << endl;

    if(board[oppMove] != '.') {
        cout << "spot taken. lose a turn" << endl;
        return;
    }

    board[oppMove] = 'G';

}


void AI::printBoard() {

    if (board.size() != 9) {
        cout << "board size is " << board.size() << endl;
        return;
    }

    for (int i = 0; i < 9; i++) {
        cout << board[i] << ' ';
        if(i%3 == 2) {
            cout << endl;
        }
    }
}

int AI::findNewMove() {

    return abs(findNewMoveHelper() - 8);

}

int AI::findNewMoveHelper() {

    if(isBoardEmpty()) {
        // cout << "board is empty" << endl;
        return 4;
    }

    int redCount = 0, greenCount = 0;
    for(int i = 0; i < 9; i++) {
        if(board[i] == 'R') {
            redCount++;
        }
        if(board[i] == 'G') {
            greenCount++;
        }
    }

    if (redCount == 0 && greenCount == 1 && board[4] == '.') {
        return 4;
    }

    int wins[8][3] = {{0,1,2},{3,4,5},{6,7,8},{0,3,6},{1,4,7},{2,5,8},{0,4,8},{2,4,6}};
    int wins2[8][3];

    for(int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            wins2[i][j] = abs(wins[i][j] - 8);
        }
    }

    int move;
    int count;
    bool validMove;

    // having 2 red balls on a line (one more move to win)
    for(int i = 0; i < 8; i++) {
        count = 0;
        validMove = false;
        for(int j = 0; j < 3; j++) {
            if(board[wins[i][j]] == 'R') {
                count++;
            } else if (board[wins[i][j]] == '.') {
                validMove = true;
                move = wins2[i][j];
            }
        }
        if(count == 2 && validMove) {
            return move;
        }
    }

    // having 2 green balls on a line (about to lose)
    for(int i = 0; i < 8; i++) {
        count = 0;
        validMove = false;
        for(int j = 0; j < 3; j++) {
            if(board[wins[i][j]] == 'G') {
                count++;
            } else if (board[wins[i][j]] == '.') {
                validMove = true;
                move = wins2[i][j];
            }
        }
        if(count == 2 && validMove) {
            return move;
        }
    }

    vector<int> possibleMoves;
    vector<int>forks;
    int forkCount;
    for(int i = 0; i < 9; i++) {
        if(board[i] == '.') {
            possibleMoves.push_back(i);
        }
    }

    for(int i = 0; i < possibleMoves.size(); i++) {
        for(int j = 0; j < 8; j++) {
            redCount = 0;
            greenCount = 0;
            forkCount = 0;
            // cout << "--------" << endl;
            if(wins[j][0] == possibleMoves[i] || wins[j][1] == possibleMoves[i] || wins[j][2] == possibleMoves[i]) {
                // cout << "here" << endl;
                for(int k = 0; k < 3; k++) {
                    if(board[wins[j][k]] == 'R') {
                        redCount++;
                    }
                    if(board[wins[j][k]] == 'G') {
                        greenCount++;
                    }
                }

                if(redCount == 1 && greenCount == 0) {
                    forkCount++;
                }
            }
        }
        forks.push_back(forkCount);
    }

    int maxForkSoFar = 0;
    int maxForkInd;
    for(int i = 0; i < forks.size(); i++) {
        if(forks[i] > maxForkSoFar) {
            maxForkSoFar = forks[i];
            maxForkInd = possibleMoves[i];
        }
    }

    if(maxForkSoFar > 0) {
        // cout << "max fork " << maxForkSoFar << endl;
        return abs(maxForkInd - 8);
    }



    int maxSoFar = 0;
    int adjCount;
    pair<int, int> coord2D;

    int coord1D;

    int emptyCount, adjCount1, adjCount2;
    pair<int, int> coord2D1;
    pair<int, int> coord2D2;

    for(int i = 0; i < 8; i++) {
        count = 0;
        emptyCount = 0;
        validMove = false;
        for(int j = 0; j < 3; j++) {
            if(board[wins[i][j]] == 'R') {
                move = j;
                count++;
            } else if (board[wins[i][j]] == '.') {
                emptyCount++;
            }
        }

        if(count == 1 && emptyCount == 2) {
            if(move == 0 || move == 2) {
                cout << "here" << endl;
                return wins2[i][1];
            }
            if(move == 1) {
                coord2D1 = oneD2twoD(wins[i][0]);
                coord2D2 = oneD2twoD(wins[i][2]);

                adjCount1 = 0;
                adjCount2 = 0;
                for (int j = -1; j < 1; j++) {
                    for (int k = -1; k < 1; k++) {
                        if (j != 0 && k != 0) {
                            if(isValidCoord2D(coord2D1.first + j, coord2D1.second + k)) {
                                if(board[twoD2oneD(coord2D1.first + j, coord2D1.second + k)] == 'R') {
                                    adjCount1++;
                                }

                                if(board[twoD2oneD(coord2D2.first + j, coord2D2.second + k)] == 'R') {
                                    adjCount2++;
                                }

                            }

                        }
                    }
                }

                if(adjCount2 > adjCount1) {
                    // cout << "near red" << endl;
                    return wins2[i][2];
                } else if (adjCount1 > adjCount2) {
                    // cout << "near red" << endl;
                    return wins2[i][0];
                }

            }
        }
        
    }

    for(int i = 0; i < 8; i++) {
        count = 0;
        emptyCount = 0;
        validMove = false;
        for(int j = 0; j < 3; j++) {
            if(board[wins[i][j]] == 'G') {
                move = j;
                count++;
            } else if (board[wins[i][j]] == '.') {
                emptyCount++;
            }
        }

        if(count == 1 && emptyCount == 2) {
            if(move == 0 || move == 2) {
                // cout << "here" << endl;
                return wins2[i][1];
            }
            if(move == 1) {
                coord2D1 = oneD2twoD(wins[i][0]);
                coord2D2 = oneD2twoD(wins[i][2]);

                adjCount1 = 0;
                adjCount2 = 0;
                for (int j = -1; j < 1; j++) {
                    for (int k = -1; k < 1; k++) {
                        if (j != 0 && k != 0) {
                            if(isValidCoord2D(coord2D1.first + j, coord2D1.second + k)) {
                                if(board[twoD2oneD(coord2D1.first + j, coord2D1.second + k)] == 'G') {
                                    adjCount1++;
                                }

                                if(board[twoD2oneD(coord2D2.first + j, coord2D2.second + k)] == 'G') {
                                    adjCount2++;
                                }

                            }

                        }
                    }
                }

                if(adjCount2 > adjCount1) {
                    // cout << "near green" << endl;
                    return wins2[i][2];
                } else if (adjCount1 > adjCount2) {
                    // cout << "near green" << endl;
                    return wins2[i][0];
                }

            }
        }
        
    }

    for (int i = 0; i < 9; i++) {
        if (board[i] == '.') {
            // cout << "first empty" << endl;
            return abs(i - 8);
        }
    }


    return -1;

}

// int main() {

//     AI A;
//     A.printBoard();

//     cout << "game starts" << endl;
//     while(1) {
//         if(A.checkEnd()) {
//             return 0;
//         }
//         // A.aiPlay();
//         A.oppPlay();

//         A.printBoard();
//         if(A.checkEnd()) {
//             return 0;
//         }

//         // The following three lines will be replaced by receiving a new board
//         // A.oppPlay();
//         A.aiPlay();
//         A.printBoard();
//     }

//     return 0;
// }