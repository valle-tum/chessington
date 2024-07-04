
#pragma once

#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <map>

using namespace cv;


enum ChessboardPieceType {
    EMPTY = 0,
    PAWN = 1,
    ROOK = 2,
    KNIGHT = 3,
    BISHOP = 4,
    QUEEN = 5,
    KING = 6
};

enum ChessboardPieceColor {
    WHITE = 0,
    BLACK = 1
};

class ChessboardPiece {
private:
    ChessboardPieceType type;
    ChessboardPieceColor color;
public:
    ChessboardPiece(ChessboardPieceType type, ChessboardPieceColor color);
    ~ChessboardPiece();

    ChessboardPieceType get_type();
    ChessboardPieceColor get_color();
};

using ChessboardUpdate = std::vector<std::pair<Point, ChessboardPiece>>;

class Chessboard
{
public:
    Chessboard();
    ~Chessboard();

    void update(const ChessboardUpdate& update);

private:
    Size board_size = Size(2,3);
    float square_size = 0.025;

    std::vector<std::pair<Point, ChessboardPiece>> pieces;
};