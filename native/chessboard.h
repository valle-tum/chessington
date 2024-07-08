
#pragma once

#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <map>
#include <opencv2/imgproc.hpp>
#include <optional>

using namespace cv;


enum ChessboardPieceType {
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
    std::vector<std::pair<Point, ChessboardPiece>> pieces;
};


class ChessboardManager {
private:
    std::optional<std::pair<Mat, Mat>> transformation;
    std::vector<Point3f> boardCorners;
public:
    Chessboard chessboard;

    ChessboardManager();
    ~ChessboardManager();

    std::optional<std::pair<Mat, Mat>> get_transformation();
    std::vector<Point3f> get_boardCorners();
    void set_boardCorners(std::vector<std::vector<Point2f>> corners);

    void update_transformation(Mat rvec, Mat tvec);
    
    void illustrate_move(Mat frame, Point from, Point to);
};


