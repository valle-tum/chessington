
#pragma once

#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <map>
#include <opencv2/imgproc.hpp>
#include <optional>
#include "chess.hpp"
#include <future>

using namespace cv;

enum ChessboardPieceType
{
    PAWN = 1,
    ROOK = 2,
    KNIGHT = 3,
    BISHOP = 4,
    QUEEN = 5,
    KING = 6
};

enum ChessboardPieceColor
{
    WHITE = 0,
    BLACK = 1
};

class ChessboardPiece
{
private:
    ChessboardPieceType type;
    ChessboardPieceColor color;

public:
    ChessboardPiece(ChessboardPieceType type, ChessboardPieceColor color);
    ~ChessboardPiece();

    ChessboardPieceType get_type() const;
    ChessboardPieceColor get_color() const;
};

using ChessboardUpdate = std::vector<std::pair<Point, ChessboardPiece>>;

class Chessboard
{
public:
    Chessboard();
    ~Chessboard();

    void update(ChessboardUpdate &update);

    void print_board();

    std::string update_board();

private:
    chess::Board board;
    std::vector<std::pair<Point, ChessboardPiece>> pieces;
    std::vector<std::vector<std::pair<Point, ChessboardPiece>>> piecesMean;
    int counterFrames;
    std::string bestMove;
};

struct PointComparator
{
    bool operator()(const cv::Point &a, const cv::Point &b) const
    {
        if (a.x == b.x)
            return a.y < b.y;
        return a.x < b.x;
    }
};

struct ChessboardPieceComparator
{
    bool operator()(const ChessboardPiece &a, const ChessboardPiece &b) const
    {
        // Assuming ChessboardPiece has methods get_type() and get_color() for comparison
        if (a.get_type() == b.get_type())
            return a.get_color() < b.get_color();
        return a.get_type() < b.get_type();
    }
};

class ChessboardManager
{
private:
    std::optional<std::pair<Mat, Mat>> transformation;
    std::vector<Point3f> boardCorners;

public:
    Chessboard chessboard;

    ChessboardManager();
    ~ChessboardManager();

    std::optional<std::pair<Mat, Mat>> get_transformation();
    std::vector<Point3f> get_boardCorners();
};

class RuedigerDestroyerOfWorlds
{
public:
    RuedigerDestroyerOfWorlds();
    ~RuedigerDestroyerOfWorlds();

public:
    std::future<std::string> getBestMoveAsync(const std::string &fen);
    std::string getBestMove(const std::string &fen);
};
