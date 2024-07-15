#include "chessboard.h"

#include "chess.hpp"


// Chessboard

Chessboard::Chessboard() : pieces()
{
}

Chessboard::~Chessboard()
{
}

void Chessboard::update(ChessboardUpdate &update)

{
  // add the current state of the board to the history
  piecesMean.push_back(update);
  // delete the oldest state if the history is too long
  int numFrames = 15;
  if (piecesMean.size() > numFrames)
  {
    piecesMean.erase(piecesMean.begin());
  }
  // asign the current state of the board based on the history
  if (piecesMean.size() == numFrames)
  {
    std::map<cv::Point, std::map<ChessboardPiece, int, ChessboardPieceComparator>, PointComparator> pieceCounter;
    for (auto &state : piecesMean)
    {
      for (auto &piece : state)
      {
        pieceCounter[piece.first][piece.second]++;
      }
    }
    // clear the current state of the board
    pieces.clear();    
    // add the pieces which were detected more than 5 times
    for (auto &piece : pieceCounter)
    {
      for (auto &p : piece.second)
      {
        if (p.second > (int)(numFrames / 2) - 1)
        {
          pieces.push_back(std::make_pair(piece.first, p.first));
        }
      }
    }
  }
  
}

void Chessboard::print_board()
{

  std::cout << "Current board: " << std::endl;
  for (int i = 0; i < 8; i++)
  {
    for (int j = 0; j < 8; j++)
    {
      bool found = false;
      for (auto &piece : pieces)
      {
        if (piece.first.x == j && piece.first.y == i)
        {
          found = true;
          char pieceChar = 'e';
          switch (piece.second.get_type())
          {
          case ChessboardPieceType::PAWN:
            pieceChar = piece.second.get_color() == ChessboardPieceColor::WHITE ? 'P' : 'p';
            break;
          case ChessboardPieceType::ROOK:
            pieceChar = piece.second.get_color() == ChessboardPieceColor::WHITE ? 'R' : 'r';
            break;
          case ChessboardPieceType::KNIGHT:
            pieceChar = piece.second.get_color() == ChessboardPieceColor::WHITE ? 'N' : 'n';
            break;
          case ChessboardPieceType::BISHOP:
            pieceChar = piece.second.get_color() == ChessboardPieceColor::WHITE ? 'B' : 'b';
            break;
          case ChessboardPieceType::QUEEN:
            pieceChar = piece.second.get_color() == ChessboardPieceColor::WHITE ? 'Q' : 'q';
            break;
          case ChessboardPieceType::KING:
            pieceChar = piece.second.get_color() == ChessboardPieceColor::WHITE ? 'K' : 'k';
            break;
          }
          std::cout << pieceChar << " ";
          break;
        }
      }
      if (!found)
      {
        std::cout << "e ";
      }
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

chess::Move Chessboard::update_board()
{
  using namespace chess;

  // check if there are kings for both colors, without the library to calculate legal moves will crash
  bool whiteKing = false;
  bool blackKing = false;
  for (auto &piece : pieces)
  {
    if (piece.second.get_type() == ChessboardPieceType::KING)
    {
      if (piece.second.get_color() == ChessboardPieceColor::WHITE)
      {
        whiteKing = true;
      }
      else
      {
        blackKing = true;
      }
    }
  }

  if (!whiteKing || !blackKing)
  {
    std::cerr << "Both kings must be on the board!" << std::endl;
    return Move();
  }

  // Create a chess board
  // e  means empty field
  std::vector<std::vector<char>> board(8, std::vector<char>(8, 'e'));

  // Fill the board with the pieces
  for (int i = 0; i < pieces.size(); i++)
  {
    auto piece = pieces[i];
    auto coord = piece.first;
    auto chessPiece = piece.second;

    char pieceChar = 'e';
    switch (chessPiece.get_type())
    {
    case ChessboardPieceType::PAWN:
      pieceChar = chessPiece.get_color() == ChessboardPieceColor::WHITE ? 'P' : 'p';
      break;
    case ChessboardPieceType::ROOK:
      pieceChar = chessPiece.get_color() == ChessboardPieceColor::WHITE ? 'R' : 'r';
      break;
    case ChessboardPieceType::KNIGHT:
      pieceChar = chessPiece.get_color() == ChessboardPieceColor::WHITE ? 'N' : 'n';
      break;
    case ChessboardPieceType::BISHOP:
      pieceChar = chessPiece.get_color() == ChessboardPieceColor::WHITE ? 'B' : 'b';
      break;
    case ChessboardPieceType::QUEEN:
      pieceChar = chessPiece.get_color() == ChessboardPieceColor::WHITE ? 'Q' : 'q';
      break;
    case ChessboardPieceType::KING:
      pieceChar = chessPiece.get_color() == ChessboardPieceColor::WHITE ? 'K' : 'k';
      break;
    }

    if (coord.x >= 0 && coord.x < 8 && coord.y >= 0 && coord.y < 8)
    {

      board[coord.y][coord.x] = pieceChar;
    }
  }

  // create a fen string, its a special way to describe a chees board
  std::string fen = "";
  for (int i = 0; i < 8; i++)
  {
    int empty = 0;
    for (int j = 0; j < 8; j++)
    {
      if (board[i][j] == 'e')
      {
        empty++;
      }
      else
      {
        if (empty > 0)
        {
          fen += std::to_string(empty);
          empty = 0;
        }
        fen += board[i][j];
      }
    }
    if (empty > 0)
    {
      fen += std::to_string(empty);
    }
    if (i < 7)
    {
      fen += "/";
    }
  }

  // add the turn (always white)
  fen += " w - - 0 1";

  this->board.setFen(fen);

  // print first 3 valid moves
  Movelist moves;
  movegen::legalmoves(moves, this->board);

  // std::cout << "Valid moves: " << std::endl;
  // for (int i = 7; i < 8; i++)
  // {
  //   std::cout << uci::moveToUci(moves[i]) << std::endl;
  // }

  return moves[7];
}

// ChessboardPiece

ChessboardPiece::ChessboardPiece(ChessboardPieceType type, ChessboardPieceColor color) : type(type), color(color)
{
}

ChessboardPiece::~ChessboardPiece()
{
}

ChessboardPieceType ChessboardPiece::get_type() const
{
  return type;
}

ChessboardPieceColor ChessboardPiece::get_color() const
{
  return color;
}

// ChessboardManager

ChessboardManager::ChessboardManager()
{
}

ChessboardManager::~ChessboardManager()
{
}

std::optional<std::pair<Mat, Mat>> ChessboardManager::get_transformation()
{
  return transformation;
}

std::vector<Point3f> ChessboardManager::get_boardCorners()
{
  return boardCorners;
}
