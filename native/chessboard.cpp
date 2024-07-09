#include "chessboard.h"

#include "chess.hpp"

// coordinate systems:
// - chessboard: 0,0 is the bottom left corner of the chessboard -> x and y are in the range [0, 7]
// - object: object coordinate system of opencv (3d coordinates) -> x and y are in the range [0, 1] and z is always 0
// - pixel: pixel coordinate system of opencv -> x and y are in the range [0, frame.cols] and [0, frame.rows]
// transform defines the transition from object to pixel coordinates


// HELPER FUNCTIONS TO TRANSITION BETWEEN ALL COORDINATE SYSTEMS
// USING OPENCV TYPES

using Transformation = std::pair<Mat, Mat>;
using ChessboardCoordinate = Point;
using ObjectCoordinate = Point3f;
using PixelCoordinate = Point2f;

ObjectCoordinate chessboardToObject(ChessboardCoordinate chessCoord, std::vector<Point3f> boardCorners)
{
  float spanX = abs(boardCorners[1].x - boardCorners[0].x);
  float spanY = abs(boardCorners[3].y - boardCorners[0].y);
  std::cout << "SpanX: " << spanX << std::endl; 
  std::cout << "SpanY: " << spanY << std::endl;
  return ObjectCoordinate(boardCorners[0].x + chessCoord.x * spanX, boardCorners[0].y + chessCoord.y * spanY, 0);
}

PixelCoordinate objectToPixel(ObjectCoordinate objCoord, const Transformation &transformation)
{
  cv::Mat R = transformation.first;
  cv::Mat t = transformation.second;

  // Convert object coordinates to homogeneous coordinates
  cv::Mat objPoint = (cv::Mat_<double>(3, 1) << objCoord.x, objCoord.y, objCoord.z);
  cv::Mat pixelPoint = R * objPoint + t;

  return PixelCoordinate(pixelPoint.at<double>(0), pixelPoint.at<double>(1));
}

PixelCoordinate chessboardToPixel(ChessboardCoordinate chessCoord, const Transformation &transformation, std::vector<Point3f> boardCorners)
{
  ObjectCoordinate objCoord = chessboardToObject(chessCoord, boardCorners);
  return objectToPixel(objCoord, transformation);
}

ObjectCoordinate pixelToObject(PixelCoordinate pixelCoord, const Transformation &transformation)
{
  cv::Mat R = transformation.first;
  cv::Mat t = transformation.second;

  // Invert the rotation matrix
  cv::Mat R_inv = R.t(); // verify

  // Compute the inverse translation vector
  cv::Mat t_inv = -R_inv * t;

  // Convert pixel coordinates to homogeneous coordinates
  cv::Mat pixelPoint = (cv::Mat_<double>(3, 1) << pixelCoord.x, pixelCoord.y, 1.0);
  cv::Mat objPoint = R_inv * pixelPoint + t_inv;

  return ObjectCoordinate(objPoint.at<double>(0), objPoint.at<double>(1), objPoint.at<double>(2));
}

ChessboardCoordinate objectToChessboard(ObjectCoordinate objCoord)
{
  // Assuming each square is of size 1/8 in object coordinates
  return ChessboardCoordinate(static_cast<int>(objCoord.x * 8), static_cast<int>(objCoord.y * 8));
}

ChessboardCoordinate pixelToChessboard(PixelCoordinate pixelCoord, const Transformation &transformation)
{
  ObjectCoordinate objCoord = pixelToObject(pixelCoord, transformation);
  return objectToChessboard(objCoord);
}

// from and to are in the chessboard coordinate system
// so x and y are in the range [0, 7]
void ChessboardManager::illustrate_move(Mat frame, ChessboardCoordinate from, ChessboardCoordinate to)
{
  // check if transformation is set
  if (!this->transformation.has_value())
  {
    std::cerr << "Transformation is not set!" << std::endl;
    return;
  }

  if (this->boardCorners.size() < 4)
  {
    std::cerr << "Board corners are not set!" << std::endl;
    return;
  }

  // convert chessboard coordinates to pixel coordinates
  PixelCoordinate fromPixel = chessboardToPixel(from, transformation.value(), this->boardCorners);
  PixelCoordinate toPixel = chessboardToPixel(to, transformation.value(), this->boardCorners);

  std::cout << "From: " << from << " -> " << fromPixel << std::endl;
  std::cout << "To: " << to << " -> " << toPixel << std::endl;

  // draw arrowed line
  arrowedLine(frame, fromPixel, toPixel, Scalar(0, 0, 255), 3, 8, 0, 0.2);
}

// Chessboard

Chessboard::Chessboard() : pieces()
{
}

Chessboard::~Chessboard()
{
}

void Chessboard::update(ChessboardUpdate &update)
{
  auto has_changed = true;
  for (auto &piece : update)
  {
    // Check if the piece is already on the board
    bool found = false;
    for (auto &p : pieces)
    {
      if (p.first == piece.first)
      {
        if (p.second.get_type() != piece.second.get_type() || p.second.get_color() != piece.second.get_color())
        {
          p.second = piece.second;
          found = true;
          has_changed = true;
          break;
        }
      }
    }
    if (!found)
    {
      pieces.push_back(piece);
      has_changed = true;
    }
  }

  if (has_changed)
  {
    update_board();
  }
}

void Chessboard::update_board()
{
  using namespace chess;

  // Create a chess board
  // e  means empty
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

    board[coord.y][coord.x] = pieceChar;
  }

  // create a fen string
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

  std::cout << "Valid moves: " << std::endl;
  for (int i = 0; i < 3; i++)
  {
    std::cout << uci::moveToUci(moves[i]) << std::endl;
  }
  std::cout << "Waiting for change..." << std::endl;

}

// ChessboardPiece

ChessboardPiece::ChessboardPiece(ChessboardPieceType type, ChessboardPieceColor color) : type(type), color(color)
{
}

ChessboardPiece::~ChessboardPiece()
{
}

ChessboardPieceType ChessboardPiece::get_type()
{
  return type;
}

ChessboardPieceColor ChessboardPiece::get_color()
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

void ChessboardManager::set_boardCorners(std::vector<std::vector<Point2f>> corners)
{
  // by the definition of the physical chessboard the markers with id 0,2,3 and 5 are in the corners
  std::vector<std::vector<Point2f>> edgeMarker;
  edgeMarker.push_back(corners[0]);
  edgeMarker.push_back(corners[2]);
  edgeMarker.push_back(corners[3]);
  edgeMarker.push_back(corners[5]);

  std::cout << "edgeMarker: " << edgeMarker[0] << std::endl;

  // transform the 2d corners to 3d corners
  std::vector<std::vector<Point3f>> edgeMarker3d;
  std::vector<Point3f> corner3d;
  for (auto marker : edgeMarker)
  {
    for (auto corner : marker)
    {
      if (!this->transformation.has_value())
        break;
      corner3d.push_back(pixelToObject(corner, this->transformation.value()));
    }
    edgeMarker3d.push_back(corner3d);
  }

  std::cout << "edgeMarker3d: " << edgeMarker3d[0] << std::endl;


  // find center of the chessboard
  Point3f center(0, 0, 0);
  for (auto marker : edgeMarker3d)
  {
    for (auto corner : marker)
    {
      center += corner;
    }
  }
  center /= (int)(edgeMarker3d.size() * 4);
  std::cout << "center: " << center << std::endl;



  
  // find the the corner which is the closest to the center for each marker
  std::vector<Point3f> sortedCorners;
  for (auto marker : edgeMarker3d)
  {
    float minDistance = std::numeric_limits<float>::max();
    Point3f closestCorner;
    for (auto corner : marker)
    {
      float distance = norm(corner - center);
      if (distance < minDistance)
      {
        minDistance = distance;
        closestCorner = corner;
      }
    }
    sortedCorners.push_back(closestCorner);
  }

  std::cout << "boardCorners: " << sortedCorners << std::endl;
  std::cout << std::endl;

  this->boardCorners = sortedCorners;
}

void ChessboardManager::update_transformation(Mat rvec, Mat tvec)
{
  this->transformation = std::make_pair(rvec, tvec);
}
