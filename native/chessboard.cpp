#include "chessboard.h"

// ChessboardPiece

ChessboardPiece::ChessboardPiece(ChessboardPieceType type, ChessboardPieceColor color) : type(type), color(color) {
  
}

ChessboardPiece::~ChessboardPiece() {
  
}

ChessboardPieceType ChessboardPiece::get_type() {
  return type;
}

ChessboardPieceColor ChessboardPiece::get_color() {
  return color;
}

// ChessboardManager

ChessboardManager::ChessboardManager() {
  
}

ChessboardManager::~ChessboardManager() {
  
}


    std::optional<std::pair<Mat, Mat>> ChessboardManager::get_transformation() 
    {
  return transformation;
    }

void ChessboardManager::update_transformation(Mat rvec, Mat tvec)
{
  this->transformation = std::make_pair(rvec, tvec);
}

void ChessboardManager::move_piece(Mat frame, ChessboardPiece piece, Point to) {
  // TODO
}

void ChessboardManager::move_piece(Mat frame, ChessboardPiece piece, Point from, Point to) {
  // TODO
}

void ChessboardManager::move_piece(Mat frame, Point from, Point to) {
  ChessboardManager::illustrate_move(frame, from, to);
}



// coordinate systems:
// - chessboard: 0,0 is the top left corner of the chessboard -> x and y are in the range [0, 7]
// - object: object coordinate system of opencv (3d coordinates) -> x and y are in the range [0, 1] and z is always 0
// - pixel: pixel coordinate system of opencv -> x and y are in the range [0, frame.cols] and [0, frame.rows]
// transform defines the transition from object to pixel coordinates

// HELPER FUNCTIONS TO TRANSITION BETWEEN ALL COORDINATE SYSTEMS
// USING OPENCV TYPES

using Transformation = std::pair<Mat, Mat>;
using ChessboardCoordinate = Point;
using ObjectCoordinate = Point3f;
using PixelCoordinate = Point;

ObjectCoordinate chessboardToObject(ChessboardCoordinate chessCoord) {
    return ObjectCoordinate(chessCoord.x / 8.0f, chessCoord.y / 8.0f, 0.0f);
}

PixelCoordinate objectToPixel(ObjectCoordinate objCoord, const Transformation& transformation) {
    cv::Mat R = transformation.first;
    cv::Mat t = transformation.second;

    // Convert object coordinates to homogeneous coordinates
    cv::Mat objPoint = (cv::Mat_<double>(3, 1) << objCoord.x, objCoord.y, objCoord.z);
    cv::Mat pixelPoint = R * objPoint + t;

    return PixelCoordinate(pixelPoint.at<double>(0), pixelPoint.at<double>(1));
}

PixelCoordinate chessboardToPixel(ChessboardCoordinate chessCoord, const Transformation& transformation) {
    ObjectCoordinate objCoord = chessboardToObject(chessCoord);
    return objectToPixel(objCoord, transformation);
}

ObjectCoordinate pixelToObject(PixelCoordinate pixelCoord, const Transformation& transformation) {
    cv::Mat R = transformation.first;
    cv::Mat t = transformation.second;

    // Invert the rotation matrix
    cv::Mat R_inv = R.t();

    // Compute the inverse translation vector
    cv::Mat t_inv = -R_inv * t;

    // Convert pixel coordinates to homogeneous coordinates
    cv::Mat pixelPoint = (cv::Mat_<double>(3, 1) << pixelCoord.x, pixelCoord.y, 1.0);
    cv::Mat objPoint = R_inv * pixelPoint + t_inv;

    return ObjectCoordinate(objPoint.at<double>(0), objPoint.at<double>(1), objPoint.at<double>(2));
}

ChessboardCoordinate objectToChessboard(ObjectCoordinate objCoord) {
    // Assuming each square is of size 1/8 in object coordinates
    return ChessboardCoordinate(static_cast<int>(objCoord.x * 8), static_cast<int>(objCoord.y * 8));
}

ChessboardCoordinate pixelToChessboard(PixelCoordinate pixelCoord, const Transformation& transformation) {
    ObjectCoordinate objCoord = pixelToObject(pixelCoord, transformation);
    return objectToChessboard(objCoord);
}

// from and to are in the chessboard coordinate system
// so x and y are in the range [0, 7]
void ChessboardManager::illustrate_move(Mat frame, ChessboardCoordinate from, ChessboardCoordinate to) {
  // check if transformation is set
  if (!this->transformation.has_value()) {
    std::cerr << "Transformation is not set!" << std::endl;
    return;
  }

  // convert chessboard coordinates to pixel coordinates
  PixelCoordinate fromPixel = chessboardToPixel(from, transformation.value());
  PixelCoordinate toPixel = chessboardToPixel(to, transformation.value());

  std::cout << "From: " << from << " -> " << fromPixel << std::endl;
  std::cout << "To: " << to << " -> " << toPixel << std::endl;

  // draw arrowed line
  arrowedLine(frame, fromPixel, toPixel, Scalar(0, 0, 255), 3, 8, 0, 0.2);
}

// Chessboard

Chessboard::Chessboard() : pieces() {
}

Chessboard::~Chessboard() {
  
}

void Chessboard::update(const ChessboardUpdate& update) {
  for (auto& piece : update) {
    // Check if the piece is already on the board
    bool found = false;
    for (auto& p : pieces) {
      if (p.first == piece.first) {
        p.second = piece.second;
        found = true;
        break;
      }
    }
    if (!found) {
      pieces.push_back(piece);
    }
  }
}