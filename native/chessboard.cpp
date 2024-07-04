#include "chessboard.h"

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