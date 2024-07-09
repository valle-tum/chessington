
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <map>

#include "camera.h"
#include <opencv2/calib3d.hpp>
#include "chessboard.h"

#define WEBCAM 1
/* WEBCAM 1 --> portable Webcam (Valentin)
 * WEBCAM 0 --> integrated Webcam (Valentin)
 */

void transformImagePointsToObjectPoints(const std::vector<cv::Point2f> &imagePoints,
                                        const cv::Matx33f &cameraMatrix,
                                        const cv::Mat &distCoeffs,
                                        const cv::Mat &rVec,
                                        const cv::Mat &tVec,
                                        std::vector<cv::Point3f> &objectPoints)
{
    // Convert rotation vector to rotation matrix
    cv::Mat R;
    cv::Rodrigues(rVec, R);

    // Invert the rotation matrix
    cv::Mat R_inv = R.inv();

    // Compute the inverse translation vector
    cv::Mat tVec_inv = -R_inv * tVec;

    // Undistort image points
    std::vector<cv::Point2f> undistortedPoints = imagePoints;
    // cv::undistortPoints(imagePoints, undistortedPoints, cameraMatrix, distCoeffs);

    // Transform points from image space to object space
    for (const auto &pt : undistortedPoints)
    {
        cv::Mat uvPoint = (cv::Mat_<double>(3, 1) << pt.x, pt.y, 1.0);

        cv::Mat leftSideMat = R.inv() * cameraMatrix.inv() * uvPoint;
        cv::Mat rightSideMat = R.inv() * tVec;

        double s = (285 + rightSideMat.at<double>(2, 0)) / leftSideMat.at<double>(2, 0);
        // 285 represents the height Zconst

        std::cout << "P = " << R.inv() * (s * cameraMatrix.inv() * uvPoint - tVec) << std::endl;

        cv::Mat objectPoint = R_inv * uvPoint + tVec_inv;
        objectPoints.emplace_back(objectPoint.at<double>(0), objectPoint.at<double>(1), objectPoint.at<double>(2));
    }
}

Camera::Camera(int input) : fps(30), flip_lr(false), flip_ud(false)
{
    capture.open(input);
    // capture.open("MarkerMovie.MP4");

    if (!capture.isOpened())
    {
        std::cout << "No webcam, using video file" << std::endl;
        capture.open("MarkerMovie.MP4");
        if (!capture.isOpened())
        {
            capture.release();
            throw std::runtime_error("Unable to open camera");
        }
    }

    width = (int)capture.get(CAP_PROP_FRAME_WIDTH);
    height = (int)capture.get(CAP_PROP_FRAME_HEIGHT);

    std::cout << "Camera ready (" << width << "x" << height << ")" << std::endl;

    worker = std::thread(&Camera::loop, this);
}

Camera::~Camera()
{
    {
        std::cout << "Closing camera" << std::endl;

        std::lock_guard<std::recursive_mutex> lock(guard);
        capture.release();
    }

    worker.join();
}

Mat Camera::getFrame(int display_option)
{
    std::lock_guard<std::recursive_mutex> lock(guard);

    if (display_option == DISPLAY_OPTION_RBG)
    {
        return frame;
    }
    // else if (display_option == DISPLAY_OPTION_GREYSCALE) {
    //     return greyscale;
    // }
    else
    {
        throw std::runtime_error("Unable to open camera");
    }

    return cv::Mat();
}

unsigned long Camera::getFrameNumber()
{
    std::lock_guard<std::recursive_mutex> lock(guard);

    return counter;
}

Mat Camera::getFrameIfNewer(unsigned long &current, int display_frame_option)
{
    std::lock_guard<std::recursive_mutex> lock(guard);

    if (current == counter)
        return Mat();

    current = counter;

    if (display_frame_option == DISPLAY_OPTION_RBG)
    {
        return frame;
    }
    // else if (display_frame_option == DISPLAY_OPTION_GREYSCALE) {
    //     return greyscale;
    // }
    else
    {
        throw std::runtime_error("Unable to open camera");
    }

    return cv::Mat();
}

int Camera::getWidth()
{
    return width;
}

int Camera::getHeight()
{
    return height;
}

int Camera::flip(bool flip_lr, bool flip_ud)
{
    this->flip_lr = flip_lr;
    this->flip_ud = flip_ud;

    return 1;
}

void Camera::setThreshold(int value = 0)
{
    this->threshold = value;
}

void Camera::loop()
{

    ChessboardManager manager;
    std::optional<std::pair<cv::Point2f, cv::Point2f>>  move;
    while (true)
    {
        auto start = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::recursive_mutex> lock(guard);

            capture.read(frame);

            if (frame.empty())
            {
                break;
            }

            this->flip(false, false);
            if (flip_lr || flip_ud)
            {
                int code = flip_lr ? (flip_ud ? -1 : 1) : 0;
                code = -1;
                cv::flip(frame, frame, code);
            }

            auto new_move = updateChessModel(&frame, &frame, &manager);
            if (new_move.has_value())
            {
                move = new_move;
            }
            if (move.has_value()) {
                cv::arrowedLine(frame, move.value().first, move.value().second, cv::Scalar(0, 0, 255), 8);
            }
             
            counter++;
        }

        auto end = std::chrono::high_resolution_clock::now();

        auto used = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        auto remaining = std::chrono::milliseconds(std::max(1l, (long)((1000.0 / fps) - used)));

        std::this_thread::sleep_for(remaining);
    }
}

std::optional<std::pair<cv::Point2f, cv::Point2f>>  updateChessModel(cv::Mat *frameIn, cv::Mat *frameOut, ChessboardManager *manager)
{

    // Create detector and dictionary

    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);

    aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
    detectorParams.minDistanceToBorder = 0;
    detectorParams.adaptiveThreshWinSizeStep = 100;

    aruco::ArucoDetector detector(dictionary, detectorParams);

    // Create GridBoard object
    int markersX = 2;            // number of markers in X
    int markersY = 2;            // number of markers in Y
    float markerLength = 4;      // marker length
    float markerSeparation = 40; // separation between markers
    aruco::GridBoard board(Size(markersX, markersY), markerLength, markerSeparation, dictionary);

    // Length of the axis in the board
    // float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
    //                            markerSeparation);
    float axisLength = 48.0f;

    // Detect markers
    std::vector<int> ids;
    std::vector<std::vector<Point2f>> corners, rejected;
    detector.detectMarkers(*frameIn, corners, ids, rejected);

    // filter ids and corners to only keep ids that are part of the board
    const std::vector<int> custom_ids = {0, 1, 2, 3}; // {100, 101, 102, 103, 104, 105};
    auto marker_to_piece = [](int id) -> ChessboardPiece
    {
        // Marker ids start with 10

        // White Rook 8 and 9
        // White Knight 12 and 13
        // White Bishop 14 and 15
        // White Queen 16
        // White King 17
        // White Pawn 50 - 57

        // Black Rook 24 and 26
        // Black Knight 28 and 32
        // Black Bishop 27 and 29
        // Black Queen 31
        // Black King 33
        // Black Pawn 18 - 23 and 25 and 30

        // white pawns 50 - 57 und irgendwo nen fehler

        auto id_without_offset = id - 0;

        auto type_id = id_without_offset;
        std::map<int, ChessboardPieceType> pieceTypeMap = {
            {8, ChessboardPieceType::ROOK},
            {9, ChessboardPieceType::ROOK},
            {12, ChessboardPieceType::KNIGHT},
            {13, ChessboardPieceType::KNIGHT},
            {14, ChessboardPieceType::BISHOP},
            {15, ChessboardPieceType::BISHOP},
            {16, ChessboardPieceType::QUEEN},
            {17, ChessboardPieceType::KING},
            {24, ChessboardPieceType::ROOK},
            {26, ChessboardPieceType::ROOK},
            {28, ChessboardPieceType::KNIGHT},
            {32, ChessboardPieceType::KNIGHT},
            {27, ChessboardPieceType::BISHOP},
            {29, ChessboardPieceType::BISHOP},
            {33, ChessboardPieceType::QUEEN},
            {31, ChessboardPieceType::KING},
        };

        for (int i = 50; i <= 57; i++)
        {
            pieceTypeMap[i] = ChessboardPieceType::PAWN;
        }

        for (int i = 18; i <= 23; i++)
        {
            pieceTypeMap[i] = ChessboardPieceType::PAWN;
        }

        pieceTypeMap[25] = ChessboardPieceType::PAWN;
        pieceTypeMap[30] = ChessboardPieceType::PAWN;

        std::map<int, ChessboardPieceColor> pieceColorMap;

        // White pieces
        for (int i = 8; i <= 17; i++)
        {
            pieceColorMap[i] = ChessboardPieceColor::WHITE;
        }
        for (int i = 50; i <= 57; i++)
        {
            pieceColorMap[i] = ChessboardPieceColor::WHITE;
        }

        // Black pieces
        for (int i = 18; i <= 33; i++)
        {
            if (i != 24)
            { // 24 is not included in the black pieces
                pieceColorMap[i] = ChessboardPieceColor::BLACK;
            }
        }
        pieceColorMap[24] = ChessboardPieceColor::BLACK;
        pieceColorMap[26] = ChessboardPieceColor::BLACK;

        ChessboardPieceType type = pieceTypeMap[id_without_offset];
        auto color = pieceColorMap[id_without_offset];

        return ChessboardPiece(type, color);
    };

    std::vector<int> board_ids, piece_ids;
    std::vector<std::vector<Point2f>> board_corners, piece_corners;
    auto use_all = false;
    for (size_t i = 0; i < ids.size(); i++)
    {
        if (use_all || std::find(custom_ids.begin(), custom_ids.end(), ids[i]) != custom_ids.end())
        {
            board_ids.push_back(ids[i]);
            board_corners.push_back(corners[i]);
        }
        else
        {
            piece_ids.push_back(ids[i]);
            piece_corners.push_back(corners[i]);
        }
    }

    // Update the board corners for the ChessboardManager
    // if (board_corners.size() == 4)
    // {
    //     manager->set_boardCorners(board_corners);
    // }

    // Draw results
    bool showBoard = false;
    if (!board_ids.empty() && showBoard)
        aruco::drawDetectedMarkers(*frameOut, board_corners, board_ids);

    bool showPieces = false;
    if (!piece_ids.empty() && showPieces)
        aruco::drawDetectedMarkers(*frameOut, piece_corners, piece_ids);
    // std::cout << "Markers detected (id): " << ids.size() << std::endl;

    // Draw rejected markers
    bool showRejected = false;
    if (showRejected && !rejected.empty())
        aruco::drawDetectedMarkers(*frameOut, rejected, noArray(), Scalar(100, 0, 255));

    // Calibrate the camera
    bool calibrate = false;
    if (calibrate)
    {
        // f[px] = x[px] * z[m] / x[m]
        // get the size of one marker in pixels using the detected corners. Also use an inline if statement to avoid division by zero
        float markerSizePixel = (ids.size() > 0) ? (float)cv::norm(corners[0][0] - corners[0][1]) : 1.0;
        // std::cout << "markerSizePixel: " << markerSizePixel << std::endl;
        // float markerSizePixel = 73.0;   // for WEBCAM 0 (Valentin) --> focalLen = 522.83783
        // float markerSizePixel = 97.0;   // for WEBCAM 1 (Valentin) --> focalLen = 694.72974

        // get the focal length of the camera
        float distance = 0.265;        // distance between the camera and the marker
        float markerSizeMeter = 0.037; // size of the markers in meters
        // f[px] = x[px] * z[m] / x[m]
        float focalLen = markerSizePixel * distance / markerSizeMeter;
        // get the x and y size of the frame
        float x = frameIn->cols;
        float y = frameIn->rows;
        cv::Matx33f camMatrix(focalLen, 0.0f, (x - 1) / 2.0f,
                              0.0f, focalLen, (y - 1) / 2.0f,
                              0.0f, 0.0f, 1.0f);
    }

#if WEBCAM
    float focalLen = -694.72974;
#else
    float focalLen = -522.83783;

    // float focalLen = 250;
#endif

    cv::Matx33f camMatrix(focalLen, 0.0f, (frameIn->cols - 1) / 2.0f,
                          0.0f, focalLen, (frameIn->rows - 1) / 2.0f,
                          0.0f, 0.0f, 1.0f);
    cv::Mat distCoeffs;
    // // Refind strategy to detect more markers
    detector.refineDetectedMarkers(*frameIn, board, board_corners, board_ids, rejected, camMatrix, distCoeffs);
    detector.refineDetectedMarkers(*frameIn, board, piece_corners, piece_ids, rejected, camMatrix, distCoeffs);

    // only continue if we have at least 4 markers
    if (board_ids.size() != 4)
    {
        return {};
    }

    // Get the center of the board using the board_corners mean
    cv::Point2f boardCenter = cv::Point2f(0, 0);
    for (size_t i = 0; i < board_corners.size(); i++)
    {
        boardCenter += board_corners[i][0];
        // for (size_t j = 0; j < board_corners[i].size(); j++)
        // {
        //     std::cout << "board_corners[" << i << "][" << j << "]: " << board_corners[i][j] << std::endl;
        //     boardCenter += board_corners[i][j];
        // }
    }
    boardCenter = boardCenter / (float)(board_corners.size());

    // get the inner most corner of everz board?corners entry
    std::vector<cv::Point2f> innerCorners;
    for (size_t i = 0; i < board_corners.size(); i++)
    {
        cv::Point2f innerCorner = board_corners[i][0];
        for (size_t j = 1; j < board_corners[i].size(); j++)
        {
            if (cv::norm(innerCorner - boardCenter) > cv::norm(board_corners[i][j] - boardCenter))
            {
                innerCorner = board_corners[i][j];
            }
        }
        innerCorners.push_back(innerCorner);
    }

    // order by board_ids
    // board_ids contains numbers from 0 to 3
    std::vector<cv::Point2f> orderedCorners(4, cv::Point2f(0, 0));
    for (size_t i = 0; i < board_ids.size(); i++)
    {
        auto id = board_ids[i];
        orderedCorners[id] = (innerCorners[i]);

    }

    // base white
    // from 0 to 1
    // base black
    // from 2 to 3

    // left side from 1 to 3
    // right side from 0 to 2

    cv::Point2f baseWhite = orderedCorners[1] - orderedCorners[0];
    cv::Point2f baseBlack = orderedCorners[3] - orderedCorners[2];
    cv::Point2f leftSide = orderedCorners[3] - orderedCorners[1];
    cv::Point2f rightSide = orderedCorners[2] - orderedCorners[0];

    // maybe switch baseBlack direction if dot product with baseWhite is negative
    if (baseWhite.dot(baseBlack) < 0)
    {
        baseBlack = -baseBlack;
    }


    // draw the inner corners
    // for (size_t i = 0; i < innerCorners.size(); i++)
    // {
    //     cv::circle(*frameOut, orderedCorners[i], 5, cv::Scalar(0, 255, 0), 2);
    //     // also draw the index
    //     std::string text = std::to_string(i);
    //     cv::putText(*frameOut, text, orderedCorners[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    // }

    // get piece image points as middle of all corners
    std::vector<cv::Point2f> pieceImgPoints;
    for (size_t i = 0; i < piece_corners.size(); i++)
    {
        cv::Point2f pieceImgPoint = cv::Point2f(0, 0);
        for (size_t j = 0; j < piece_corners[i].size(); j++)
        {
            pieceImgPoint += piece_corners[i][j];
        }
        pieceImgPoint = pieceImgPoint / (float)piece_corners[i].size();
        pieceImgPoints.push_back(pieceImgPoint);
    }

    // draw the piece image points
    for (size_t i = 0; i < pieceImgPoints.size(); i++)
    {
        cv::circle(*frameOut, pieceImgPoints[i], 5, cv::Scalar(0, 0, 255), 2);
    }

    // BW is the base white vector
    // BB is the base black vector
    // BW and BB are the opposite sides of the board and are pointing in the same direction
    // LS is the left side vector
    // RS is the right side vector
    // LS and RS are the opposite sides of the board and are pointing in the same direction

    // These edges represent a skewed chess board
    // The board is not a perfect square (eg has no parrallel size)

    // first of we want to find all intersection points in a 2d grid
    // we can do this by finding the intersection of the lines of the board
    // by subdividing each of the edges into 8 parts we can find 64 intersection points


    auto drawIntersectionPoints = false;
    std::vector<std::vector<cv::Point2f>> intersectionPoints;
    for (size_t i = 0; i < 8; i++)
    {
        std::vector<cv::Point2f> row;
        for (size_t j = 0; j < 8; j++)
        {
            // base point bw
            cv::Point2f firstBW = orderedCorners[0] + (((float)i + 0.5) / 8.0f) * baseWhite;
            // base point bb
            cv::Point2f firstBB = orderedCorners[2] + (((float)i + 0.5) / 8.0f) * baseBlack;

            // first line is defined between firstBW and firstBB

            cv::Point2f secondLeft = orderedCorners[1] + (((float)j + 0.5) / 8.0f) * leftSide;
            cv::Point2f secondRight = orderedCorners[0] + (((float)j + 0.5) / 8.0f) * rightSide;

            // second line is defined between secondLeft and secondRight

            // draw the lines
            if (drawIntersectionPoints)
            {
            cv::line(*frameOut, firstBW, firstBB, cv::Scalar(255, 0, 0), 2);
            cv::line(*frameOut, secondLeft, secondRight, cv::Scalar(255, 0, 0), 2);
            }

            // intersection of the two lines
            // TODO:

            // intersection of the two lines
            cv::Point2f x = secondLeft - firstBW;
            cv::Point2f d1 = firstBB - firstBW;
            cv::Point2f d2 = secondRight - secondLeft;

            float cross = d1.x * d2.y - d1.y * d2.x;
            if (abs(cross) < /*EPS*/ 1e-8)
                continue;

            double t1 = (x.x * d2.y - x.y * d2.x) / cross;
            cv::Point2f intersectionPoint = firstBW + d1 * t1;

            // offset each intersection point

            row.push_back(intersectionPoint);
        }
        intersectionPoints.push_back(row);
    }

    // draw the intersection points
    if (drawIntersectionPoints) {
        for (size_t i = 0; i < intersectionPoints.size(); i++)
        {
            for (size_t j = 0; j < intersectionPoints[i].size(); j++)
            {
                cv::circle(*frameOut, intersectionPoints[i][j], 5, cv::Scalar(255, 0, 0), 2);
            }
        }
    }

    // for everz piece find next intersection point, we need the i, j index of the intersection point
    // we can do this by finding the closest intersection point to the piece image point
    ChessboardUpdate chessboard_update;
    for (size_t i = 0; i < pieceImgPoints.size(); i++)
    {
        cv::Point2f pieceImgPoint = pieceImgPoints[i];
        cv::Point2f closestIntersectionPoint = intersectionPoints[0][0];
        size_t best_j, best_k;
        for (size_t j = 0; j < intersectionPoints.size(); j++)
        {
            for (size_t k = 0; k < intersectionPoints[j].size(); k++)
            {
                if (cv::norm(pieceImgPoint - closestIntersectionPoint) > cv::norm(pieceImgPoint - intersectionPoints[j][k]))
                {
                    closestIntersectionPoint = intersectionPoints[j][k];
                    best_j = j;
                    best_k = k;
                }
            }
        }
        // std::cout << "Closest intersection point to piece " << i << ": " << best_j << ", " << best_k << std::endl;

        auto id = marker_to_piece(piece_ids[i]);
        chessboard_update.push_back(std::make_pair(cv::Point(best_j, best_k), id));
    }

    // // cv::Mat R;
    // cv::Rodrigues(rvec, R);

    // // update the transformation in the ChessboardManager
    // manager->update_transformation(R, tvec);

    // if (manager->get_transformation().has_value())
    // {
    //     std::cout << "transformation" << manager->get_transformation().value().first << std::endl;
    // }
    // // update the chessboard grid in the ChessboardManager
    manager->chessboard.update(chessboard_update);

    manager->chessboard.print_board();

    try {
        auto move = manager->chessboard.update_board();

        
        auto from = move.from();
        auto to = move.to();

        int from_i = from.rank();
        int from_j = from.file();

        int to_i = to.rank();
        int to_j = to.file();

        std::cout << "Move from " << from_i << ", " << from_j << " to " << to_i << ", " << to_j << std::endl;

      
        return std::make_pair(intersectionPoints[from_j][7 - from_i], intersectionPoints[to_j][7 - to_i]);

    
    } catch (...) {
        std::cerr << "Failed to interface with chess.hpp" << std::endl;

    }

    // manager->illustrate_move(*frameOut, cv::Point(0, 0), cv::Point(8, 8));

    // return std::move(chessboard_grid);
    return {};
}

static std::map<int, std::weak_ptr<Camera>> cameras;

static int default_camera = 0;

SharedCamera camera_open(int id)
{
    if (id < 0)
        id = default_camera;

#if WEBCAM
    id = 2;
#endif

    std::cout << "Query camera " << id << std::endl;

    if (cameras.find(id) == cameras.end())
    {

        try
        {

            SharedCamera camera = std::make_shared<Camera>(id);

            cameras[id] = camera;

            std::cout << "Ready camera " << id << std::endl;

            return camera;
        }
        catch (const std::runtime_error &e)
        {
            return std::shared_ptr<Camera>();
        }
    }
    else
    {

        return cameras[id].lock();
    }
}

void camera_delete_all()
{
    cameras.clear();
}

void *camera_create(int id)
{
    SharedCamera camera = camera_open(id);

    return new SharedCamera(camera);
}

void camera_delete(void *obj)
{
    if (!obj)
        return;

    delete (SharedCamera *)obj;
}

int camera_get_image(void *obj, uint8_t *buffer, unsigned long *newer, int display_option)
{
    SharedCamera user_data = *((SharedCamera *)obj);

    Mat frame;

    if (newer)
        frame = user_data->getFrameIfNewer(*newer, display_option);
    else
    {
        frame = user_data->getFrame(display_option);
    }

    if (frame.empty())
    {
        return 0;
    }

    Mat wrapper(user_data->getHeight(), user_data->getWidth(), CV_8UC3, buffer, std::max(user_data->getHeight(), user_data->getWidth()) * 3);
    cvtColor(frame, wrapper, COLOR_BGR2RGB);

    return 1;
}

int camera_get_width(void *obj)
{
    SharedCamera user_data = *((SharedCamera *)obj);

    return user_data->getWidth();
}

int camera_get_height(void *obj)
{
    SharedCamera user_data = *((SharedCamera *)obj);

    return user_data->getHeight();
}

void camera_set_default(int id)
{
    default_camera = id;
}

void camera_flip(void *obj, int flip_lr, int flip_ud)
{
    SharedCamera user_data = *((SharedCamera *)obj);
    user_data->flip(flip_lr, flip_ud);
}

void camera_set_threshold(void *obj, int value)
{
    SharedCamera user_data = *((SharedCamera *)obj);
    user_data->setThreshold(value);
}

/*int camera_get_first_strip_height(void* obj)
{
    SharedCamera user_data = *((SharedCamera*)obj);
    return user_data->getFirstStripSizeHeight();
}*/
