
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <map>

#include "camera.h"
#include <opencv2/calib3d.hpp>
#include "chessboard.h"


#define WEBCAM 1

void transformImagePointsToObjectPoints(const std::vector<cv::Point2f>& imagePoints,
                                        const cv::Matx33f& cameraMatrix,
                                        const cv::Mat& distCoeffs,
                                        const cv::Mat& rVec,
                                        const cv::Mat& tVec,
                                        std::vector<cv::Point3f>& objectPoints) {
    // Convert rotation vector to rotation matrix
    cv::Mat R;
    cv::Rodrigues(rVec, R);

    // Invert the rotation matrix
    cv::Mat R_inv = R.t();

    // Compute the inverse translation vector
    cv::Mat tVec_inv = -R_inv * tVec;

    // Undistort image points
    std::vector<cv::Point2f> undistortedPoints;
    cv::undistortPoints(imagePoints, undistortedPoints, cameraMatrix, distCoeffs);

    // Transform points from image space to object space
    for (const auto& pt : undistortedPoints) {
        cv::Mat pt_homogeneous = (cv::Mat_<double>(3, 1) << pt.x, pt.y, 1.0);
        cv::Mat objectPoint = R_inv * pt_homogeneous + tVec_inv;
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

            auto board_grid = calculateBoardGrid(&frame, &frame);

            counter++;
        }

        auto end = std::chrono::high_resolution_clock::now();

        auto used = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        auto remaining = std::chrono::milliseconds(std::max(1l, (long)((1000.0 / fps) - used)));

        std::this_thread::sleep_for(remaining);
    }
}

ChessboardUpdate calculateBoardGrid(cv::Mat *frameIn, cv::Mat *frameOut)
{

    // Create detector and dictionary

    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);

    aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
    detectorParams.minDistanceToBorder = 0;
    detectorParams.adaptiveThreshWinSizeStep = 100;

    aruco::ArucoDetector detector(dictionary, detectorParams);

    // Create GridBoard object
    int markersX = 5;           // number of markers in X
    int markersY = 7;           // number of markers in Y
    float markerLength = 20;    // marker length
    float markerSeparation = 3; // separation between markers
    aruco::GridBoard board(Size(markersX, markersY), markerLength, markerSeparation, dictionary);

    // Length of the axis in the board
    float axisLength =  0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
                          markerSeparation);

    // Detect markers
    std::vector<int> ids;
    std::vector<std::vector<Point2f>> corners, rejected;
    detector.detectMarkers(*frameIn, corners, ids, rejected);

    // filter ids and corners to only keep ids that are part of the board
    const std::vector<int> custom_ids = { 0, 1, 2, 3, 4, 5 };
    auto marker_to_piece = [](int id) -> ChessboardPiece {
        // Marker ids start with 10
        // White pieces come first
        // 0 - 7: White pawns
        // 8 - 15: White pieces (ROOK, KNIGHT, BISHOP, QUEEN, KING)
        // 16 - 23: Black pawns
        // 24 - 31: Black pieces (ROOK, KNIGHT, BISHOP, QUEEN, KING)

        auto id_without_offset = id - 10;
        auto color = id_without_offset < 16 ? ChessboardPieceColor::WHITE : ChessboardPieceColor::BLACK;

        auto type = ChessboardPieceType::PAWN;
        auto type_id = id_without_offset % 16;
        if (type_id <= 7)
        {
            type = ChessboardPieceType::PAWN;
        }
        else if (type_id <= 11)
        {
            type = ChessboardPieceType::ROOK;
        }
        else if (type_id <= 13)
        {
            type = ChessboardPieceType::KNIGHT;
        }
        else if (type_id <= 15)
        {
            type = ChessboardPieceType::BISHOP;
        }
        else if (type_id == 16)
        {
            type = ChessboardPieceType::QUEEN;
        }
        else if (type_id == 17)
        {
            type = ChessboardPieceType::KING;
        }

        return ChessboardPiece(type, color);
    };


    std::vector<int> board_ids, piece_ids;
    std::vector<std::vector<Point2f>> board_corners, piece_corners;
    for (size_t i = 0; i < ids.size(); i++)
    {
        if (std::find(custom_ids.begin(), custom_ids.end(), ids[i]) != custom_ids.end())
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

    // Draw results
    if (!ids.empty())
        aruco::drawDetectedMarkers(*frameOut, board_corners, board_ids);
    // std::cout << "Markers detected (id): " << ids.size() << std::endl;

    // Draw rejected markers
    bool showRejected = false;
    if (showRejected && !rejected.empty())
        aruco::drawDetectedMarkers(*frameOut, rejected, noArray(), Scalar(100, 0, 255));

//     // Calibrate the camera
//     bool calibrate = true;
//     if (calibrate)
//     {
//         // f[px] = x[px] * z[m] / x[m]
//         // get the size of one marker in pixels using the detected corners. Also use an inline if statement to avoid division by zero
//         float markerSizePixel = (ids.size() > 0) ? (float)cv::norm(corners[0][0] - corners[0][1]) : 1.0;
//         std::cout << "markerSizePixel: " << markerSizePixel << std::endl;
//         // float markerSizePixel = 73.0;   // for WEBCAM 0 (Valentin) --> focalLen = 522.83783
//         // float markerSizePixel = 97.0;   // for WEBCAM 1 (Valentin) --> focalLen = 694.72974

//         // get the focal length of the camera
//         float distance = 0.265; // distance between the camera and the marker
//         float markerSizeMeter = 0.037; // size of the markers in meters     
//         // f[px] = x[px] * z[m] / x[m]
//         float focalLen = markerSizePixel * distance / markerSizeMeter;
//         // get the x and y size of the frame
//         float x = frameIn->cols;
//         float y = frameIn->rows;
//         cv::Matx33f camMatrix(focalLen, 0.0f, (x - 1) / 2.0f,
//                               0.0f, focalLen, (y - 1) / 2.0f,
//                               0.0f, 0.0f, 1.0f);
//         std::cout << "camMatrix: " << camMatrix << std::endl;
// }

#if WEBCAM
    float focalLen = 694.72974;
#else
    float focalLen = 522.83783;
#endif
    cv::Matx33f camMatrix(focalLen, 0.0f, (frameIn->cols - 1) / 2.0f,
                          0.0f, focalLen, (frameIn->rows - 1) / 2.0f,
                          0.0f, 0.0f, 1.0f);
    cv::Mat distCoeffs;
    // Refind strategy to detect more markers
    detector.refineDetectedMarkers(*frameIn, board, board_corners, board_ids, rejected, camMatrix, distCoeffs);
    detector.refineDetectedMarkers(*frameIn, board, piece_corners, piece_ids, rejected, camMatrix, distCoeffs);

    // Estimate board pose
    int markersOfBoardDetected = 0;
    cv::Mat rvec, tvec;
    if (board_ids.size() >= 4)
    {
        // Get object and image points for the solvePnP function
        std::vector<cv::Point3f> objPoints;
        std::vector<cv::Point2f> imgPoints;
        board.matchImagePoints(board_corners, board_ids, objPoints, imgPoints);

        if (imgPoints.size() >= 4)
        {
            // Find pose of the board from the detected markers
            cv::solvePnP(objPoints, imgPoints, camMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
            markersOfBoardDetected = (int)objPoints.size() / 4;
        } else {
            return {};
        }

    } else {
        return {};
    }

    if (markersOfBoardDetected > 0)
        cv::drawFrameAxes(*frameOut, camMatrix, distCoeffs, rvec, tvec, axisLength);


    // Transform pieces into objPoints using tVec and rVec
    std::vector<cv::Point3f> piecePoints;

    // get piece image points as middle of all corners
    std::vector<cv::Point2f> pieceImgPoints;
    for (size_t i = 0; i < piece_corners.size(); i++)
    {
        cv::Point2f pieceImgPoint = cv::Point2f(0, 0);
        for (size_t j = 0; j < piece_corners[i].size(); j++)
        {
            pieceImgPoint += piece_corners[i][j];
        }
        pieceImgPoint = pieceImgPoint / (float) piece_corners[i].size();
        pieceImgPoints.push_back(pieceImgPoint);
    }
    
    // transform piece image points to object points
    transformImagePointsToObjectPoints(pieceImgPoints, camMatrix, distCoeffs, rvec, tvec, piecePoints);

    // drop z coordinate for all pieces
    std::vector<cv::Point2f> board_grid;
    for (size_t i = 0; i < piecePoints.size(); i++)
    {
        board_grid.push_back(cv::Point2f(piecePoints[i].x, piecePoints[i].y));
    }

    // Whole board is length 1, get Point2i for each piece
    std::vector<std::pair<cv::Point2i, ChessboardPiece>> chessboard_grid;
    for (size_t i = 0; i < board_grid.size(); i++)
    {
        cv::Point2i point = cv::Point2i((int)(board_grid[i].x * 8), (int)(board_grid[i].y * 8));
        // get id
        int id = piece_ids[i];
        // get piece
        ChessboardPiece piece = marker_to_piece(id);
        chessboard_grid.push_back(std::make_pair(point, piece));
    }

    return std::move(chessboard_grid);
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

