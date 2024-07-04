
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <map>

#include "camera.h"


#define WEBCAM 1

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

std::vector<cv::Point2f> calculateBoardGrid(cv::Mat *frameIn, cv::Mat *frameOut)
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
    float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
                               markerSeparation);

    // Detect markers
    std::vector<int> ids;
    std::vector<std::vector<Point2f>> corners, rejected;
    detector.detectMarkers(*frameIn, corners, ids, rejected);

    // Draw results
    if (!ids.empty())
        aruco::drawDetectedMarkers(*frameOut, corners, ids);
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
    detector.refineDetectedMarkers(*frameIn, board, corners, ids, rejected, camMatrix, distCoeffs);
    // std::cout << "camMatrix: " << camMatrix << std::endl;
    // std::cout << "distCoeffs: " << distCoeffs << std::endl;
    // std::cout << std::endl;

    // Estimate board pose
    int markersOfBoardDetected = 0;
    cv::Mat rvec, tvec;
    if (ids.size() >= 4)
    {
        // std::cout << "Board detected" << std::endl;
        // Get object and image points for the solvePnP function
        // cv::Mat objPoints, imgPoints;
        std::vector<cv::Point3f> objPoints;
        std::vector<cv::Point2f> imgPoints;
        board.matchImagePoints(corners, ids, objPoints, imgPoints);
        // std::cout << "corners (num detected marker): " << corners.size() << " ids " << ids.size() << std::endl;
        // std::cout << "imgPoints (calculated points 2d from corners): " << imgPoints.size() << std::endl;
        // std::cout << "objPoints (calculated points 3d from ids): " << objPoints.size() << std::endl;

        if (imgPoints.size() >= 4)
        {
            // Find pose of the board from the detected markers
            cv::solvePnP(objPoints, imgPoints, camMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
            // cv::solvePnP(objPoints, imgPoints, camMatrix, distCoeffs, rvec, tvec);
            // std::cout << "rvec: " << rvec << std::endl;
            // std::cout << "tvec: " << tvec << std::endl;
            // std::cout << std::endl;

            // markersOfBoardDetected = (int)objPoints.total() / 4;
            markersOfBoardDetected = (int)objPoints.size() / 4;
            // std::cout << "Markers of board detected (objPoint): " << markersOfBoardDetected << std::endl;
        }
    }

    if (markersOfBoardDetected > 0)
        cv::drawFrameAxes(*frameOut, camMatrix, distCoeffs, rvec, tvec, axisLength);

        // // Generate markers for the GridBoard (for printing)
        // board.generateImage(cv::Size(640, 480), *frameOut, 5, 1);


    std::vector<cv::Point2f> board_grid;

    return board_grid;
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

