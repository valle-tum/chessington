
#ifndef CAMERA_API_H
#define CAMERA_API_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

    void *camera_create(int camera);
    void camera_delete(void *obj);

    int camera_get_image(void *obj, uint8_t *buffer, unsigned long *newer, int display_option);
    int camera_get_width(void *obj);
    int camera_get_height(void *obj);

    void camera_flip(void *obj, int flip_lr, int flip_ud);

    void camera_delete_all();

    void camera_set_default(int id);

    void camera_set_threshold(void* obj, int value);

    static const int DISPLAY_OPTION_RBG = 0,
        DISPLAY_OPTION_GREYSCALE = 1,
        DISPLAY_OPTION_PIXELSTRIP = 2,
        DISPLAY_OPTION_MARKER = 3;

#ifdef __cplusplus
}

#pragma once
#include <thread>
#include <mutex>
#include <memory>

#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace cv;

struct StripDimensions {
    int stripLength;
    int nStop;
    int nStart;
    Point2f stripeVecX;
    Point2f stripeVecY;
};


using labeled_marker = std::pair<std::array<cv::Point_<float>,4>, int>;

class Camera
{
public:
    Camera(int input);
    ~Camera();

    Mat getFrame(int display_option);
    Mat getFrameIfNewer(unsigned long &current, int display_option);
    unsigned long getFrameNumber();
    int getWidth();
    int getHeight();

    int flip(bool flip_lr = true, bool flip_ud = false);

    void setThreshold(int value);


private:
    void loop();
    void computeThreshold(cv::Mat* frame_src, cv::Mat* frame_dst);
    std::vector<std::vector<Point>> computeApproxContours(cv::Mat* frame_src, cv::Mat* frame_draw);
    labeled_marker processContour(std::vector<Point> approx_contour, cv::Mat* frame_draw);
    void computeStrip(cv::Point* center_point, StripDimensions* strip, cv::Mat* image_pixel_strip);
    std::array<cv::Point2f, 4> calculateSubpixCorners(float subpix_line_params[16], cv::Mat* frame_draw);
    int getMarkerID(cv::Mat* frame_src, std::array<cv::Point2f, 4> subpix_corners, bool draw_marker_id);

    cv::VideoCapture capture;
    cv::Mat frame, greyscale;
    cv::Mat first_pixel_strip, first_marker;
    int width;
    int height;
    int threshold = 0;
    unsigned long counter;
    float fps;
    bool flip_lr, flip_ud;
    bool is_first_strip = true, is_first_marker = true;
    std::array<cv::Point,4> board_corners;

    std::recursive_mutex guard;
    std::thread worker;
};


typedef std::shared_ptr<Camera> SharedCamera;

SharedCamera camera_open(int id = -1);

std::array<cv::Point2f,4> find_board_corners(std::vector<labeled_marker> marker);
// std::vector<cv::Point2f> calculateBoardGrid(std::vector<cv::Point> approx_board, std::vector<labeled_marker> marker_pairs, cv::Mat* frame);
std::vector<cv::Point2f> calculateBoardGrid(cv::Mat* frameIn, cv::Mat* frameOut);

#endif

#endif
