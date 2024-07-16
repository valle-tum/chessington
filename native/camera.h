
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
    
    int camera_get_counterFrame(void *obj);

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

#include "chessboard.h"

using namespace cv;


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

    int getCounterFrame();

    int flip(bool flip_lr = true, bool flip_ud = false);

    void setThreshold(int value);

private:
    void loop();
    
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
    int counterFrames;

    std::recursive_mutex guard;
    std::thread worker;
};


typedef std::shared_ptr<Camera> SharedCamera;

SharedCamera camera_open(int id = -1);
std::optional<std::pair<cv::Point2f, cv::Point2f>> updateChessModel(cv::Mat* frameIn, cv::Mat* frameOut, ChessboardManager* manager, int* counterFrames);

#endif

#endif
