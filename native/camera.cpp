
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <map>

#include "camera.h"

#define DRAW_CONTOUR 0
#define THICKNESS_VALUE 4
#define MARKER_ID_UNDEFINED -1
#define SCRATCH 0


int subpixSampleSafe(const Mat& pSrc, const Point2f& p) {
    // floorf -> like int casting, but -2.3 will be the smaller number -> -3
    // Point is float, we want to know which color does it have
    int fx = int(floorf(p.x));
    int fy = int(floorf(p.y));

    if (fx < 0 || fx >= pSrc.cols - 1 ||
        fy < 0 || fy >= pSrc.rows - 1)
        return 127;

    // Slides 15
    int px = int(256 * (p.x - floorf(p.x)));
    int py = int(256 * (p.y - floorf(p.y)));

    // Here we get the pixel of the starting point
    unsigned char* i = (unsigned char*)((pSrc.data + fy * pSrc.step) + fx);

    // Shift 2^8
    // Internsity
    int a = i[0] + ((px * (i[1] - i[0])) >> 8);
    i += pSrc.step;
    int b = i[0] + ((px * (i[1] - i[0])) >> 8);

    // We want to return Intensity for the subpixel
    return a + ((py * (b - a)) >> 8);
}

Mat calculateStripDimensions(double dx, double dy, StripDimensions& strip_dimensions) {
    // Norm (euclidean distance) from the direction vector is the length (derived from the Pythagoras Theorem)
    double diffLength = sqrt(dx * dx + dy * dy);

    // Length proportional to the marker size
    strip_dimensions.stripLength = (int)(0.8 * diffLength);

    if (strip_dimensions.stripLength < 5)
        strip_dimensions.stripLength = 5;

    // Make stripeLength odd (because of the shift in nStop), Example 6: both sides of the strip must have the same length XXXOXXX
    //st.stripeLength |= 1;
    if (strip_dimensions.stripLength % 2 == 0)
        strip_dimensions.stripLength++;

    // E.g. stripeLength = 5 --> from -2 to 2: Shift -> half top, the other half bottom
    //st.nStop = st.stripeLength >> 1;
    strip_dimensions.nStop = strip_dimensions.stripLength / 2;
    strip_dimensions.nStart = -strip_dimensions.nStop;

    Size stripSize;

    // Sample a strip of width 3 pixels
    stripSize.width = 3;
    stripSize.height = strip_dimensions.stripLength;

    // Normalized direction vector
    strip_dimensions.stripeVecX.x = dx / diffLength;
    strip_dimensions.stripeVecX.y = dy / diffLength;

    // Normalized perpendicular direction vector (rotated 90° clockwise, rotation matrix)
    strip_dimensions.stripeVecY.x = strip_dimensions.stripeVecX.y;
    strip_dimensions.stripeVecY.y = -strip_dimensions.stripeVecX.x;

    // 8 bit unsigned char with 1 channel, gray
    return Mat(stripSize, CV_8UC1);
}


Camera::Camera(int input) : fps(30), flip_lr(false), flip_ud(false)
{
    capture.open(input);
    //capture.open("MarkerMovie.MP4");

    if (!capture.isOpened())
    {
        std::cout << "No webcam, using video file" << std::endl;
        capture.open("MarkerMovie.MP4");
        if (!capture.isOpened()) {
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

    if (display_option == DISPLAY_OPTION_RBG) {
        return frame;
    }
    // else if (display_option == DISPLAY_OPTION_GREYSCALE) {
    //     return greyscale;
    // }
    else {
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

    if (display_frame_option == DISPLAY_OPTION_RBG) {
        return frame;
    }
    // else if (display_frame_option == DISPLAY_OPTION_GREYSCALE) {
    //     return greyscale;
    // }
    else {
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

            if (flip_lr || flip_ud)
            {
                int code = flip_lr ? (flip_ud ? -1 : 1) : 0;
                cv::flip(frame, frame, code);
            }

            this->computeThreshold(&frame, &greyscale);

            std::vector<std::vector<cv::Point>> approx_contours = this->computeApproxContours(&greyscale, &frame);
            std::vector<labeled_marker> markers;
            for (int i = 0; i < approx_contours.size(); i++)
            {
                markers.push_back(this->processContour(approx_contours[i], &frame));
            }

            auto board = find_board_corners(markers);
            std::vector<Point> approx_board;
            for (auto corner : board) {
                // std::cout << corner <<std::endl;
                approx_board.push_back(corner);
            }
            
            auto board_grid = calculateBoardGrid(approx_board, markers, &frame);

            counter++;
        }

        auto end = std::chrono::high_resolution_clock::now();

        auto used = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        auto remaining = std::chrono::milliseconds(std::max(1l, (long)((1000.0 / fps) - used)));

        std::this_thread::sleep_for(remaining);
    }
}


std::vector<cv::Point2f> calculateBoardGrid(std::vector<cv::Point> approx_board, std::vector<labeled_marker> marker_pairs, cv::Mat* frame)
{
    // Sort the corners in a way that they form a rectangle, if one connects the corners in the order they are stored in the vector
    std::sort(approx_board.begin(), approx_board.end(), [](cv::Point a, cv::Point b) { return a.x < b.x; });
    if (approx_board[0].y > approx_board[1].y)
    {
        std::swap(approx_board[0], approx_board[1]);
    }
    if (approx_board[2].y < approx_board[3].y)
    {
        std::swap(approx_board[2], approx_board[3]);
    }    

    // Draw the outline of the board
    cv::polylines(*frame, approx_board, true, CV_RGB(255, 0, 0), THICKNESS_VALUE);
    // Mark the first corner with a green circle
    cv::circle(*frame, approx_board[0], 5, CV_RGB(0, 255, 0), -1);

    // Divide the board into a grid of 8x8 squares
    std::vector<cv::Point2f> board_grid;
    for (int i = 0; i < 9; i++)
    {
        // Calculate the position of the current row
        cv::Point2f row_start = approx_board[0] + (approx_board[3] - approx_board[0]) * i / 8;
        cv::Point2f row_end = approx_board[1] + (approx_board[2] - approx_board[1]) * i / 8;

        for (int j = 0; j < 9; j++)
        {
            // Calculate the position of the current square
            cv::Point2f square = row_start + (row_end - row_start) * j / 8;
            board_grid.push_back(square);
        }
    }

#if SCRATCH
    // Draw the grid
    for (int i = 0; i < 9; i++)
    {
        cv::line(*frame, board_grid[i * 9], board_grid[i * 9 + 8], CV_RGB(0, 0, 255), THICKNESS_VALUE);
        cv::line(*frame, board_grid[i], board_grid[72 + i], CV_RGB(0, 0, 255), THICKNESS_VALUE);
    }

    // Draw the corners of the grid
    for (int i = 0; i < 81; i++)
    {
        cv::circle(*frame, board_grid[i], 2, CV_RGB(0, 0, 255), -1);
    }

#else

    // Same but with cv::aruco::GridBoard
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
    auto detectorParams = cv::aruco::DetectorParameters::create();

    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    auto board = cv::aruco::GridBoard::create(2, 2, 0.031, 0.1, dictionary);


    std::vector<std::array<cv::Point2f, 4>> marker_corners(marker_pairs.size());
    // get all the corners of the markers
    std::transform(marker_pairs.begin(), marker_pairs.end(), marker_corners.begin(), [](labeled_marker marker) {
        return marker.first;
    });

    // get all of the ids
    std::vector<int> ids(marker_pairs.size());
    std::transform(marker_pairs.begin(), marker_pairs.end(), ids.begin(), [](labeled_marker marker) {
        return marker.second;
    });

    // detect the markers
    detector.detectMarkers(frame, marker_corners, ids);

     // Refind strategy to detect more markers
    if(true)
    detector.refineDetectedMarkers(frame, board, marker_corners, ids, camMatrix, distCoeffs);
    
    // Estimate the board pose
    cv::Mat objPoints, imgPoints;
    board->matchImagePoints(corners, ids, objPoints, imgPoints);

    // Find pose
    cv::solvePnP(objPoints, imgPoints, camMatrix, distCoeffs, rvec, tvec);

    // Draw the detected markers
    cv::aruco::drawDetectedMarkers(frame, corners, ids);

    // Draw the board
    cv::aruco::drawAxis(frame, camMatrix, distCoeffs, rvec, tvec, 0.1);

    // Draw Frame axis
    cv::drawFrameAxes(frame, camMatrix, distCoeffs, rvec, tvec, 0.1);

#endif

    return board_grid;
}


void Camera::computeThreshold(cv::Mat* frame_in, cv::Mat* frame_out)
{
    cv::cvtColor(*frame_in, *frame_out, COLOR_BGR2GRAY);

    if (this->threshold == 0)
    {
        cv::adaptiveThreshold(*frame_out, *frame_out, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 33, 5);
    }
    else {
        cv::threshold(*frame_out, *frame_out, this->threshold, 255, THRESH_BINARY);
    }
}


std::vector<std::vector<Point>> Camera::computeApproxContours(cv::Mat* frame_in, cv::Mat* frame_out)
{
    std::vector<std::vector<Point>> contours;
    // RETR_LIST is a list of all found contour, SIMPLE is to just save the begin and ending of each edge which belongs to the contour
    cv::findContours(*frame_in, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

    //cv::drawContours(frame, contours, -1, CV_RGB(0, 255, 0), 4);

    std::vector<std::vector<Point>> approx_contours;
    // size is always positive, so unsigned int -> size_t; if you have not initialized the vector it is -1, hence crash
    for (size_t k = 0; k < contours.size(); k++) {

        // -------------------------------------------------

        // --- Process Contour ---

        std::vector<Point> approx_contour;

        // Simplifying of the contour with the Ramer-Douglas-Peuker Algorithm
        // true -> Only closed contours
        // Approximation of old curve, the difference (epsilon) should not be bigger than: perimeter(->arcLength)*0.02
        cv::approxPolyDP(contours[k], approx_contour, cv::arcLength(contours[k], true) * 0.02, true);

        // Convert to a usable rectangle
        Rect bounding_rect = cv::boundingRect(approx_contour);

        // skip if != 4 corners
        if (approx_contour.size() != 4) {
            continue;
        }

        // --- Filter tiny ones --- If the found contour is too small (20 -> pixels, frame.cols - 10 to prevent extreme big contours)
        if (bounding_rect.height < 20 || bounding_rect.width < 20 || bounding_rect.width > frame.cols - 10 || bounding_rect.height > frame.rows - 10) {
            continue;
        }

        // -> Cleaning done!

        // 1 -> 1 contour, we have a closed contour, true -> closed, 4 -> thickness
        // cv::polylines(*frame_out, approx_contour, true, CV_RGB(255, 0, 0), THICKNESS_VALUE);

        approx_contours.push_back(approx_contour);
    }

    return approx_contours;
}

labeled_marker Camera::processContour(std::vector<Point> approx_contour, cv::Mat* frame_draw)
{
    
    // Direction vector (x0,y0) and contained point (x1,y1) -> For each line -> 4x4 = 16
    float subpix_line_params[16];
    // lineParams is shared, CV_32F -> Same data type like lineParams
    cv::Mat subpix_line_params_mat(Size(4, 4), CV_32F, subpix_line_params);
    

    for (size_t i = 0; i < approx_contour.size(); ++i) {
        // Render the corners, 3 -> Radius, -1 filled circle
        // cv::circle(*frame_draw, approx_contour[i], 3, CV_RGB(0, 255, 0), -1);

        // Euclidic distance, 7 -> parts, both directions dx and dy
        double dx = ((double)approx_contour[(i + 1) % 4].x - (double)approx_contour[i].x) / 7.0;
        double dy = ((double)approx_contour[(i + 1) % 4].y - (double)approx_contour[i].y) / 7.0;


        StripDimensions strip_dimensions;
        // A simple array of unsigned char cv::Mat
        Mat image_pixel_strip = calculateStripDimensions(dx, dy, strip_dimensions);
        

        cv::Point2f edge_points_subpix[6];

        // First point already rendered, now the other 6 points
        for (int j = 1; j < 7; ++j) {
            // Position calculation
            double px = (double)approx_contour[i].x + (double)j * dx;
            double py = (double)approx_contour[i].y + (double)j * dy;

            cv::Point point_approx_edge;
            point_approx_edge.x = (int)px;
            point_approx_edge.y = (int)py;
            //cv::circle(*frame_draw, p, 2, CV_RGB(0, 0, 255), -1);

            computeStrip(&point_approx_edge, &strip_dimensions, &image_pixel_strip);

            // TEST ALTERNATIVE - START
            // The first and last row must be excluded from the sobel calculation because they have no top or bottom neighbors
            std::vector<double> sobel_values(strip_dimensions.stripLength - 2.);

            // To use the kernel we start with the second row (n) and stop before the last one
            for (int n = 1; n < (strip_dimensions.stripLength - 1); n++) {
                // Take the intensity value from the stripe 
                unsigned char* stripePtr = &(image_pixel_strip.at<uchar>(n - 1, 0));
                // Calculation of the gradient with the sobel for the first row
                double r1 = -stripePtr[0] - 2. * stripePtr[1] - stripePtr[2];
                // r2 -> Is equal to 0 because of sobel
                // Go two lines for the third line of the sobel, step = size of the data type, here uchar
                stripePtr += 2 * image_pixel_strip.step;
                // Calculation of the gradient with the sobel for the third row
                double r3 = stripePtr[0] + 2. * stripePtr[1] + stripePtr[2];
                // Writing the result into our sobel value vector
                unsigned int ti = n - 1;
                sobel_values[ti] = r1 + r3;
            }

            double max_intensity = -1;
            int max_intensity_index = 0;

            // Finding the max value
            for (int n = 0; n < strip_dimensions.stripLength - 2; ++n) {
                if (sobel_values[n] > max_intensity) {
                    max_intensity = sobel_values[n];
                    max_intensity_index = n;
                }
            }
            // TEST ALTERNATIVE - END

            // f(x) slide 7 -> y0 .. y1 .. y2
            double y0, y1, y2;

            // Point before and after
            unsigned int max1 = max_intensity_index - 1, max2 = max_intensity_index + 1;

            // If the index is at the border we are out of the stripe, then we will take 0
            y0 = (max_intensity_index <= 0) ? 0 : sobel_values[max1]; //y0 = (max_intensity_index <= 0) ? 0 : sobel_gradient_y.at<uchar>(max1, 1);  //TEST ALTERNATIVE
            y1 = sobel_values[max_intensity_index]; //y1 = sobel_gradient_y.at<uchar>(max_intensity_index, 1);  // TEST ALTERNATIVE 
            // If we are going out of the array of the sobel values
            y2 = (max_intensity_index >= strip_dimensions.stripLength - 3) ? 0 : sobel_values[max2]; //y2 = (max_intensity_index >= strip_dimensions.stripLength - 3) ? 0 : sobel_gradient_y.at<uchar>(max2, 1);  // TEST ALTERNATIVE 

            // Formula for calculating the x-coordinate of the vertex of a parabola, given 3 points with equal distances 
            // (xv means the x value of the vertex, d the distance between the points): 
            // xv = x1 + (d / 2) * (y2 - y0)/(2*y1 - y0 - y2)

            // d = 1 because of the normalization and x1 will be added later
            double pos = (y2 - y0) / (4 * y1 - 2 * y0 - 2 * y2);

            // What happens when there is no solution -> /0 or Number == other Number
            // If the found pos is not a number -> there is no solution
            if (isnan(pos)) {
                continue;
            }

            // Where is the edge (max gradient) in the picture?
            int max_index_shift = max_intensity_index - (strip_dimensions.stripLength >> 1);

            // Find the original edgepoint -> Is the pixel point at the top or bottom?
            edge_points_subpix[j - 1].x = (double)point_approx_edge.x + (((double)max_index_shift + pos) * strip_dimensions.stripeVecY.x);
            edge_points_subpix[j - 1].y = (double)point_approx_edge.y + (((double)max_index_shift + pos) * strip_dimensions.stripeVecY.y);

            // Highlight the subpixel with blue color
            // cv::circle(frame, edge_points_subpix[j - 1], 2, CV_RGB(0, 0, 255), -1);
            
            if (is_first_strip) {
                cv::resize(image_pixel_strip, this->first_pixel_strip, Size(this->width, this->height), 0, 0, INTER_NEAREST);
                is_first_strip = false;
            }
        }

        // We now have the array of exact edge centers stored in "points", every row has two values -> 2 channels!
        cv::Mat high_intensity_points(Size(1, 6), CV_32FC2, edge_points_subpix);

        // fitLine stores the calculated line in lineParams per column in the following way:
        // vec.x, vec.y, point.x, point.y
        // Norm 2, 0 and 0.01 -> Optimal parameters
        // i -> Edge points
        cv::fitLine(high_intensity_points, subpix_line_params_mat.col(i), DIST_L2, 0, 0.01, 0.01);
        // We need two points to draw the line
        cv::Point p1;
        // We have to jump through the 4x4 matrix, meaning the next value for the wanted line is in the next row -> +4
        // d = -50 is the scalar -> Length of the line, g: Point + d*Vector
        // p1<----Middle---->p2
        //   <-----100----->
        p1.x = (int)subpix_line_params[8 + i] - (int)(50.0 * subpix_line_params[i]);
        p1.y = (int)subpix_line_params[12 + i] - (int)(50.0 * subpix_line_params[4 + i]);

        Point p2;
        p2.x = (int)subpix_line_params[8 + i] + (int)(50.0 * subpix_line_params[i]);
        p2.y = (int)subpix_line_params[12 + i] + (int)(50.0 * subpix_line_params[4 + i]);

        // Draw line
        // cv::line(*frame_draw, p1, p2, CV_RGB(0, 255, 255), 1, 8, 0);
    }

    // So far we stored the exact line parameters and show the lines in the image now we have to calculate the exact corners
    std::array<cv::Point2f,4> subpix_corners = calculateSubpixCorners(subpix_line_params, &frame);

    int marker_id = getMarkerID(&greyscale, subpix_corners, true);  // Exercise 10
    
    return std::make_pair(subpix_corners, marker_id);
}


void Camera::computeStrip(cv::Point* center_point, StripDimensions* strip_dims, cv::Mat* out_image_pixel_strip)
{
    // Columns: Loop over 3 pixels
    for (int m = -1; m <= 1; ++m) {
        // Rows: From bottom to top of the stripe, e.g. -3 to 3
        for (int n = strip_dims->nStart; n <= strip_dims->nStop; ++n) {
            cv::Point2f subpix_edge_point;

            // m -> going over the 3 pixel thickness of the stripe, n -> over the length of the stripe, direction comes from the orthogonal vector in st
            // Going from bottom to top and defining the pixel coordinate for each pixel belonging to the stripe
            subpix_edge_point.x = (double)center_point->x + ((double)m * strip_dims->stripeVecX.x) + ((double)n * strip_dims->stripeVecY.x);
            subpix_edge_point.y = (double)center_point->y + ((double)m * strip_dims->stripeVecX.y) + ((double)n * strip_dims->stripeVecY.y);

            cv::Point point_draw;  // Just for markings in the image!
            point_draw.x = (int)subpix_edge_point.x;
            point_draw.y = (int)subpix_edge_point.y;

            // The one (purple color) which is shown in the stripe window
            // if (is_first_strip)
            //     cv::circle(frame, point_draw, 1, CV_RGB(255, 0, 255), -1);
            // else
            //     cv::circle(frame, point_draw, 1, CV_RGB(0, 255, 255), -1);

            // Combined Intensity of the subpixel
            int pixelIntensity = subpixSampleSafe(this->greyscale, subpix_edge_point);
            //int pixelIntensity = (((m+1)+n) % 2) * 255; // TEST

            // Converte from index to pixel coordinate
            // m (Column, real) -> -1,0,1 but we need to map to 0,1,2 -> add 1 to 0..2
            int w = m + 1;

            // n (Row, real) -> add stripelenght >> 1 to shift to 0..stripeLength
            // n=0 -> -length/2, n=length/2 -> 0 ........ + length/2
            int h = n + (strip_dims->stripLength >> 1);

            // Set pointer to correct position and safe subpixel intensity
            out_image_pixel_strip->at<uchar>(h, w) = (uchar)pixelIntensity;
        }
    }
}

std::array<cv::Point2f, 4> Camera::calculateSubpixCorners(float subpix_line_params[16], cv::Mat* frame_draw)
{
    std::array<cv::Point2f, 4> subpix_corners;

    // Calculate the intersection points of both lines
    for (int i = 0; i < 4; ++i) {
        // Go through the corners of the rectangle, 3 -> 0
        int j = (i + 1) % 4;

        double x0, x1, y0, y1, u0, u1, v0, v1;

        // We have to jump through the 4x4 matrix, meaning the next value for the wanted line is in the next row -> +4
        // g: Point + d*Vector
        // g1 = (x0,y0) + scalar0*(u0,v0) == g2 = (x1,y1) + scalar1*(u1,v1)
        x0 = subpix_line_params[i + 8]; y0 = subpix_line_params[i + 12];
        x1 = subpix_line_params[j + 8]; y1 = subpix_line_params[j + 12];

        // Direction vector
        u0 = subpix_line_params[i]; v0 = subpix_line_params[i + 4];
        u1 = subpix_line_params[j]; v1 = subpix_line_params[j + 4];

        // Cramer's rule
        // 2 unknown a,b -> Equation system
        double a = x1 * u0 * v1 - y1 * u0 * u1 - x0 * u1 * v0 + y0 * u0 * u1;
        double b = -x0 * v0 * v1 + y0 * u0 * v1 + x1 * v0 * v1 - y1 * v0 * u1;

        // Calculate the cross product to check if both direction vectors are parallel -> = 0
        // c -> Determinant = 0 -> linear dependent -> the direction vectors are parallel -> No division with 0
        double c = v1 * u0 - v0 * u1;
        if (fabs(c) < 0.001) {
            // std::cout << "lines parallel" << std::endl;
            continue;
        }

        // We have checked for parallelism of the direction vectors
        // -> Cramer's rule, now divide through the main determinant
        a /= c;
        b /= c;

        // Exact corner
        subpix_corners[i].x = a;
        subpix_corners[i].y = b;

        Point point_draw;
        point_draw.x = (int)subpix_corners[i].x;
        point_draw.y = (int)subpix_corners[i].y;

        // circle(*frame_draw, point_draw, 5, CV_RGB(255, 255, 0), -1);
    } // End of the loop to extract the exact corners

    return subpix_corners;
}

int Camera::getMarkerID(cv::Mat* frame_src, std::array<cv::Point2f, 4> subpix_corners, bool draw_marker_id = false)
{
    // Coordinates on the original marker images to go to the actual center of the first pixel -> 6x6
    cv::Point2f target_corners[4];
    target_corners[0].x = -0.5; target_corners[0].y = -0.5;
    target_corners[1].x = 5.5; target_corners[1].y = -0.5;
    target_corners[2].x = 5.5; target_corners[2].y = 5.5;
    target_corners[3].x = -0.5; target_corners[3].y = 5.5;

    // Create and calculate the matrix of perspective transform -> non affin -> parallel stays not parallel
    // Homography is a matrix to describe the transformation from an image region to the 2D projected image
    cv::Mat homography_matrix(Size(3, 3), CV_32FC1);
    // Corner which we calculated and our target Mat, find the transformation
    homography_matrix = getPerspectiveTransform(subpix_corners.data(), target_corners);

    // Create image for the marker
    cv::Mat image_marker(Size(6, 6), CV_8UC1);

    // Change the perspective in the marker image using the previously calculated Homography Matrix
    // In the Homography Matrix there is also the position in the image saved
    warpPerspective(*frame_src, image_marker, homography_matrix, Size(6, 6));

    // Now we have a B/W image of a supposed Marker
    //threshold(imageMarker, imageMarker, bw_thresh, 255, THRESH_BINARY);
    int code = 0;
    for (int i = 0; i < 6; ++i) {
        // Check if border is black
        int pixel1 = image_marker.at<uchar>(0, i); //top
        int pixel2 = image_marker.at<uchar>(5, i); //bottom
        int pixel3 = image_marker.at<uchar>(i, 0); //left
        int pixel4 = image_marker.at<uchar>(i, 5); //right

        // 0 -> black
        if ((pixel1 > 0) || (pixel2 > 0) || (pixel3 > 0) || (pixel4 > 0)) {
            code = -1;
            break;
        }
    }

    if (code < 0) {
        return MARKER_ID_UNDEFINED;
    }

    // Copy the BW values into cP -> codePixel on the marker 4x4 (inner part of the marker, no black border)
    int cP[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            // +1 -> no borders!
            cP[i][j] = image_marker.at<uchar>(i + 1, j + 1);
            cP[i][j] = (cP[i][j] == 0) ? 1 : 0;  // If black then 1 else 0
        }
    }

    // Save the ID of the marker, for each side
    int codes[4];
    codes[0] = codes[1] = codes[2] = codes[3] = 0;

    // Calculate the code from all sides at once
    for (int i = 0; i < 16; i++) {
        // /4 to go through the rows
        int row = i >> 2;
        int col = i % 4;

        // Multiplied by 2 to check for black values -> 0*2 = 0
        codes[0] <<= 1;
        codes[0] |= cP[row][col]; // 0�

        // 4x4 structure -> Each column represents one side 
        codes[1] <<= 1;
        codes[1] |= cP[3 - col][row]; // 90�

        codes[2] <<= 1;
        codes[2] |= cP[3 - row][3 - col]; // 180�

        codes[3] <<= 1;
        codes[3] |= cP[col][3 - row]; // 270�
    }

    // Account for symmetry -> One side complete white or black
    if ((codes[0] == 0) || (codes[0] == 0xffff)) {
        return MARKER_ID_UNDEFINED;
    }

    // Search for the smallest marker ID
    code = codes[0];
    for (int i = 1; i < 4; ++i) {
        if (codes[i] < code) {
            code = codes[i];
        }
    }

    // Print ID
    // printf("Found: %04x\n", code);

    // Show the first detected marker in the image
    if (this->is_first_marker) {
        cv::resize(image_marker, this->first_marker, Size(this->width, this->height), 0, 0, INTER_NEAREST);
        this->is_first_marker = false;
    }

    if (draw_marker_id)
    {
        cv::Point point_draw_marker_id;
        point_draw_marker_id.x = (int)(subpix_corners[0].x + subpix_corners[1].x + subpix_corners[2].x + subpix_corners[3].x) / 4;
        point_draw_marker_id.y = (int)(subpix_corners[0].y + subpix_corners[1].y + subpix_corners[2].y + subpix_corners[3].y) / 4;
        cv::putText(frame, std::to_string(code), point_draw_marker_id, cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255), 2);
    }

    return code;
}

static std::map<int, std::weak_ptr<Camera>> cameras;

static int default_camera = 0;

SharedCamera camera_open(int id)
{
    if (id < 0)
        id = default_camera;

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

void camera_set_threshold(void* obj, int value)
{
    SharedCamera user_data = *((SharedCamera*)obj);
    user_data->setThreshold(value);
}

/*int camera_get_first_strip_height(void* obj)
{
    SharedCamera user_data = *((SharedCamera*)obj);
    return user_data->getFirstStripSizeHeight();
}*/


std::array<cv::Point2f,4> find_board_corners(std::vector<labeled_marker> marker) {
// filter out marker with matching ids
auto allowed_markers = { 0xA5, 0x1068, 0x690, 0x10E2 };
std::vector<labeled_marker> filtered_marker;
    for (auto m : marker) {
        for (auto allowed : allowed_markers) {
            if (m.second == allowed) {
                filtered_marker.push_back(m);
                break;
            }
        }
    }
    // Find center points of markers
    std::vector<cv::Point2f> center_points;
    for (auto m : filtered_marker) {
        cv::Point2f center = cv::Point2f(0.0, 0.0);
        for (auto p : m.first) {
            center += p;
        }
        center_points.push_back(center / 4);
    }

    // find center of centers
    cv::Point2f center = cv::Point2f(0.0, 0.0);
    for (auto p : center_points) {
        center += p;
    }
    center = center / ((int) center_points.size());

    // for each filtered_marker, find the point closest to center point calculated above
    std::array<cv::Point2f, 4> corners;
    int i = 0;
    for (auto marker : filtered_marker) {
        cv::Point2f closest = marker.first[0];
        double min_dist = cv::norm(center - closest);
        for (auto p : marker.first) {
            double dist = cv::norm(center - p);
            if (dist < min_dist) {
                min_dist = dist;
                closest = p;
            }
        }
        corners[i] = closest;
        i++;
    }

    return corners;

}

