#ifndef COMPUTER_VISION_H_
#define COMPUTER_VISION_H_

#include <iostream>
#include <cmath>
#include <vector>
#include "mapping.h"
#include "car_config.h"
// #include "system/systick.h"
using namespace std;

struct Point { 
    int x, y;  // x, y coordinates of the point
    float gradient;  // gradient of the point
    bool isExtraPoint;  // whether the point is contained between a corner and the opposing edge
};

enum TrackStatus { Straight, CrossRoad, RightAhead, LeftAhead, RightNow, LeftNow, NotSure };

class ComputerVision {
    public:
        /**
         * update raw camera data pointer
         * @param cameraDataPtr raw camera data pointer
        */
        inline void receiveData(const uint8_t *cameraPtr);

        /**
         * converts camera frame into useful data
         * find optimal path and store result in angle and distance
        */
        void conversion();
        /**
         * converts camera frame into useful data
         * find optimal path and store result in angle and distance
         * for obstacle avoidance
        */
        void conversionAvoid(int tofValue1, int tofValue2);
        void conversionAvoidTest();

        /**
         * failsave if can't find any target
         * i.e. distance = 0, angle = 0
         * steering to max left / right or go straight
         * @return 0 -> straight, 1 -> right, 2 -> left
        */
        int failsave();

        inline float getDistance() const;
        inline float getAngle() const;
        inline float getAngleDegree() const;
        inline TrackStatus getTrackStatus() const;

        inline float getCutoff() { return cutoff; };
        inline void setCutoff(float value) { cutoff = value; };

        /**
         * visualize the edge before perspective transformation
         * @param processedDataPtr: pointer of processed data array
         * @param drawLeft: draw left edge?
         * @param drawRight: draw right edge?
         * @param drawExtraPoint: draw the edge in the cross road?
        */
        void edgeVisualizer(uint8_t *processedDataPtr, bool drawLeft, bool drawRight, bool drawExtraPoint);
        
        /**
         * visualize the edge after perspective transformation
         * scaling ratio base on REMAP_RATIO, REMAP_X_OFFSET, REMAP_Y_OFFSET in mapping.cpp
         * @param processedDataPtr: pointer of processed data array
         * @param drawLeft: draw left edge?
         * @param drawRight: draw right edge?
        */
        void edgeVisualizerPT(uint8_t *processedDataPtr, bool drawLeft, bool drawRight);
        /**
         * visualize the path after perspective transformation
         * scaling ratio base on REMAP_RATIO, REMAP_X_OFFSET, REMAP_Y_OFFSET in mapping.cpp
         * @param processedDataPtr: pointer of processed data array
        */
        void pathVisualizerPT(uint8_t *processedDataPtr);

        /**
         * visualize the edge after perspective transformation
         * scaling ratio base on ratio, x_offset, y_offset hardcoded in this function
         * @param processedDataPtr: pointer of processed data array
         * @param width: width of processedData image
         * @param height: height of processedData image
         * @param drawLeft: draw left edge?
         * @param drawRight: draw right edge?
        */
        void edgeVisualizerPT_Test(uint8_t *processedDataPtr, int width, int height, bool drawLeft, bool drawRight);
        /**
         * visualize the path after perspective transformation
         * scaling ratio base on ratio, x_offset, y_offset hardcoded in this function
         * @param processedDataPtr: pointer of processed data array
         * @param width: width of processedData image
         * @param height: height of processedData image
        */
        void pathVisualizerPT_Test(uint8_t *processedDataPtr, int width, int height);
        void pathVisualizerPT_Test1(uint8_t *processedDataPtr, int width, int height);
        void pathVisualizerPTAvoid_Test(uint8_t *processedDataPtr, int width, int height);

        bool rightBox = false;
        bool leftBox = false;
        int stickRight = 1;  // 0 = left, 1 = mid, 2 = right;
        bool useRight = true;
        bool boxflag = false;
        int tofValue1, tofValue2;
        uint32_t timer;
    private:
        // Data pointer of camera input
        const uint8_t* cameraPtr;

        // Stores the points of the left edge and right edge before perspective transformation
        vector<Point> leftEdge, rightEdge;
        // Stores limit edges and middle line data detected after perspective transformation
        vector<pair<float,float>> leftLimit, rightLimit, midPoints; 
        
        float cutoff = PATH_PLAN_CUTOFF;
        int targetIndex;  // store the index of target point in edge array for visualization
        float distance;  // distance of the path (in pixel after perspective transformation)
        float angle;  // steering angle (in radian)
        TrackStatus trackStatus = TrackStatus::NotSure;

        float prevDistance = 0.0;  // previous distance of the path (in pixel after perspective transformation)
        float prevAngle = 0.0;  // previous steering angle (in radian)
        TrackStatus prevTrackStatus = TrackStatus::NotSure;


        #ifdef OP_fix_camera_noise
        uint8_t processedData[120*184];
        inline void fixHenry();
        #endif

        /**
         * detect the start point of left or right edge
         * be the reference for edgeDetection function (starting point)
         * also store the edge point into the corresponding vector (leftEdge / rightEdge)
         * @param topLimit y-coord of the farest edge in the middle part of the image (topPoint function)
         * @param isRightEdge is the right edge? or the left edge.
         * @return have successfully detect a starting point?
        */
        bool startingPoint(int topLimit, bool isRightEdge);
        bool startingPoint1(int topLimit, bool isRightEdge);
        bool startingPoint2(int topLimit, bool isRightEdge);
        bool startingPoint3(int topLimit, bool isRightEdge);
        bool startingPoint4(int topLimit, bool isRightEdge);
        bool startingPoint8(int topLimit, bool isRightEdge);
        bool startingPointAvoid(int topLimit, bool isRightEdge);

        bool startingPointGuard();

        /**
         * tracking the edge based on gradient
         * store the edge detected in to the corresponding array (leftEdge / rightEdge)
         * @param isRightEdge is the right edge? or the left edge.
        */
        void edgeDetection(bool isRightEdge);
        void edgeDetectionAvoid(bool isRightEdge);

        void classifyTrackStatus();

        /**
         * genearate leftLimit, rightLimit and midPoints by remapping the edge using remapImg in mapping.cpp
         * basicly is fisheye correction and perspective transformation
         * only using the longest side of edge as reference and remap
        */
        void remap();
        void remap1();
        void remap2();
        void remap3();
        void remapAvoid();
        uint8_t whichEdge = 1;
        void remapAvoid2(int topLimit, int rightLimit, int LeftLimit);
        void remapAvoid3(int topLimit);

        /**
         * find optimal path and store result in angle and distance
         * search the farest mid point the car can go without hitting the edge of the track
         * using the edge and mid points after remapping
        */
        void pathPlanning();
        void pathPlanning1();

        void pathPlanningWithStatus();

        /**
         * check whether it is an edge point using sobel operator
         * @param x x-coord of the point
         * @param y y-coord of the point
         * @return is an edge point?
        */
        inline bool isEdge(int x, int y);
        /**
         * check whether it is an edge point using sobel operator and get the gradient
         * @param x x-coord of the point
         * @param y y-coord of the point
         * @return if it is an edge point -> return gradient, else -> NAN
        */
        inline float edgeGradient(int x, int y);

        /**
         * check the difference of angle 1 and 2 is smaller than PI/10.0 (18 degree)
         * @param angle1 angle 1
         * @param angle2 angle 2
         * @return is similar angle?
        */
        inline bool isSimilarAngles(float angle1, float angle2);
        /**
         * check the difference of angle 1 and 2 is smaller than PI/3.0 (60 degree)
         * @param angle1 angle 1
         * @param angle2 angle 2
         * @return is same direction?
        */
        inline bool isSameDirection(float angle1, float angle2);

        /**
         * check is a corner point or not
         * @param isRightEdge is the right edge? or the left edge.
         * @return is a corner point?
        */
        inline bool isCorner(bool isRightEdge);
        inline bool isCorner1(bool isRightEdge);
        inline bool isCorner2(bool isRightEdge);
        inline bool isCorner3(bool isRightEdge);

        /**
         * search for the corresponding edge after the cross road and continue tracking the edge based on gradient
         * store the edge detected in to the corresponding array (leftEdgePts / rightEdgePts)
         * --> take over the duty of trackEdge function after detected a cross road
         * @param isRightEdge is the right edge? or the left edge.
         * 
         * function will be called when isCorner is true, find next point that has a similar gradient
         * @param isRightEdge is the right edge? or the left edge.
         * @return is cross road?
        */
        bool isCrossRoad(bool isRightEdge);

        /**
         * detect the point of farest edge in the middle part of the image (mid straight vertical)
         * be the reference for startingPoint function to avoid detecting wrong edge
         * @return y-coord of the point
        */
        int topPoint();
        int topLeftPoint();
        int topRightPoint();

        /**
         * Check if point is cointained in line
         * @param Ax x-coord of base point
         * @param Ay y-coord of base point
         * @param Bx x-coord of second point on midline
         * @param By y-coord of second point on midline
         * @param threshold minimum distance between point and line
         * @param searchLimit minimum distance between point and line
         * @return point is cointained in line?
        */
        bool ptIntersectLimit(float Ax, float Ay,float Bx, float By, float threshold, uint16_t searchLimit);

        /**
         * find the side of edge which have more edge point (i.e. longer)
         * @return the side of the longer edge (true -> right, false -> left)
        */
        bool findBestRefEdge();
        bool findBestRefEdge1();
        bool findBestRefEdge2();
};

inline void ComputerVision::receiveData(const uint8_t *cameraPtr) {
    this->cameraPtr = cameraPtr;
}

#ifdef OP_fix_camera_noise
inline void ComputerVision::fixHenry() {
    for (uint16_t x=0; x<RAW_WIDTH; x++) {
        for (uint16_t y=0; y<RAW_HEIGHT; y++) {
            int offset = RAW_WIDTH * y + x;
            if (x%2 == 0) {
                if (x == 0) {
                    processedData[offset] = *(cameraPtr + offset + 1);
                } else if (x == RAW_WIDTH-1) {
                    processedData[offset] = *(cameraPtr + offset - 1);
                } else {
                    processedData[offset] = (*(cameraPtr + offset - 1) + *(cameraPtr + offset + 1)) / 2;
                }
            } else {
                processedData[offset] = *(cameraPtr + offset);
            }
        }
    }
    cameraPtr = processedData;
}
#endif

inline bool ComputerVision::isEdge(int x, int y) {
    int gX = *(cameraPtr + RAW_WIDTH * (y-1) + (x+1))
        + *(cameraPtr + RAW_WIDTH * y + (x+1)) * 2
        + *(cameraPtr + RAW_WIDTH * (y+1) + (x+1))
        - *(cameraPtr + RAW_WIDTH * (y-1) + (x-1))
        - *(cameraPtr + RAW_WIDTH * y + (x-1)) * 2
        - *(cameraPtr + RAW_WIDTH * (y+1) + (x-1));
    int gY = *(cameraPtr + RAW_WIDTH * (y-1) + (x-1))
        + *(cameraPtr + RAW_WIDTH * (y-1) + x) * 2
        + *(cameraPtr + RAW_WIDTH * (y-1) + (x+1))
        - *(cameraPtr + RAW_WIDTH * (y+1) + (x-1))
        - *(cameraPtr + RAW_WIDTH * (y+1) + x) * 2
        - *(cameraPtr + RAW_WIDTH * (y+1) + (x+1));
    return hypot(gX, gY) > SOBEL_THRESHOLD;
}

inline float ComputerVision::edgeGradient(int x, int y) {
    int gX = *(cameraPtr + RAW_WIDTH * (y-1) + (x+1))
        + *(cameraPtr + RAW_WIDTH * y + (x+1)) * 2
        + *(cameraPtr + RAW_WIDTH * (y+1) + (x+1))
        - *(cameraPtr + RAW_WIDTH * (y-1) + (x-1))
        - *(cameraPtr + RAW_WIDTH * y + (x-1)) * 2
        - *(cameraPtr + RAW_WIDTH * (y+1) + (x-1));
    int gY = *(cameraPtr + RAW_WIDTH * (y-1) + (x-1))
        + *(cameraPtr + RAW_WIDTH * (y-1) + x) * 2
        + *(cameraPtr + RAW_WIDTH * (y-1) + (x+1))
        - *(cameraPtr + RAW_WIDTH * (y+1) + (x-1))
        - *(cameraPtr + RAW_WIDTH * (y+1) + x) * 2
        - *(cameraPtr + RAW_WIDTH * (y+1) + (x+1));
    return hypot(gX, gY) > SOBEL_THRESHOLD ? atan2f(gY, gX) : NAN;
}

inline bool ComputerVision::isSimilarAngles(float angle1, float angle2) {
    float diff = fabsf(angle1 - angle2);
    if (diff >= PI) diff = PI*2 - diff;
    return diff < PI/10.0;
}

inline bool ComputerVision::isSameDirection(float angle1, float angle2) {
    float diff = fabsf(angle1 - angle2);
    if (diff >= PI) diff = PI*2 - diff;
    return diff < PI/3.0;
}

inline bool ComputerVision::isCorner(bool isRightEdge) {
    float a, b, c;
    if (isRightEdge) {
        if (rightEdge.size() < 3) return false;
        a = powf((float)rightEdge[rightEdge.size()-1].x-(float)rightEdge[rightEdge.size()-3].x, 2) + powf((float)rightEdge[rightEdge.size()-1].y-(float)rightEdge[rightEdge.size()-3].y, 2);
        b = powf((float)rightEdge[rightEdge.size()-1].x-(float)rightEdge[rightEdge.size()-2].x, 2) + powf((float)rightEdge[rightEdge.size()-1].y-(float)rightEdge[rightEdge.size()-2].y, 2);
        c = powf((float)rightEdge[rightEdge.size()-2].x-(float)rightEdge[rightEdge.size()-3].x, 2) + powf((float)rightEdge[rightEdge.size()-2].y-(float)rightEdge[rightEdge.size()-3].y, 2);
    } else {
        if (leftEdge.size() < 3) return false;
        a = powf((float)leftEdge[leftEdge.size()-1].x-(float)leftEdge[leftEdge.size()-3].x, 2) + powf((float)leftEdge[leftEdge.size()-1].y-(float)leftEdge[leftEdge.size()-3].y, 2);
        b = powf((float)leftEdge[leftEdge.size()-1].x-(float)leftEdge[leftEdge.size()-2].x, 2) + powf((float)leftEdge[leftEdge.size()-1].y-(float)leftEdge[leftEdge.size()-2].y, 2);
        c = powf((float)leftEdge[leftEdge.size()-2].x-(float)leftEdge[leftEdge.size()-3].x, 2) + powf((float)leftEdge[leftEdge.size()-2].y-(float)leftEdge[leftEdge.size()-3].y, 2);
    }
    return acosf((b+c-a)/(2*sqrtf(b)*sqrtf(c))) < 2*PI/3;  // 120
}

inline bool ComputerVision::isCorner1(bool isRightEdge) {
    float prevGradient = 0, prevprevGradient = 0;
    if (!isRightEdge) {
        int size = leftEdge.size();
        prevGradient = atan2f(leftEdge[size-1].y-leftEdge[size-5].y, leftEdge[size-1].x-leftEdge[size-5].x);
        prevprevGradient = atan2f(leftEdge[size-5].y-leftEdge[size-8].y, leftEdge[size-5].x-leftEdge[size-8].x);

        if (prevprevGradient-prevGradient>80/180*PI && prevprevGradient-prevGradient<110/180*PI) {
            for (int i=1; i<=5; ++i) {
                leftEdge[size-i].isExtraPoint = true;
            }
            return true;
        }
    } else {
        int size = rightEdge.size();
        prevGradient = atan2f(rightEdge[size-1].y-rightEdge[size-5].y, rightEdge[size-1].x-rightEdge[size-5].x);
        prevprevGradient = atan2f(rightEdge[size-5].y-rightEdge[size-8].y, rightEdge[size-5].x-rightEdge[size-8].x);

        if (prevGradient-prevprevGradient>80/180*PI && prevGradient-prevprevGradient<110/180*PI) {
            for (int i=1; i<=5; ++i) {
                rightEdge[size-i].isExtraPoint = true;
            }
            return true;
        }
    }
    return false;
}

inline bool ComputerVision::isCorner2(bool isRightEdge) {
    float a, b, c;
    if (isRightEdge) {
        if (rightEdge.size() < 5) return false;
        a = powf((float)rightEdge[rightEdge.size()-1].x-(float)rightEdge[rightEdge.size()-5].x, 2) + powf((float)rightEdge[rightEdge.size()-1].y-(float)rightEdge[rightEdge.size()-5].y, 2);
        b = powf((float)rightEdge[rightEdge.size()-1].x-(float)rightEdge[rightEdge.size()-3].x, 2) + powf((float)rightEdge[rightEdge.size()-1].y-(float)rightEdge[rightEdge.size()-3].y, 2);
        c = powf((float)rightEdge[rightEdge.size()-3].x-(float)rightEdge[rightEdge.size()-5].x, 2) + powf((float)rightEdge[rightEdge.size()-3].y-(float)rightEdge[rightEdge.size()-5].y, 2);
    } else {
        if (leftEdge.size() < 5) return false;
        a = powf((float)leftEdge[leftEdge.size()-1].x-(float)leftEdge[leftEdge.size()-5].x, 2) + powf((float)leftEdge[leftEdge.size()-1].y-(float)leftEdge[leftEdge.size()-5].y, 2);
        b = powf((float)leftEdge[leftEdge.size()-1].x-(float)leftEdge[leftEdge.size()-3].x, 2) + powf((float)leftEdge[leftEdge.size()-1].y-(float)leftEdge[leftEdge.size()-3].y, 2);
        c = powf((float)leftEdge[leftEdge.size()-3].x-(float)leftEdge[leftEdge.size()-5].x, 2) + powf((float)leftEdge[leftEdge.size()-3].y-(float)leftEdge[leftEdge.size()-5].y, 2);
    }
    return acosf((b+c-a)/(2*sqrtf(b)*sqrtf(c))) < 2*PI/3;  // 120
}

inline bool ComputerVision::isCorner3(bool isRightEdge) {
    float a, b, c;
    if (isRightEdge) {
        if (rightEdge.size() < 5) return false;
        a = powf((float)rightEdge[rightEdge.size()-1].x-(float)rightEdge[rightEdge.size()-5].x, 2) + powf((float)rightEdge[rightEdge.size()-1].y-(float)rightEdge[rightEdge.size()-5].y, 2);
        b = powf((float)rightEdge[rightEdge.size()-1].x-(float)rightEdge[rightEdge.size()-3].x, 2) + powf((float)rightEdge[rightEdge.size()-1].y-(float)rightEdge[rightEdge.size()-3].y, 2);
        c = powf((float)rightEdge[rightEdge.size()-3].x-(float)rightEdge[rightEdge.size()-5].x, 2) + powf((float)rightEdge[rightEdge.size()-3].y-(float)rightEdge[rightEdge.size()-5].y, 2);
    } else {
        if (leftEdge.size() < 5) return false;
        a = powf((float)leftEdge[leftEdge.size()-1].x-(float)leftEdge[leftEdge.size()-5].x, 2) + powf((float)leftEdge[leftEdge.size()-1].y-(float)leftEdge[leftEdge.size()-5].y, 2);
        b = powf((float)leftEdge[leftEdge.size()-1].x-(float)leftEdge[leftEdge.size()-3].x, 2) + powf((float)leftEdge[leftEdge.size()-1].y-(float)leftEdge[leftEdge.size()-3].y, 2);
        c = powf((float)leftEdge[leftEdge.size()-3].x-(float)leftEdge[leftEdge.size()-5].x, 2) + powf((float)leftEdge[leftEdge.size()-3].y-(float)leftEdge[leftEdge.size()-5].y, 2);
    }
    return acosf((b+c-a)/(2*sqrtf(b)*sqrtf(c))) < 2*PI/3;  // 120
}

inline float ComputerVision::getAngle() const {
    return angle;
}

inline float ComputerVision::getAngleDegree() const {
    return angle*180/PI;
}

inline float ComputerVision::getDistance() const {
    return distance;
}

inline TrackStatus ComputerVision::getTrackStatus() const {
    return trackStatus;
}

#endif /* COMPUTER_VISION_H_ */