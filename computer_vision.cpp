#include "computer_vision.h"
#include <chrono>
#include "mode.h"

void ComputerVision::conversion() {
    // Reset all variables
    // auto start0 = std::chrono::steady_clock::now();
    targetIndex = 0;
    angle = 0.0;
    distance = 0.0;
    trackStatus = TrackStatus::NotSure;
    leftEdge.clear();
    rightEdge.clear();
    leftLimit.clear();
    rightLimit.clear();
    midPoints.clear();
    // std::chrono::duration<double, std::micro> diff0 = std::chrono::steady_clock::now()-start0;
    // std::cout << "Time taken (clear) " << diff0.count() << " us\n";

    // auto start1 = std::chrono::steady_clock::now();
    int topLimit = topPoint();
    #ifdef vision_debug_output
    std::cout<<"topLimit: "<<topLimit<<std::endl;
    #endif
    if (topLimit == 99999) return;  // Can't find any path
    if (topLimit <= 5) trackStatus = TrackStatus::Straight;
    // std::chrono::duration<double, std::micro> diff1 = std::chrono::steady_clock::now()-start1;
    // std::cout << "Time taken (top point) " << diff1.count() << " us\n";

    // auto start2 = std::chrono::steady_clock::now();
    bool rightSuccess = startingPoint(topLimit, true);
    bool leftSuccess = startingPoint(topLimit, false);
    // std::chrono::duration<double, std::micro> diff2 = std::chrono::steady_clock::now()-start2;
    // std::cout << "Time taken (start points) " << diff2.count() << " us\n";

    // auto start3 = std::chrono::steady_clock::now();
    if (rightSuccess) edgeDetection(true);
    if (leftSuccess && startingPointGuard()) edgeDetection(false);
    // std::chrono::duration<double, std::micro> diff3 = std::chrono::steady_clock::now()-start3;
    // std::cout << "Time taken (edge detection) " << diff3.count() << " us\n";

    // auto start4 = std::chrono::steady_clock::now();
    remap();
    // std::chrono::duration<double, std::micro> diff4 = std::chrono::steady_clock::now()-start4;
    // std::cout << "Time taken (remap) " << diff4.count() << " us\n";

    // auto start5 = std::chrono::steady_clock::now();
    pathPlanning();
    // std::chrono::duration<double, std::micro> diff5 = std::chrono::steady_clock::now()-start5;
    // std::cout << "Time taken (path planning) " << diff5.count() << " us\n";

    // remap1();
    // pathPlanning1();

    classifyTrackStatus();

    pathPlanningWithStatus();

    #ifdef vision_debug_output
    std::cout<<"Found target: "<<targetIndex<<" dist: "<<distance<<" steer: "<<angle/PI*180<<std::endl;
    std::cout<<"Track status: ";
    switch (trackStatus) {
        case TrackStatus::NotSure: std::cout<<"NotSure"<<std::endl; break;
        case TrackStatus::Straight: std::cout<<"Straight"<<std::endl; break;
        case TrackStatus::CrossRoad: std::cout<<"CrossRoad"<<std::endl; break;
        case TrackStatus::RightAhead: std::cout<<"RightAhead"<<std::endl; break;
        case TrackStatus::LeftAhead: std::cout<<"LeftAhead"<<std::endl; break;
        case TrackStatus::RightNow: std::cout<<"RightNow"<<std::endl; break;
        case TrackStatus::LeftNow: std::cout<<"LeftNow"<<std::endl; break;
        default: std::cout<<std::endl; break;
    }
    #endif
}

void ComputerVision::conversionAvoid(int tofValue1, int tofValue2) {
    // Reset all variables
    targetIndex = 0;
    angle = 0.0;
    distance = 0.0;
    trackStatus = TrackStatus::NotSure;
    leftEdge.clear();
    rightEdge.clear();
    leftLimit.clear();
    rightLimit.clear();
    midPoints.clear();

    rightBox = false;
    leftBox = false;
    
    this->tofValue1 = tofValue1;
    this->tofValue2 = tofValue2;

    int topLimit = topPoint();
    int topRight = topRightPoint();
    int topLeft = topLeftPoint();
    #ifdef vision_debug_output
    std::cout<<"topLimit: "<<topLimit<<std::endl;
    #endif
    if (topLimit == 99999) return;  // Can't find any path
    if (topLimit <= 5) trackStatus = TrackStatus::Straight;

    bool rightSuccess = startingPointAvoid(topLimit, true);
    bool leftSuccess = startingPointAvoid(topLimit, false);

    if (rightSuccess) edgeDetectionAvoid(true);
    if (leftSuccess && startingPointGuard()) edgeDetectionAvoid(false);

    if (!boxflag){
        if (tofValue1 < 650 || tofValue2 < 650){
            bool useRight = findBestRefEdge();
            if (rightBox) useRight = false;
            if (leftBox) useRight = true;
            if (rightBox && leftBox) useRight = findBestRefEdge();
            if (topLimit > 60) useRight = !findBestRefEdge();
            if (useRight){
                stickRight = 2;
            }
            else{
                stickRight = 0;
            }
            boxflag = true;
            // timer = System::Systick::GetTimeInMS();
            // pit->Start();
        }
    }
    // else {
    //     if (System::Systick::GetTimeInMS() - timer > 7000){
    //         stickRight = 1;
    //         // boxflag = false;
    //     }
    // }
    stickRight = 0;
    // remapAvoid2(topLimit, topRight, topLeft);
    remapAvoid3(topLimit);

    #ifdef vision_debug_output
    std::cout<<"Found target: "<<targetIndex<<" dist: "<<distance<<" steer: "<<angle/PI*180<<std::endl;
    std::cout<<"Track status: ";
    #endif
}

void ComputerVision::conversionAvoidTest() {
    // Reset all variables
    targetIndex = 0;
    angle = 0.0;
    distance = 0.0;
    trackStatus = TrackStatus::NotSure;
    leftEdge.clear();
    rightEdge.clear();
    leftLimit.clear();
    rightLimit.clear();
    midPoints.clear();

    rightBox = false;
    leftBox = false;

    int topLimit = topPoint();
    #ifdef vision_debug_output
    std::cout<<"topLimit: "<<topLimit<<std::endl;
    #endif

    if (topLimit == 99999) return;  // Can't find any path
    if (topLimit <= 5) trackStatus = TrackStatus::Straight;

    bool rightSuccess = startingPointAvoid(topLimit, true);
    bool leftSuccess = startingPointAvoid(topLimit, false);

    if (rightSuccess) edgeDetectionAvoid(true);
    if (leftSuccess && startingPointGuard()) edgeDetectionAvoid(false);

    
    bool useRight = findBestRefEdge();
    if (rightBox) useRight = false;
    if (leftBox) useRight = true;
    if (rightBox && leftBox) useRight = findBestRefEdge();
    if (topLimit > 100) useRight = !findBestRefEdge();
    if (useRight){
        stickRight = 2;
    }
    else{
        stickRight = 0;
    }

    // remapAvoid2(topLimit, topRight, topLeft);
    remapAvoid3(topLimit);

    #ifdef vision_debug_output
    std::cout<<"Found target: "<<targetIndex<<" dist: "<<distance<<" steer: "<<angle/PI*180<<std::endl;
    std::cout<<"Track status: ";
    #endif
}

bool ComputerVision::startingPoint(int topLimit, bool isRightEdge) {
    int halfWidth = RAW_WIDTH / 2;
    if (isRightEdge) {
        // Finding right starting edge
        for (int h=RAW_HEIGHT-2; h>topLimit+7; h-=3) {  // search initial edge from bottom to top limit
            int edgeSum = 0;
            int edgeCount = 0;
            for (int y=h; y>h-5; y--) {
                for (int x=halfWidth; x<RAW_WIDTH-2; x++) {
                    if (isEdge(x, y)) {
                        edgeSum += x;
                        edgeCount++;
                        break;
                    }
                }
            }
            if (edgeCount == 5) {  // When edge is found
                for (int x=edgeSum/edgeCount-7; x<=edgeSum/edgeCount+7&&x>0&&x<RAW_WIDTH-1; x++) {
                    float angle = edgeGradient(x, h-2);
                    if (!std::isnan(angle)) {
                        if (angle > 0) continue;  // avoid incorrect edge detection in the start of crossroad
                        rightEdge.emplace_back((Point) {x, h-2, angle+PI, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found right starting point:"<<std::endl;
                        std::cout<<"x: "<<x<<"    y: "<<h-2<<"    G: "<<angle+PI<<"\n\n";
                        #endif
                        return true;
                    }
                }
                return false;
            }
        }
    } else {
        // Finding left bottom edge
        for (int h=RAW_HEIGHT-2; h>topLimit+7; h-=3) {  // search initial edge from bottom to top limit
            int edgeSum = 0;
            int edgeCount = 0;
            for (int y=h; y>h-5; y--) {
                for (int x=halfWidth; x>1; x--) {
                    if (isEdge(x, y)) {
                        edgeSum += x;
                        edgeCount++;
                        break;
                    }
                }
            }
            if (edgeCount == 5) {  // When edge is found
                for (int x=edgeSum/edgeCount+7; x>=edgeSum/edgeCount-7&&x>0&&x<RAW_WIDTH-1; x--) {
                    float angle = edgeGradient(x, h-2);
                    if (!std::isnan(angle)) {
                        if (angle > 0) continue;  // avoid incorrect edge detection in the start of crossroad
                        leftEdge.emplace_back((Point) {x, h-2, angle, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found left starting point:"<<std::endl;
                        std::cout<<"x: "<<x<<"    y: "<<h-2<<"    G: "<<angle<<"\n\n";
                        #endif
                        return true;
                    }
                }
                return false;
            }
        }
    }
    return false;
}

bool ComputerVision::startingPointAvoid(int topLimit, bool isRightEdge) {
    int halfWidth = RAW_WIDTH / 2;
    if (isRightEdge) {
        // Finding right starting edge
        for (int h=RAW_HEIGHT-5; h>topLimit+7 && h>RAW_HEIGHT-20; h-=3) {  // search initial edge from bottom to top limit
            int edgeSum = 0;
            int edgeCount = 0;
            for (int y=h; y>h-5; y--) {
                for (int x=halfWidth; x<RAW_WIDTH-2; x++) {
                    if (*(cameraPtr+RAW_WIDTH*y+x) < 200 && isEdge(x, y)) {
                        edgeSum += x;
                        edgeCount++;
                        break;
                    }
                }
            }
            if (edgeCount == 5) {  // When edge is found
                for (int x=edgeSum/edgeCount-7; x<=edgeSum/edgeCount+7&&x>0&&x<RAW_WIDTH-1; x++) {
                    float angle = edgeGradient(x, h-2);
                    if (!std::isnan(angle)) {
                        rightEdge.emplace_back((Point) {x, h-2, angle+PI, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found right starting point:"<<std::endl;
                        std::cout<<"x: "<<x<<"    y: "<<h-2<<"    G: "<<angle+PI<<"\n\n";
                        #endif
                        return true;
                    }
                }
                return false;
            }
        }
    } else {
        // Finding left bottom edge
        for (int h=RAW_HEIGHT-5; h>topLimit+7 && h>RAW_HEIGHT-20; h-=3) {  // search initial edge from bottom to top limit
            int edgeSum = 0;
            int edgeCount = 0;
            for (int y=h; y>h-5; y--) {
                for (int x=halfWidth; x>1; x--) {
                    if (*(cameraPtr+RAW_WIDTH*y+x) < 200 && isEdge(x, y)) {
                        edgeSum += x;
                        edgeCount++;
                        break;
                    }
                }
            }
            if (edgeCount == 5) {  // When edge is found
                for (int x=edgeSum/edgeCount+7; x>=edgeSum/edgeCount-7&&x>0&&x<RAW_WIDTH-1; x--) {
                    float angle = edgeGradient(x, h-2);
                    if (!std::isnan(angle)) {
                        leftEdge.emplace_back((Point) {x, h-2, angle, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found left starting point:"<<std::endl;
                        std::cout<<"x: "<<x<<"    y: "<<h-2<<"    G: "<<angle<<"\n\n";
                        #endif
                        return true;
                    }
                }
                return false;
            }
        }
    }
    return false;
}

bool ComputerVision::startingPoint1(int topLimit, bool isRightEdge) {
    int halfWidth = RAW_WIDTH / 2;
    if (isRightEdge) {
        // Finding right starting edge
        for (int h=RAW_HEIGHT-2; h>topLimit+7; h-=3) {  // search initial edge from bottom to top limit
            int edgeSum = 0;
            int edgeCount = 0;
            for (int y=h; y>h-5; y--) {
                for (int x=halfWidth; x<RAW_WIDTH-2; x++) {
                    if (isEdge(x, y)) {
                        edgeSum += x;
                        edgeCount++;
                        break;
                    }
                }
            }
            if (edgeCount == 5) {  // When edge is found
                for (int x=edgeSum/edgeCount-7; x<=edgeSum/edgeCount+7&&x>0&&x<RAW_WIDTH-1; x++) {
                    float angle = edgeGradient(x, h-2);
                    if (!std::isnan(angle)) {
                        rightEdge.emplace_back((Point) {x, h-2, angle, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found right starting point:"<<std::endl;
                        std::cout<<"x: "<<x<<"    y: "<<h-2<<"    G: "<<angle<<"\n\n";
                        #endif
                        return true;
                    }
                }
                return false;
            }
        }
    } else {
        // Finding left bottom edge
        for (int h=RAW_HEIGHT-2; h>topLimit+7; h-=3) {  // search initial edge from bottom to top limit
            int edgeSum = 0;
            int edgeCount = 0;
            for (int y=h; y>h-5; y--) {
                for (int x=halfWidth; x>1; x--) {
                    if (isEdge(x, y)) {
                        edgeSum += x;
                        edgeCount++;
                        break;
                    }
                }
            }
            if (edgeCount == 5) {  // When edge is found
                for (int x=edgeSum/edgeCount+7; x>=edgeSum/edgeCount-7&&x>0&&x<RAW_WIDTH-1; x--) {
                    float angle = edgeGradient(x, h-2);
                    if (!std::isnan(angle)) {
                        leftEdge.emplace_back((Point) {x, h-2, angle, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found left starting point:"<<std::endl;
                        std::cout<<"x: "<<x<<"    y: "<<h-2<<"    G: "<<angle<<"\n\n";
                        #endif
                        return true;
                    }
                }
                return false;
            }
        }
    }
    return false;
}

bool ComputerVision::startingPoint2(int topLimit, bool isRightEdge) {
    int halfWidth = RAW_WIDTH / 2;
    if (isRightEdge) {
        // Finding right starting edge
        for (int h=RAW_HEIGHT-2; h>topLimit+7; h-=3) {  // search initial edge from bottom to top limit
            int edgeSum = 0;
            int edgeCount = 0;
            for (int y=h; y>h-2; y--) {
                for (int x=halfWidth; x<RAW_WIDTH-2; x++) {
                    if (isEdge(x, y)) {
                        edgeSum += x;
                        edgeCount++;
                        break;
                    }
                }
            }
            if (edgeCount == 2) {  // When edge is found
                for (int x=edgeSum/edgeCount-7; x<=edgeSum/edgeCount+7&&x>0&&x<RAW_WIDTH-1; x++) {
                    float angle = edgeGradient(x, h-2);
                    if (!std::isnan(angle)) {
                        if (angle > 0) continue;  // avoid incorrect edge detection in the start of crossroad
                        rightEdge.emplace_back((Point) {x, h, angle+PI, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found right starting point:"<<std::endl;
                        std::cout<<"x: "<<x<<"    y: "<<h-2<<"    G: "<<angle+PI<<"\n\n";
                        #endif
                        return true;
                    }
                }
                return false;
            }
        }
    } else {
        // Finding left bottom edge
        for (int h=RAW_HEIGHT-2; h>topLimit+7; h-=3) {  // search initial edge from bottom to top limit
            int edgeSum = 0;
            int edgeCount = 0;
            for (int y=h; y>h-2; y--) {
                for (int x=halfWidth; x>1; x--) {
                    if (isEdge(x, y)) {
                        edgeSum += x;
                        edgeCount++;
                        break;
                    }
                }
            }
            if (edgeCount == 2) {  // When edge is found
                for (int x=edgeSum/edgeCount+7; x>=edgeSum/edgeCount-7&&x>0&&x<RAW_WIDTH-1; x--) {
                    float angle = edgeGradient(x, h-2);
                    if (!std::isnan(angle)) {
                        if (angle > 0) continue;  // avoid incorrect edge detection in the start of crossroad
                        leftEdge.emplace_back((Point) {x, h, angle, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found left starting point:"<<std::endl;
                        std::cout<<"x: "<<x<<"    y: "<<h-2<<"    G: "<<angle<<"\n\n";
                        #endif
                        return true;
                    }
                }
                return false;
            }
        }
    }
    return false;
}

bool ComputerVision::startingPoint3(int topLimit, bool isRightEdge) {
    int halfWidth = RAW_WIDTH / 2;
    if (isRightEdge) {
        // Finding right starting edge
        for (int h=RAW_HEIGHT-2; h>topLimit+7; h-=3) {  // search initial edge from bottom to top limit
            int edgeSum = 0;
            int edgeCount = 0;
            for (int y=h; y>h-5; y--) {
                for (int x=halfWidth; x<RAW_WIDTH-2; x++) {
                    if (*(cameraPtr + RAW_WIDTH * y + x) < 120) {
                        edgeSum += x;
                        edgeCount++;
                        break;
                    }
                }
            }
            if (edgeCount == 5) {  // When edge is found
                for (int x=edgeSum/edgeCount-7; x<=edgeSum/edgeCount+7&&x>0&&x<RAW_WIDTH-1; x++) {
                    float angle = edgeGradient(x, h-2);
                    if (!std::isnan(angle)) {
                        if (angle > 0) continue;  // avoid incorrect edge detection in the start of crossroad
                        rightEdge.emplace_back((Point) {x, h-2, angle+PI, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found right starting point:"<<std::endl;
                        std::cout<<"x: "<<x<<"    y: "<<h-2<<"    G: "<<angle+PI<<"\n\n";
                        #endif
                        return true;
                    }
                }
                return false;
            }
        }
    } else {
        // Finding left bottom edge
        for (int h=RAW_HEIGHT-2; h>topLimit+7; h-=3) {  // search initial edge from bottom to top limit
            int edgeSum = 0;
            int edgeCount = 0;
            for (int y=h; y>h-5; y--) {
                for (int x=halfWidth; x>1; x--) {
                    if (*(cameraPtr + RAW_WIDTH * y + x) < 120) {
                        edgeSum += x;
                        edgeCount++;
                        break;
                    }
                }
            }
            if (edgeCount == 5) {  // When edge is found
                for (int x=edgeSum/edgeCount+7; x>=edgeSum/edgeCount-7&&x>0&&x<RAW_WIDTH-1; x--) {
                    float angle = edgeGradient(x, h-2);
                    if (!std::isnan(angle)) {
                        if (angle > 0) continue;  // avoid incorrect edge detection in the start of crossroad
                        leftEdge.emplace_back((Point) {x, h-2, angle, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found left starting point:"<<std::endl;
                        std::cout<<"x: "<<x<<"    y: "<<h-2<<"    G: "<<angle<<"\n\n";
                        #endif
                        return true;
                    }
                }
                return false;
            }
        }
    }
    return false;
}

bool ComputerVision::startingPoint8(int topLimit, bool isRightEdge) {
    int edgeSum = 0;
    int edgeCount = 0;
    int fullHeight = RAW_HEIGHT-2;
    int halfWidth = RAW_WIDTH / 2;
    int adjustment = topLimit;
    topLimit=RAW_HEIGHT-topLimit;
    int ang = atan((topLimit)/CAR_HALF_WIDTH) * 180 / PI;//calculate base angle
    int y=0;
    int newX = 0;
    int counter = 1;
    if (isRightEdge) {
        for (int y=fullHeight; y>fullHeight-3; y--) {
            for (int x=halfWidth; x<RAW_WIDTH-2; x++) {
                if (isEdge(x, y)) {
                    edgeSum += x;
                    edgeCount++;
                    break;
                }
            }
        }
        if(edgeCount==3){
            //edge found
            // cout<<"edgeFound1"<<endl;
            for (int x=edgeSum/edgeCount-7; x<=edgeSum/edgeCount+7&&x>0&&x<RAW_WIDTH-1; x++) {  // When edge is found
                for (int x=edgeSum/edgeCount-7; x<=edgeSum/edgeCount+7&&x>0&&x<RAW_WIDTH-1; x++) {
                    float angle = edgeGradient(x, fullHeight-1);
                    if (!std::isnan(angle)) {
                        if (angle > 0) continue;  // avoid incorrect edge detection in the start of crossroad
                        rightEdge.emplace_back((Point) {x, fullHeight-1, angle+PI, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found right starting point:"<<std::endl;
                        std::cout<<"x: "<<x<<"    y: "<<fullHeight-1<<"    G: "<<angle+PI<<"\n\n";
                        #endif
                        return true;
                    }
                }
                // return false;
                ang = atan((topLimit)/(edgeSum/edgeCount + 5 -halfWidth)) * 180 / PI;
                // ang+=5;
                if(ang<=50){
                    counter=2;
                }
            }
        }
        for(;ang>0;ang-=counter){
            if(ang==50){
                counter=2;
            }
            // cout<<"ANG: "<<ang<<endl;
            for(int x=CAR_HALF_WIDTH; x<halfWidth-1 && y <RAW_HEIGHT-1; x++){
                newX = x+halfWidth;
                y = tan( ang * PI / 180.0 ) * x + adjustment;
                // cout<<newX<<" "<<y<<endl;
                if(y>0 && y<RAW_HEIGHT-1 && isEdge(newX,y)){
                    float angle = edgeGradient(newX, y);
                    if (!std::isnan(angle)) {
                        if (angle > 0) continue;  // avoid incorrect edge detection in the start of crossroad
                        rightEdge.emplace_back((Point) {newX, y, angle + PI, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found right starting point:"<<std::endl;
                        std::cout<<"x: "<<newX<<"    y: "<<y<<"    G: "<<angle+PI<<"\n\n";
                        #endif
                        return true;
                    }
                }
            }
            y=0;
        }
        return false;
    } else {
        for (int y=fullHeight; y>fullHeight-3; y--) {
            for (int x=halfWidth; x>2; x--) {
                if (isEdge(x, y)) {
                    edgeSum += x;
                    edgeCount++;
                    break;
                }
            }
        }
        if(edgeCount==3){
            //edge found
            // cout<<"edgeFound1"<<endl;
            for (int x=edgeSum/edgeCount+7; x>=edgeSum/edgeCount-7&&x>0&&x<RAW_WIDTH-1; x--) {
                float angle = edgeGradient(x, fullHeight-1);
                if (!std::isnan(angle)) {
                    if (angle > 0) continue;  // avoid incorrect edge detection in the start of crossroad
                    leftEdge.emplace_back((Point) {x, fullHeight-1, angle, false});
                    #ifdef vision_debug_output
                    std::cout<<"Found left starting point:"<<std::endl;
                    std::cout<<"x: "<<x<<"    y: "<<fullHeight-1<<"    G: "<<angle<<"\n\n";
                    #endif
                    return true;
                }
            }
            // return false;
            ang = atan((topLimit)/(halfWidth - edgeSum/edgeCount-5)) * 180 / PI;
            // cout<<"ANG: "<<ang<<endl;
            // ang+=10;
            if(ang<=50){
                counter=2;
            }
        }
        for(;ang>0;ang-=counter){
            if(ang==50){
                counter=2;
            }
            // cout<<"ANG: "<<ang<<endl;
            for(int x=CAR_HALF_WIDTH; x<halfWidth-1 && y <RAW_HEIGHT-1; x++){
                y = tan( ang * PI / 180.0 ) * x + adjustment;
                newX = halfWidth - x;
                // cout<<newX<<" "<<y<<endl;
                if(y>0 && y<RAW_HEIGHT-1 && isEdge(newX,y)){
                    float angle = edgeGradient(newX, y);
                    if (!std::isnan(angle)) {
                        if (angle > 0) continue;  // avoid incorrect edge detection in the start of crossroad
                        leftEdge.emplace_back((Point) {newX, y, angle, false});
                        #ifdef vision_debug_output
                        std::cout<<"Found left starting point:"<<std::endl;
                        std::cout<<"x: "<<newX<<"    y: "<<y<<"    G: "<<angle<<"\n\n";
                        #endif
                        return true;
                    }
                }
            }
            y=0;
        }
        return false;
    }
}

bool ComputerVision::startingPointGuard() {
    if (rightEdge.empty()) return true;
    if (!leftEdge.empty()) {
        float dist = sqrtf(powf(leftEdge[0].x-rightEdge.back().x, 2) + powf(leftEdge[0].y-rightEdge.back().y, 2));
        if (dist > 20) {
            return true;
        } else {
            if (leftEdge[0].y > rightEdge[0].y) {
                #ifdef vision_debug_output
                std::cout<<"left edge and right edge are identical\nshould be left edge"<<std::endl;
                #endif
                rightEdge.clear();
                return true;
            } else {
                #ifdef vision_debug_output
                std::cout<<"left edge and right edge are identical\nshould be right edge"<<std::endl;
                #endif
                leftEdge.clear();
                return false;
            }
        }
    }
    return false;
}

void ComputerVision::edgeDetection(bool isRightEdge) {
    #ifdef vision_debug_output
    if (isRightEdge) std::cout<<std::endl<<"start right side detection"<<std::endl;
    else std::cout<<std::endl<<"start left side detection"<<std::endl;
    #endif

    int prevX, prevY, newX, newY;
    float prevAngle, shiftAngle, newAngle;
    bool notPassedCrossRoad = true;

    if (isRightEdge) {
        prevX = rightEdge[0].x;
        prevY = rightEdge[0].y;
        prevAngle = rightEdge[0].gradient;
    } else {
        prevX = leftEdge[0].x;
        prevY = leftEdge[0].y;
        prevAngle = leftEdge[0].gradient;
    }

    int count = 0;
    while (count < 256) {
        for (int shiftCount=0; shiftCount<25; shiftCount++) {  // Try to adjust the angle for finding edge
            // Calculate xy-coord and try to find the gradient
            shiftAngle = prevAngle + powf(-1, shiftCount%2)*(PI/24)*((shiftCount+1)/2);
            newX = prevX - 3*sinf(shiftAngle);
            newY = prevY - 3*cosf(shiftAngle);
            if (newX<=1 || newX>=RAW_WIDTH-2 || newY<=1 || newY>=RAW_HEIGHT-2) return;
            if (isRightEdge) {
                newAngle = edgeGradient(newX, newY);
                if (!std::isnan(newAngle)) newAngle += PI;
            } else {
                newAngle = edgeGradient(newX, newY);
            }
            #ifdef vision_debug_output_more
            std::cout<<"-- check1 x:"<<newX<<" y:"<<newY<<" pa:"<<shiftAngle<<" na:"<<newAngle<<std::endl;
            #endif
            // Check whether the gradient is match
            if (!std::isnan(newAngle) && isSameDirection(shiftAngle, newAngle)) {
                // return if it is wrong direction (going backward)
                if (isSimilarAngles(shiftAngle, PI)) return;
                // store the edge to the corresponding array
                if (isRightEdge) {
                    rightEdge.emplace_back((Point) {newX, newY, newAngle, false});
                } else {
                    leftEdge.emplace_back((Point) {newX, newY, newAngle, false});
                }
                prevX = newX;
                prevY = newY;
                prevAngle = newAngle;

                // check whether we are passing cross road
                if (notPassedCrossRoad && isCorner2(isRightEdge)) {
                    if (isRightEdge) {
                        rightEdge[rightEdge.size()-1].isExtraPoint = true;
                        rightEdge[rightEdge.size()-2].isExtraPoint = true;
                        rightEdge[rightEdge.size()-3].isExtraPoint = true;
                        rightEdge[rightEdge.size()-4].isExtraPoint = true;
                    } else {
                        leftEdge[leftEdge.size()-1].isExtraPoint = true;
                        leftEdge[leftEdge.size()-2].isExtraPoint = true;
                        leftEdge[leftEdge.size()-3].isExtraPoint = true;
                        leftEdge[leftEdge.size()-4].isExtraPoint = true;
                    }
                    #ifdef vision_debug_output
                    std::cout<<"found cross road!! x:"<<newX<<" y:"<<newY<<" pa:"<<shiftAngle<<" na:"<<newAngle<<std::endl;
                    #endif

                    if (isCrossRoad(isRightEdge)) {
                        notPassedCrossRoad = false;
                        if (isRightEdge) {
                            prevX = rightEdge[rightEdge.size()-1].x;
                            prevY = rightEdge[rightEdge.size()-1].y;
                            prevAngle = rightEdge[rightEdge.size()-1].gradient;
                        } else {
                            prevX = leftEdge[leftEdge.size()-1].x;
                            prevY = leftEdge[leftEdge.size()-1].y;
                            prevAngle = leftEdge[leftEdge.size()-1].gradient;
                        }
                    } else {
                        return;
                    }
                }

                #ifdef vision_debug_output_more
                std::cout<<"-- hi x:"<<newX<<" y:"<<newY<<" pa:"<<shiftAngle<<" na:"<<newAngle<<std::endl;
                #endif

                break;
            }
        }
        if (std::isnan(newAngle)) {  // Can't find any edge, stop
            #ifdef vision_debug_output
            std::cout<<"stop tracking x:"<<newX<<" y:"<<newY<<std::endl;
            #endif
            break;
        }
        count++;
    }
}

void ComputerVision::edgeDetectionAvoid(bool isRightEdge) {
    #ifdef vision_debug_output
    if (isRightEdge) std::cout<<std::endl<<"start right side detection"<<std::endl;
    else std::cout<<std::endl<<"start left side detection"<<std::endl;
    #endif

    int prevX, prevY, newX, newY;
    float prevAngle, shiftAngle, newAngle;

    if (isRightEdge) {
        prevX = rightEdge[0].x;
        prevY = rightEdge[0].y;
        prevAngle = rightEdge[0].gradient;
    } else {
        prevX = leftEdge[0].x;
        prevY = leftEdge[0].y;
        prevAngle = leftEdge[0].gradient;
    }

    int count = 0;
    while (count < 256) {
        for (int shiftCount=0; shiftCount<25; shiftCount++) {  // Try to adjust the angle for finding edge
            // Calculate xy-coord and try to find the gradient
            shiftAngle = prevAngle + powf(-1, shiftCount%2)*(PI/24)*((shiftCount+1)/2);
            newX = prevX - 3*sinf(shiftAngle);
            newY = prevY - 3*cosf(shiftAngle);
            if (newX<=1 || newX>=RAW_WIDTH-2 || newY<=1 || newY>=RAW_HEIGHT-2) return;
            if (isRightEdge) {
                newAngle = edgeGradient(newX, newY);
                if (!std::isnan(newAngle)) newAngle += PI;
            } else {
                newAngle = edgeGradient(newX, newY);
            }
            #ifdef vision_debug_output_more
            std::cout<<"-- check1 x:"<<newX<<" y:"<<newY<<" pa:"<<shiftAngle<<" na:"<<newAngle<<std::endl;
            #endif
            // Check whether the gradient is match
            if (!std::isnan(newAngle) && isSameDirection(shiftAngle, newAngle)) {
                // return if it is wrong direction (going backward)
                if (isSimilarAngles(shiftAngle, PI)) return;
                // float diff = fabsf(shiftAngle - PI);
                // if (diff >= PI) diff = PI*2 - diff;
                // if (diff < PI/8.0) return;
                // store the edge to the corresponding array

                bool checkCorner = false;
                // if (newY > 30) checkCorner = isCorner2(isRightEdge);
                checkCorner = isCorner2(isRightEdge);

                // if (newY > prevY) return;
                if (checkCorner) {
                    #ifdef vision_debug_output
                    std::cout<<"have corner";
                    if (isRightEdge) std::cout<<" - right"<<std::endl;
                    else std::cout<<" - left"<<std::endl;
                    #endif
                    if (isRightEdge) rightBox = true;
                    else leftBox = true;
                    return;
                }
                if (isRightEdge) {
                    rightEdge.emplace_back((Point) {newX, newY, newAngle, false});
                } else {
                    leftEdge.emplace_back((Point) {newX, newY, newAngle, false});
                }
                prevX = newX;
                prevY = newY;
                prevAngle = newAngle;

                #ifdef vision_debug_output_more
                std::cout<<"-- hi x:"<<newX<<" y:"<<newY<<" pa:"<<shiftAngle<<" na:"<<newAngle<<std::endl;
                #endif

                break;
            }
        }
        if (std::isnan(newAngle)) {  // Can't find any edge, stop
            #ifdef vision_debug_output
            std::cout<<"stop tracking x:"<<newX<<" y:"<<newY<<std::endl;
            #endif
            break;
        }
        count++;
    }
}

void ComputerVision::remap() {
    if (findBestRefEdge()) {  // Use right Edge to be the reference
        #ifdef vision_debug_output
        std::cout<<"use right ref"<<std::endl;
        #endif
        for (size_t i=0; i<rightEdge.size(); i++) {
            if (!rightEdge[i].isExtraPoint) {
                float x = remapImg[rightEdge[i].y][rightEdge[i].x][0];
                float y = remapImg[rightEdge[i].y][rightEdge[i].x][1];
                // Calculate a smoother angle / gradient
                int refFrothIndex = i;
                int refBackIndex = i;
                for (size_t j=i-1; j>=i-3&&j<leftEdge.size(); j--) {
                    if (rightEdge[j].isExtraPoint) break;
                    refBackIndex = j;
                }
                for (size_t j=i+1; j<=i+3&&j<rightEdge.size(); j++) {
                    if (rightEdge[j].isExtraPoint) break;
                    refFrothIndex = j;
                }
                float angle = atan2f(remapImg[rightEdge[refBackIndex].y][rightEdge[refBackIndex].x][0]-remapImg[rightEdge[refFrothIndex].y][rightEdge[refFrothIndex].x][0],
                                     remapImg[rightEdge[refBackIndex].y][rightEdge[refBackIndex].x][1]-remapImg[rightEdge[refFrothIndex].y][rightEdge[refFrothIndex].x][1]);
                // Map both right, left and mid points base on the calculated angle / gradient
                rightLimit.emplace_back(make_pair(x, y));
                leftLimit.emplace_back(make_pair(x-TRACK_WIDTH*cosf(angle), y+TRACK_WIDTH*sinf(angle)));;
                midPoints.emplace_back(make_pair(x-TRACK_HALF_WIDTH*cosf(angle), y+TRACK_HALF_WIDTH*sinf(angle)));
            }
        }
    } else {  // Use left Edge to be the reference
        #ifdef vision_debug_output
        std::cout<<"use left ref"<<std::endl;
        #endif
        for (size_t i=0; i<leftEdge.size(); i++) {
            if (!leftEdge[i].isExtraPoint) {
                float x = remapImg[leftEdge[i].y][leftEdge[i].x][0];
                float y = remapImg[leftEdge[i].y][leftEdge[i].x][1];
                // Calculate a smoother angle / gradient
                int refFrothIndex = i;
                int refBackIndex = i;
                for (size_t j=i-1; j>=i-3&&j<leftEdge.size(); j--) {
                    if (leftEdge[j].isExtraPoint) break;
                    refBackIndex = j;
                }
                for (size_t j=i+1; j<=i+3&&j<leftEdge.size(); j++) {
                    if (leftEdge[j].isExtraPoint) break;
                    refFrothIndex = j;
                }
                float angle = atan2f(remapImg[leftEdge[refBackIndex].y][leftEdge[refBackIndex].x][0]-remapImg[leftEdge[refFrothIndex].y][leftEdge[refFrothIndex].x][0],
                                     remapImg[leftEdge[refBackIndex].y][leftEdge[refBackIndex].x][1]-remapImg[leftEdge[refFrothIndex].y][leftEdge[refFrothIndex].x][1]);
                // Map both right, left and mid points base on the calculated angle / gradient
                leftLimit.emplace_back(make_pair(x, y));
                rightLimit.emplace_back(make_pair(x+TRACK_WIDTH*cosf(angle), y-TRACK_WIDTH*sinf(angle)));;
                midPoints.emplace_back(make_pair(x+TRACK_HALF_WIDTH*cosf(angle), y-TRACK_HALF_WIDTH*sinf(angle)));
            }
        }
    }
}

void ComputerVision::remap1() {
    float x,y,temp_x,temp_y,angle;
    int FrontRef,BackRef;
    if (rightEdge.size()>leftEdge.size() && rightEdge.size()>=3) {
        for (size_t i=0; i<rightEdge.size(); i++) {
            if (!rightEdge[i].isExtraPoint) {
                x = remapImg[rightEdge[i].y][rightEdge[i].x][0];
                y = remapImg[rightEdge[i].y][rightEdge[i].x][1];
                FrontRef = i;
                BackRef = i;
                for (size_t j=i-1; j>=i-3&&j<leftEdge.size(); j--) {
                    if (rightEdge[j].isExtraPoint) break;
                    BackRef = j;
                }
                for (size_t j=i+1; j<=i+3&&j<rightEdge.size(); j++) {
                    if (rightEdge[j].isExtraPoint) break;
                    FrontRef = j;
                }
                angle = atan2f(remapImg[rightEdge[BackRef].y][rightEdge[BackRef].x][0]-remapImg[rightEdge[FrontRef].y][rightEdge[FrontRef].x][0],
                                remapImg[rightEdge[BackRef].y][rightEdge[BackRef].x][1]-remapImg[rightEdge[FrontRef].y][rightEdge[FrontRef].x][1]);
                temp_x = x - TRACK_ONE_QUARTER_WIDTH*cosf(angle);
                temp_y = y + TRACK_ONE_QUARTER_WIDTH*sinf(angle);
                rightLimit.emplace_back(make_pair(temp_x,temp_y));
                temp_x = x - TRACK_HALF_WIDTH*cosf(angle);
                temp_y = y + TRACK_HALF_WIDTH*sinf(angle);
                midPoints.emplace_back(make_pair(temp_x,temp_y));
                temp_x = x - TRACK_THREE_QUARTER_WIDTH*cosf(angle);
                temp_y = y + TRACK_THREE_QUARTER_WIDTH*sinf(angle);
                leftLimit.emplace_back(make_pair(temp_x,temp_y));
            }
        }
    }
    else {
        for (size_t i=0;i<leftEdge.size(); i++) {
            if (!leftEdge[i].isExtraPoint) {
                x = remapImg[leftEdge[i].y][leftEdge[i].x][0];
                y = remapImg[leftEdge[i].y][leftEdge[i].x][1];
                FrontRef = i;
                BackRef = i;
                for (size_t j=i-1; j>=i-3&&j<leftEdge.size(); j--) {
                    if (leftEdge[j].isExtraPoint) break;
                    BackRef = j;
                }
                for (size_t j=i+1; j<=i+3&&j<leftEdge.size(); j++) {
                    if (leftEdge[j].isExtraPoint) break;
                    FrontRef = j;
                }
                angle = atan2f(remapImg[leftEdge[BackRef].y][leftEdge[BackRef].x][0]-remapImg[leftEdge[FrontRef].y][leftEdge[FrontRef].x][0],
                             remapImg[leftEdge[BackRef].y][leftEdge[BackRef].x][1]-remapImg[leftEdge[FrontRef].y][leftEdge[FrontRef].x][1]);
                temp_x = x + TRACK_THREE_QUARTER_WIDTH*cosf(angle);
                temp_y = y - TRACK_THREE_QUARTER_WIDTH*sinf(angle);
                rightLimit.emplace_back(make_pair(temp_x,temp_y));
                temp_x = x + TRACK_HALF_WIDTH*cosf(angle);
                temp_y = y - TRACK_HALF_WIDTH*sinf(angle);
                midPoints.emplace_back(make_pair(temp_x,temp_y));
                temp_x = x + TRACK_ONE_QUARTER_WIDTH*cosf(angle);
                temp_y = y - TRACK_ONE_QUARTER_WIDTH*sinf(angle);
                leftLimit.emplace_back(make_pair(temp_x,temp_y));
            }
        }
    }
}

void ComputerVision::remap2() {
    if (findBestRefEdge()) {  // Use right Edge to be the reference
        #ifdef vision_debug_output
        std::cout<<"use right ref"<<std::endl;
        #endif
        for (size_t i=0; i<rightEdge.size(); i++) {
            if (!rightEdge[i].isExtraPoint) {
                float x = remapImg[rightEdge[i].y][rightEdge[i].x][0];
                float y = remapImg[rightEdge[i].y][rightEdge[i].x][1];
                // Calculate a smoother angle / gradient
                int refFrothIndex = i;
                int refBackIndex = i;
                for (size_t j=i-1; j>=i-3&&j<leftEdge.size(); j--) {
                    if (rightEdge[j].isExtraPoint) break;
                    refBackIndex = j;
                }
                for (size_t j=i+1; j<=i+3&&j<rightEdge.size(); j++) {
                    if (rightEdge[j].isExtraPoint) break;
                    refFrothIndex = j;
                }
                float angle = atan2f(remapImg[rightEdge[refBackIndex].y][rightEdge[refBackIndex].x][0]-remapImg[rightEdge[refFrothIndex].y][rightEdge[refFrothIndex].x][0],
                                     remapImg[rightEdge[refBackIndex].y][rightEdge[refBackIndex].x][1]-remapImg[rightEdge[refFrothIndex].y][rightEdge[refFrothIndex].x][1]);
                // Map both right and mid points base on the calculated angle / gradient
                rightLimit.emplace_back(make_pair(x, y));
                midPoints.emplace_back(make_pair(x-TRACK_HALF_WIDTH*cosf(angle), y+TRACK_HALF_WIDTH*sinf(angle)));
            }
        }
        for (size_t i=0; i<leftEdge.size(); i++) {
            if (!leftEdge[i].isExtraPoint) {
                float x = remapImg[leftEdge[i].y][leftEdge[i].x][0];
                float y = remapImg[leftEdge[i].y][leftEdge[i].x][1];
                leftLimit.emplace_back(make_pair(x, y));
            }
        }
    } else {  // Use left Edge to be the reference
        #ifdef vision_debug_output
        std::cout<<"use left ref"<<std::endl;
        #endif
        for (size_t i=0; i<leftEdge.size(); i++) {
            if (!leftEdge[i].isExtraPoint) {
                float x = remapImg[leftEdge[i].y][leftEdge[i].x][0];
                float y = remapImg[leftEdge[i].y][leftEdge[i].x][1];
                // Calculate a smoother angle / gradient
                int refFrothIndex = i;
                int refBackIndex = i;
                for (size_t j=i-1; j>=i-3&&j<leftEdge.size(); j--) {
                    if (leftEdge[j].isExtraPoint) break;
                    refBackIndex = j;
                }
                for (size_t j=i+1; j<=i+3&&j<leftEdge.size(); j++) {
                    if (leftEdge[j].isExtraPoint) break;
                    refFrothIndex = j;
                }
                float angle = atan2f(remapImg[leftEdge[refBackIndex].y][leftEdge[refBackIndex].x][0]-remapImg[leftEdge[refFrothIndex].y][leftEdge[refFrothIndex].x][0],
                                     remapImg[leftEdge[refBackIndex].y][leftEdge[refBackIndex].x][1]-remapImg[leftEdge[refFrothIndex].y][leftEdge[refFrothIndex].x][1]);
                // Map both left and mid points base on the calculated angle / gradient
                leftLimit.emplace_back(make_pair(x, y));
                midPoints.emplace_back(make_pair(x+TRACK_HALF_WIDTH*cosf(angle), y-TRACK_HALF_WIDTH*sinf(angle)));
            }
        }
        for (size_t i=0; i<rightEdge.size(); i++) {
            if (!rightEdge[i].isExtraPoint) {
                float x = remapImg[rightEdge[i].y][rightEdge[i].x][0];
                float y = remapImg[rightEdge[i].y][rightEdge[i].x][1];
                rightLimit.emplace_back(make_pair(x, y));
            }
        }
    }
}

void ComputerVision::remap3() {
    float remapYControl = 420.0;
    if (findBestRefEdge()) {  // Use right Edge to be the reference
        #ifdef vision_debug_output
        std::cout<<"use right ref"<<std::endl;
        #endif
        for (size_t i=0; i<rightEdge.size(); i++) {
            if (!rightEdge[i].isExtraPoint) {
                float x = remapImg[rightEdge[i].y][rightEdge[i].x][0];
                float y = remapImg[rightEdge[i].y][rightEdge[i].x][1];
                // Calculate a smoother angle / gradient
                int refFrothIndex = i;
                int refBackIndex = i;
                for (size_t j=i-1; j>=i-3&&j<leftEdge.size(); j--) {
                    if (rightEdge[j].isExtraPoint) break;
                    refBackIndex = j;
                }
                for (size_t j=i+1; j<=i+3&&j<rightEdge.size(); j++) {
                    if (rightEdge[j].isExtraPoint) break;
                    refFrothIndex = j;
                }
                float angle = atan2f(remapImg[rightEdge[refBackIndex].y][rightEdge[refBackIndex].x][0]-remapImg[rightEdge[refFrothIndex].y][rightEdge[refFrothIndex].x][0],
                                     remapImg[rightEdge[refBackIndex].y][rightEdge[refBackIndex].x][1]-remapImg[rightEdge[refFrothIndex].y][rightEdge[refFrothIndex].x][1]);
                // Map both right, left and mid points base on the calculated angle / gradient
                rightLimit.emplace_back(make_pair(x, y));
                midPoints.emplace_back(make_pair(x-TRACK_HALF_WIDTH*cosf(angle), y+TRACK_HALF_WIDTH*sinf(angle)));
                float newY = y+TRACK_WIDTH*sinf(angle);
                if (newY <= remapYControl) {
                    leftLimit.emplace_back(make_pair(x-TRACK_WIDTH*cosf(angle), newY));
                    remapYControl = newY;
                }
            }
        }
    } else {  // Use left Edge to be the reference
        #ifdef vision_debug_output
        std::cout<<"use left ref"<<std::endl;
        #endif
        for (size_t i=0; i<leftEdge.size(); i++) {
            if (!leftEdge[i].isExtraPoint) {
                float x = remapImg[leftEdge[i].y][leftEdge[i].x][0];
                float y = remapImg[leftEdge[i].y][leftEdge[i].x][1];
                // Calculate a smoother angle / gradient
                int refFrothIndex = i;
                int refBackIndex = i;
                for (size_t j=i-1; j>=i-3&&j<leftEdge.size(); j--) {
                    if (leftEdge[j].isExtraPoint) break;
                    refBackIndex = j;
                }
                for (size_t j=i+1; j<=i+3&&j<leftEdge.size(); j++) {
                    if (leftEdge[j].isExtraPoint) break;
                    refFrothIndex = j;
                }
                float angle = atan2f(remapImg[leftEdge[refBackIndex].y][leftEdge[refBackIndex].x][0]-remapImg[leftEdge[refFrothIndex].y][leftEdge[refFrothIndex].x][0],
                                     remapImg[leftEdge[refBackIndex].y][leftEdge[refBackIndex].x][1]-remapImg[leftEdge[refFrothIndex].y][leftEdge[refFrothIndex].x][1]);
                // Map both right, left and mid points base on the calculated angle / gradient
                leftLimit.emplace_back(make_pair(x, y));
                midPoints.emplace_back(make_pair(x+TRACK_HALF_WIDTH*cosf(angle), y-TRACK_HALF_WIDTH*sinf(angle)));
                float newY = y-TRACK_WIDTH*sinf(angle);
                if (newY <= remapYControl) {
                    rightLimit.emplace_back(make_pair(x+TRACK_WIDTH*cosf(angle), newY));
                    remapYControl = newY;
                }
            }
        }
    }
}

void ComputerVision::remapAvoid() {

    if(rightEdge.size() < 3 && leftEdge.size() < 3){
        this->distance=0;
        this->angle=0;
        return;
    }

    useRight = findBestRefEdge();
    if (rightBox) useRight = false;
    if (leftBox) useRight = true;
    if (rightBox && leftBox) useRight = findBestRefEdge();

    // bool useRight = true;
    // if (rightBox) useRight = false;
    // if (rightBox) stickRight = false;

    if (useRight) {  // Use right Edge to be the reference
        #ifdef vision_debug_output
        std::cout<<"************** use right ref _ avoid **************"<<std::endl;
        #endif
        
        size_t i=(rightEdge.size()+1) /2;
        i = 0;
        if (i < rightEdge.size()) {
            float x = remapImg[rightEdge[i].y][rightEdge[i].x][0];
            float y = remapImg[rightEdge[i].y][rightEdge[i].x][1];

            int rightX = x - CAR_HALF_WIDTH;
            int rightY = y;

            midPoints.emplace_back(make_pair(rightX, rightY));

            this->distance = sqrtf(powf(CAR_REAL_Y_REAR-rightY, 2) + powf(CAR_REAL_X-rightX, 2));
            this->angle = atanf((CAR_REAL_X-rightX) / (CAR_REAL_Y_FRONT-rightY));
            this->targetIndex = i;
        } else {
            float x = remapImg[leftEdge[i].y][leftEdge[i].x][0];
            float y = remapImg[leftEdge[i].y][leftEdge[i].x][1];
            // float y = remapImg[rightEdge[Ri].y][rightEdge[Ri].x][1];

            int leftX = x + TRACK_WIDTH;
            int leftY = y;

            midPoints.emplace_back(make_pair(leftX, leftY));

            this->distance = sqrtf(powf(CAR_REAL_Y_REAR-leftY, 2) + powf(CAR_REAL_X-leftX, 2));
            this->angle = atanf((CAR_REAL_X-leftX) / (CAR_REAL_Y_FRONT-leftY));
            this->targetIndex = i;

        }
    } else {  // Use left Edge to be the reference
        #ifdef vision_debug_output
        std::cout<<"************** use left ref _ avoid **************!!!"<<std::endl;
        #endif
        int Ri=(rightEdge.size()+1) /2;
        // if (i < rightEdge.size()) {
        //     float x = remapImg[rightEdge[i].y][rightEdge[i].x][0];
        //     float y = remapImg[rightEdge[i].y][rightEdge[i].x][1];

        //     int rightX = x - TRACK_WIDTH + CAR_HALF_WIDTH;
        //     int rightY = y;

        //     this->distance = sqrtf(powf(CAR_REAL_Y_REAR-rightY, 2) + powf(CAR_REAL_X-rightX, 2));
        //     this->angle = atanf((CAR_REAL_X-rightX) / (CAR_REAL_Y_FRONT-rightY));
        //     this->targetIndex = i;
        // }

        size_t i=(leftEdge.size()+1) /2;
        i = 0;
        // if (i < leftEdge.size() && Ri < rightEdge.size()) {
        if (i < leftEdge.size()) {
            float x = remapImg[leftEdge[i].y][leftEdge[i].x][0];
            float y = remapImg[leftEdge[i].y][leftEdge[i].x][1];
            // float y = remapImg[rightEdge[Ri].y][rightEdge[Ri].x][1];

            int leftX = x + CAR_HALF_WIDTH;
            int leftY = y;

            midPoints.emplace_back(make_pair(leftX, leftY));

            this->distance = sqrtf(powf(CAR_REAL_Y_REAR-leftY, 2) + powf(CAR_REAL_X-leftX, 2));
            this->angle = atanf((CAR_REAL_X-leftX) / (CAR_REAL_Y_FRONT-leftY));
            this->targetIndex = i;
        } else {
            float x = remapImg[rightEdge[i].y][rightEdge[i].x][0];
            float y = remapImg[rightEdge[i].y][rightEdge[i].x][1];

            int rightX = x - TRACK_WIDTH;
            int rightY = y;

            midPoints.emplace_back(make_pair(rightX, rightY));

            this->distance = sqrtf(powf(CAR_REAL_Y_REAR-rightY, 2) + powf(CAR_REAL_X-rightX, 2));
            this->angle = atanf((CAR_REAL_X-rightX) / (CAR_REAL_Y_FRONT-rightY));
            this->targetIndex = i;
        }
    }
}

void ComputerVision::remapAvoid2(int topLimit, int topRight, int topLeft) {

    if(rightEdge.size() < 3 && leftEdge.size() < 3){
        this->distance=0;
        this->angle=0;
        return;
    }
    // uint8_t whichEdge = 1;
    // if(whichEdge==2){
    //     if(leftEdge.size()>0){
    //         whichEdge=1;
    //     }
    // }
    // else if (whichEdge==0){
    //     if(rightEdge.size()>0){
    //         whichEdge=1;
    //     }
    // } else {
    //     whichEdge=1;
    //     if (rightBox) whichEdge = 0;
    //     if (leftBox) whichEdge = 2;
    //     if (rightBox && leftBox) whichEdge = 1;

    // }
    // bool useRight = findBestRefEdge();
    // if (rightBox) whichEdge = 0;
    // if (leftBox) whichEdge = 2;
    // if (rightBox && leftBox) whichEdge = 1;

    // if (topLimit > 55){
    //     if(topRight>topLeft){
    //         useRight= false;
    //     } else {
    //         useRight = true;
    //     }
    // }
    
    // bool useRight = true;
    // if (rightBox) useRight = false;
    // if (rightBox) stickRight = false;

    // stickRight = 1;

    if (stickRight == 2) {  // Use right Edge to be the reference
        #ifdef vision_debug_output
        std::cout<<"************** use right ref _ avoid **************"<<std::endl;
        #endif
        
        size_t i=(rightEdge.size()+1) /2;
        i = 0;
        if (i < rightEdge.size()) {
            float x = remapImg[rightEdge[i].y][rightEdge[i].x][0];
            float y = remapImg[rightEdge[i].y][rightEdge[i].x][1];

            int rightX = x - CAR_HALF_WIDTH;
            int rightY = y;

            midPoints.emplace_back(make_pair(rightX, rightY));

            this->distance = sqrtf(powf(CAR_REAL_Y_REAR-rightY, 2) + powf(CAR_REAL_X-rightX, 2));
            this->angle = atanf((CAR_REAL_X-rightX) / (CAR_REAL_Y_FRONT-rightY));
            this->targetIndex = i;
        } 
        else {
            if (i < leftEdge.size()) {
                float x = remapImg[leftEdge[i].y][leftEdge[i].x][0];
                float y = remapImg[leftEdge[i].y][leftEdge[i].x][1];
                // float y = remapImg[rightEdge[Ri].y][rightEdge[Ri].x][1];

                int leftX = x + TRACK_WIDTH;
                int leftY = y;

                midPoints.emplace_back(make_pair(leftX, leftY));

                this->distance = sqrtf(powf(CAR_REAL_Y_REAR-leftY, 2) + powf(CAR_REAL_X-leftX, 2));
                this->angle = atanf((CAR_REAL_X-leftX) / (CAR_REAL_Y_FRONT-leftY));
                this->targetIndex = i;
            }
        }
    } 
    if (stickRight == 0) {  // Use left Edge to be the reference
        #ifdef vision_debug_output
        std::cout<<"************** use left ref _ avoid **************!!!"<<std::endl;
        #endif
        int Ri=(rightEdge.size()+1) /2;
        // if (i < rightEdge.size()) {
        //     float x = remapImg[rightEdge[i].y][rightEdge[i].x][0];
        //     float y = remapImg[rightEdge[i].y][rightEdge[i].x][1];

        //     int rightX = x - TRACK_WIDTH + CAR_HALF_WIDTH;
        //     int rightY = y;

        //     this->distance = sqrtf(powf(CAR_REAL_Y_REAR-rightY, 2) + powf(CAR_REAL_X-rightX, 2));
        //     this->angle = atanf((CAR_REAL_X-rightX) / (CAR_REAL_Y_FRONT-rightY));
        //     this->targetIndex = i;
        // }

        size_t i=(leftEdge.size()+1) /2;
        i = 0;
        // if (i < leftEdge.size() && Ri < rightEdge.size()) {
        if (i < leftEdge.size()) {
            float x = remapImg[leftEdge[i].y][leftEdge[i].x][0];
            float y = remapImg[leftEdge[i].y][leftEdge[i].x][1];
            // float y = remapImg[rightEdge[Ri].y][rightEdge[Ri].x][1];

            int leftX = x + CAR_HALF_WIDTH;
            int leftY = y;

            midPoints.emplace_back(make_pair(leftX, leftY));

            this->distance = sqrtf(powf(CAR_REAL_Y_REAR-leftY, 2) + powf(CAR_REAL_X-leftX, 2));
            this->angle = atanf((CAR_REAL_X-leftX) / (CAR_REAL_Y_FRONT-leftY));
            this->targetIndex = i;
        } else {
            if (i < rightEdge.size()) {
                float x = remapImg[rightEdge[i].y][rightEdge[i].x][0];
                float y = remapImg[rightEdge[i].y][rightEdge[i].x][1];

                int rightX = x - TRACK_WIDTH;
                int rightY = y;

                midPoints.emplace_back(make_pair(rightX, rightY));

                this->distance = sqrtf(powf(CAR_REAL_Y_REAR-rightY, 2) + powf(CAR_REAL_X-rightX, 2));
                this->angle = atanf((CAR_REAL_X-rightX) / (CAR_REAL_Y_FRONT-rightY));
                this->targetIndex = i;
            }
        }
    }
    if (stickRight == 1){  // (stick middle)
        if (leftEdge.size() >= 3) {
            float angle = atan2f(remapImg[leftEdge[0].y][leftEdge[0].x][0]-remapImg[leftEdge[2].y][leftEdge[2].x][0],
                                 remapImg[leftEdge[0].y][leftEdge[0].x][1]-remapImg[leftEdge[2].y][leftEdge[2].x][1]);
            float x = remapImg[leftEdge[1].y][leftEdge[1].x][0] + TRACK_HALF_WIDTH*cosf(angle);
            float y = remapImg[leftEdge[1].y][leftEdge[1].x][1] - TRACK_HALF_WIDTH*sinf(angle);
            midPoints.emplace_back(make_pair(x, y));

            this->distance = sqrtf(powf(CAR_REAL_Y_REAR-y, 2) + powf(CAR_REAL_X-x, 2));
            this->angle = atanf((CAR_REAL_X-x) / (CAR_REAL_Y_MIDDLE-y));
            this->targetIndex = 0;
        } else {
            if (rightEdge.size() >= 3) {
                float angle = atan2f(remapImg[rightEdge[0].y][rightEdge[0].x][0]-remapImg[rightEdge[2].y][rightEdge[2].x][0],
                                    remapImg[rightEdge[0].y][rightEdge[0].x][1]-remapImg[rightEdge[2].y][rightEdge[2].x][1]);
                float x = remapImg[rightEdge[1].y][rightEdge[1].x][0] - TRACK_HALF_WIDTH*cosf(angle);
                float y = remapImg[rightEdge[1].y][rightEdge[1].x][1] + TRACK_HALF_WIDTH*sinf(angle);
                midPoints.emplace_back(make_pair(x, y));

                this->distance = sqrtf(powf(CAR_REAL_Y_REAR-y, 2) + powf(CAR_REAL_X-x, 2));
                this->angle = atanf((CAR_REAL_X-x) / (CAR_REAL_Y_MIDDLE-y));
                this->targetIndex = 0;
            }
        }
    }
}

void ComputerVision::remapAvoid3(int topLimit) {
    int avoidWidth = CAR_HALF_WIDTH;

    int topRight, topLeft;
    // stickRight == 0;
    if (stickRight == 2) {  // Use right Edge to be the reference (stick right)
        #ifdef vision_debug_output
        std::cout<<"************** use right ref _ avoid **************"<<std::endl;
        #endif

        if (rightEdge.size() >= 3) {
            float angle = atan2f(remapImg[rightEdge[0].y][rightEdge[0].x][0]-remapImg[rightEdge[2].y][rightEdge[2].x][0],
                                remapImg[rightEdge[0].y][rightEdge[0].x][1]-remapImg[rightEdge[2].y][rightEdge[2].x][1]);
            float x = remapImg[rightEdge[1].y][rightEdge[1].x][0] - avoidWidth*cosf(angle);
            float y = remapImg[rightEdge[1].y][rightEdge[1].x][1] + avoidWidth*sinf(angle);
            if (y >= CAR_REAL_Y_FRONT-5) y = CAR_REAL_Y_FRONT-5;
            midPoints.emplace_back(make_pair(x, y));

            this->distance = 50;
            this->angle = atanf((CAR_REAL_X-x) / (CAR_REAL_Y_FRONT-y));
            this->targetIndex = 0;
        } else if (leftEdge.size() >= 3) {
            float angle = atan2f(remapImg[leftEdge[0].y][leftEdge[0].x][0]-remapImg[leftEdge[2].y][leftEdge[2].x][0],
                                remapImg[leftEdge[0].y][leftEdge[0].x][1]-remapImg[leftEdge[2].y][leftEdge[2].x][1]);
            float x = remapImg[leftEdge[1].y][leftEdge[1].x][0] + (TRACK_WIDTH-avoidWidth)*cosf(angle);
            float y = remapImg[leftEdge[1].y][leftEdge[1].x][1] - (TRACK_WIDTH-avoidWidth)*sinf(angle);
            if (y >= CAR_REAL_Y_FRONT-5) y = CAR_REAL_Y_FRONT-5;
            midPoints.emplace_back(make_pair(x, y));

            this->distance = 50;
            this->angle = atanf((CAR_REAL_X-x) / (CAR_REAL_Y_FRONT-y));
            this->targetIndex = 0;
        }
    } 
    if (stickRight == 0) {  // Use left Edge to be the reference (stick left)
        #ifdef vision_debug_output
        std::cout<<"************** use left ref _ avoid **************!!!"<<std::endl;
        #endif

        if (leftEdge.size() >= 3) {
            
            std::cout<<"*****dfh********* use left ref _ avoid **************!!!"<<std::endl;
            float angle = atan2f(remapImg[leftEdge[0].y][leftEdge[0].x][0]-remapImg[leftEdge[2].y][leftEdge[2].x][0],
                                 remapImg[leftEdge[0].y][leftEdge[0].x][1]-remapImg[leftEdge[2].y][leftEdge[2].x][1]);
            float x = remapImg[leftEdge[1].y][leftEdge[1].x][0] + avoidWidth*cosf(angle);
            float y = remapImg[leftEdge[1].y][leftEdge[1].x][1] - avoidWidth*sinf(angle);
            if (y >= CAR_REAL_Y_FRONT-5) y = CAR_REAL_Y_FRONT-5;
            midPoints.emplace_back(make_pair(x, y));

            this->distance = 50;
            this->angle = atanf((CAR_REAL_X-x) / (CAR_REAL_Y_FRONT-y));
            this->targetIndex = 0;
        } else if (rightEdge.size() >= 3) {
            
            std::cout<<"*****agd********* use left ref _ avoid **************!!!"<<std::endl;
            float angle = atan2f(remapImg[rightEdge[0].y][rightEdge[0].x][0]-remapImg[rightEdge[2].y][rightEdge[2].x][0],
                                remapImg[rightEdge[0].y][rightEdge[0].x][1]-remapImg[rightEdge[2].y][rightEdge[2].x][1]);
            float x = remapImg[rightEdge[1].y][rightEdge[1].x][0] - (TRACK_WIDTH-avoidWidth)*cosf(angle);
            float y = remapImg[rightEdge[1].y][rightEdge[1].x][1] + (TRACK_WIDTH-avoidWidth)*sinf(angle);
            if (y >= CAR_REAL_Y_FRONT-5) y = CAR_REAL_Y_FRONT-5;
            midPoints.emplace_back(make_pair(x, y));

            this->distance = 50;
            this->angle = atanf((CAR_REAL_X-x) / (CAR_REAL_Y_FRONT-y));
            this->targetIndex = 0;
        }
    } 
    if (stickRight == 1){  // (stick middle)
        if (rightEdge.size() >= 3) {
            float angle = atan2f(remapImg[rightEdge[0].y][rightEdge[0].x][0]-remapImg[rightEdge[2].y][rightEdge[2].x][0],
                                remapImg[rightEdge[0].y][rightEdge[0].x][1]-remapImg[rightEdge[2].y][rightEdge[2].x][1]);
            float x = remapImg[rightEdge[1].y][rightEdge[1].x][0] - (TRACK_HALF_WIDTH+5)*cosf(angle);
            float y = remapImg[rightEdge[1].y][rightEdge[1].x][1] + (TRACK_HALF_WIDTH+5)*sinf(angle);
            if (y >= CAR_REAL_Y_FRONT-5) y = CAR_REAL_Y_FRONT-5;
            midPoints.emplace_back(make_pair(x, y));

            this->distance = 50;
            this->angle = atanf((CAR_REAL_X-x) / (CAR_REAL_Y_FRONT-y));
            this->targetIndex = 0;
        } else if (leftEdge.size() >= 3) {
            float angle = atan2f(remapImg[leftEdge[0].y][leftEdge[0].x][0]-remapImg[leftEdge[2].y][leftEdge[2].x][0],
                                 remapImg[leftEdge[0].y][leftEdge[0].x][1]-remapImg[leftEdge[2].y][leftEdge[2].x][1]);
            float x = remapImg[leftEdge[1].y][leftEdge[1].x][0] + (TRACK_HALF_WIDTH-5)*cosf(angle);
            float y = remapImg[leftEdge[1].y][leftEdge[1].x][1] - (TRACK_HALF_WIDTH-5)*sinf(angle);
            if (y >= CAR_REAL_Y_FRONT-5) y = CAR_REAL_Y_FRONT-5;
            midPoints.emplace_back(make_pair(x, y));

            this->distance = 50;
            this->angle = atanf((CAR_REAL_X-x) / (CAR_REAL_Y_FRONT-y));
            this->targetIndex = 0;
        }
    }
}

void ComputerVision::pathPlanning() {
    float para_car_half_width_small = PP01_CAR_HALF_WIDTH_SMALL;  //9.5
    float para_car_half_width_large = PP01_CAR_HALF_WIDTH_LARGE;  // 17
    float para_switch_height = PP01_SWITCH_HEIGHT;  // bottom - 6*track_width/4

    targetIndex = 0;

    int hitEdge = 0;  // number of time edge point is hitted (to reduce noise)

    if (midPoints.size() > 2) {
        for (int i=midPoints.size()-1; i>=0; i--) { // search through all possible case
            hitEdge = 0;

            float car_half_width = midPoints[i].second > para_switch_height ? para_car_half_width_large : para_car_half_width_small;

            if (midPoints[i].first == CAR_REAL_X) {  // special case (vertical line)
                for (size_t j=0; j<leftLimit.size(); j++) {
                    if (leftLimit[j].second < CAR_REAL_Y-cutoff) {
                        if (leftLimit[j].second >= midPoints[i].second && leftLimit[j].first >= CAR_REAL_X-car_half_width) {
                            hitEdge++;
                            if (hitEdge >= 3) break;
                        }
                    }
                }
                if (hitEdge >= 3) continue;
                for (size_t j=0; j<rightLimit.size(); j++) {
                    if (rightLimit[j].second < CAR_REAL_Y-cutoff) {
                        if (rightLimit[j].second >= midPoints[i].second && rightLimit[j].first <= CAR_REAL_X+car_half_width) {
                            hitEdge++;
                            if (hitEdge >= 3) break;
                        }
                    }
                }
                if (hitEdge >= 3) continue;
                
                this->distance = CAR_REAL_Y_REAR - midPoints[i].second;
                this->angle = 0.0;
                this->targetIndex = i;
                return;
            } else {
                float m = (CAR_REAL_Y_REAR-midPoints[i].second) / (CAR_REAL_X-midPoints[i].first);  // slope of the path
                float c = CAR_REAL_Y_REAR - m*CAR_REAL_X;  // y-intercept of the path
                float angle = atanf(1.0/m);  // steering angle
                float bond = car_half_width/sinf(fabsf(angle));  // y-intercept offset of side boundaries with the path
                float mLimit = -1.0/m;  // slope of front & back boundaries
                float cLimitF = midPoints[i].second - mLimit*midPoints[i].first;  // y-intercept of front boundary
                float cLimitB = CAR_REAL_Y_REAR - mLimit*CAR_REAL_X; // y-intercept of back boundary
                if (midPoints[i].first < CAR_REAL_X) {
                    for (size_t j=0; j<leftLimit.size(); j++) {
                        if (leftLimit[j].second < CAR_REAL_Y-cutoff) {
                            if (leftLimit[j].second >= mLimit*leftLimit[j].first+cLimitF
                                && leftLimit[j].second <= mLimit*leftLimit[j].first+cLimitB
                                && leftLimit[j].second <= m*leftLimit[j].first+c+bond) {
                                hitEdge++;
                                if (hitEdge >= 3) break;
                            }
                        }
                    }
                    if (hitEdge >= 3) continue;
                    for (size_t j=0; j<rightLimit.size(); j++) {
                        if (rightLimit[j].second < CAR_REAL_Y-cutoff) {
                            if (rightLimit[j].second >= mLimit*rightLimit[j].first+cLimitF
                                && rightLimit[j].second <= mLimit*rightLimit[j].first+cLimitB
                                && rightLimit[j].second >= m*rightLimit[j].first+c-bond) {
                                hitEdge++;
                                if (hitEdge >= 3) break;
                            }
                        }
                    }
                    if (hitEdge >= 3) continue;
                } else {
                    for (size_t j=0; j<leftLimit.size(); j++) {
                        if (leftLimit[j].second < CAR_REAL_Y-cutoff) {
                            if (leftLimit[j].second >= mLimit*leftLimit[j].first+cLimitF
                                && leftLimit[j].second <= mLimit*leftLimit[j].first+cLimitB
                                && leftLimit[j].second >= m*leftLimit[j].first+c-bond) {
                                hitEdge++;
                                if (hitEdge >= 3) break;
                            }
                        }
                    }
                    if (hitEdge >= 3) continue;
                    for (size_t j=0; j<rightLimit.size(); j++) {
                        if (rightLimit[j].second < CAR_REAL_Y-cutoff) {
                            if (rightLimit[j].second >= mLimit*rightLimit[j].first+cLimitF
                                && rightLimit[j].second <= mLimit*rightLimit[j].first+cLimitB
                                && rightLimit[j].second <= m*rightLimit[j].first+c+bond) {
                                hitEdge++;
                                if (hitEdge >= 3) break;
                            }
                        }
                    }
                    if (hitEdge >= 3) continue;
                }
                
                // i = i * 3 / 4;

                this->distance = sqrtf(powf(CAR_REAL_Y_REAR-midPoints[i].second, 2) + powf(CAR_REAL_X-midPoints[i].first, 2));
                this->angle = atanf((CAR_REAL_X-midPoints[i].first) / (CAR_REAL_Y_FRONT-midPoints[i].second));
                this->targetIndex = i;
                return;
            }
        }
    }
    this->distance = 0.0;
    this->angle = 0.0;
    this->targetIndex = 0;
}

void ComputerVision::pathPlanningWithStatus() {
    if (this->distance != 0.0) {
        float offset = 10.0;
        if (trackStatus == TrackStatus::LeftAhead) {
            this->distance = sqrtf(powf(CAR_REAL_Y_REAR-midPoints[targetIndex].second, 2) + powf(CAR_REAL_X-(midPoints[targetIndex].first+offset), 2));
            this->angle = atanf((CAR_REAL_X-(midPoints[targetIndex].first+offset)) / (CAR_REAL_Y_FRONT-midPoints[targetIndex].second));
        } else if (trackStatus == TrackStatus::RightAhead) {
            this->distance = sqrtf(powf(CAR_REAL_Y_REAR-midPoints[targetIndex].second, 2) + powf(CAR_REAL_X-(midPoints[targetIndex].first-offset), 2));
            this->angle = atanf((CAR_REAL_X-(midPoints[targetIndex].first-offset)) / (CAR_REAL_Y_FRONT-midPoints[targetIndex].second));
        }
    }
}

void ComputerVision::pathPlanning1() {
    int f = 0;
    size_t i = 1;
    for (; i<midPoints.size(); i++) {
        if(ptIntersectLimit(CAR_REAL_X, CAR_REAL_Y_MIDDLE, midPoints[i].first, midPoints[i].second, 0.125, i) ) {
            //pt found
            f = 1;
            break;
        }
    }
    if (f == 1) {
        targetIndex = i;
        angle = atan2(CAR_REAL_X - midPoints[i].first,CAR_REAL_Y_MIDDLE - midPoints[i].second);
        distance = sqrt((midPoints[i].second - CAR_REAL_Y_MIDDLE)*(midPoints[i].second - CAR_REAL_Y_MIDDLE) + (midPoints[i].first - CAR_REAL_X)*(midPoints[i].first - CAR_REAL_X));
    } else {
        targetIndex = i-2;
        angle = atan2(CAR_REAL_X - midPoints[i-2].first, CAR_REAL_Y_MIDDLE - midPoints[i-2].second);
        distance = sqrt((midPoints[i-2].second - CAR_REAL_Y_MIDDLE)*(midPoints[i-2].second - CAR_REAL_Y_MIDDLE) + (midPoints[i-2].first - CAR_REAL_X)*(midPoints[i-2].first - CAR_REAL_X));
    }
}

bool ComputerVision::isCrossRoad(bool isRightEdge) {
    #ifdef vision_debug_output
    if (isRightEdge) {
        std::cout<<"passing right cross road"<<std::endl;
    } else {
        std::cout<<"passing left cross road"<<std::endl;
    }
    #endif
    int prevX, prevY, newX, newY;
    float crossRoadPtBuffAngle, prevAngle, shiftAngle, newAngle;
    if (isRightEdge) {
        prevX = rightEdge.back().x;
        prevY = rightEdge.back().y;
        prevAngle = rightEdge.back().gradient;
        // store original angle before passing cross road
        crossRoadPtBuffAngle = rightEdge[rightEdge.size()-5].gradient;
    } else {
        prevX = leftEdge.back().x;
        prevY = leftEdge.back().y;
        prevAngle = leftEdge.back().gradient;
        // store original angle before passing cross road
        crossRoadPtBuffAngle = leftEdge[leftEdge.size()-5].gradient;
    }

    // Status: --- before passing cross road ---
    int count = 0;
    bool outOfBound = false;
    // Don't do unnecessary search on cross road (stop once you are to determine the searching angle / slope)
    while (count < 4 && !outOfBound) {
        for (int shiftCount=0; shiftCount<25; shiftCount++) {  // Try to adjust the angle for finding edge
            // Calculate xy-coord and try to find the gradient
            shiftAngle = prevAngle + powf(-1, shiftCount%2)*(PI/24)*((shiftCount+1)/2);
            newX = prevX - 3*sinf(shiftAngle);
            newY = prevY - 3*cosf(shiftAngle);
            if (newX<=1 || newX>=RAW_WIDTH-2 || newY<=1 || newY>=RAW_HEIGHT-2) {
                outOfBound = true;
                break;
            }
            if (isRightEdge) {
                newAngle = edgeGradient(newX, newY);
                if (!std::isnan(newAngle)) newAngle += PI;
            } else {
                newAngle = edgeGradient(newX, newY);
            }
            #ifdef vision_debug_output_more
            std::cout<<"-- check x:"<<newX<<" y:"<<newY<<" pa:"<<shiftAngle<<" na:"<<newAngle<<std::endl;
            #endif
            // Check whether the gradient is match
            if (!std::isnan(newAngle) && isSameDirection(shiftAngle, newAngle)) {
                // return if it is wrong direction (going backward)
                if (isSimilarAngles(shiftAngle, PI)) return false;
                // store the edge to the corresponding array
                if (isRightEdge) {
                    rightEdge.emplace_back((Point) {newX, newY, newAngle, true});
                } else {
                    leftEdge.emplace_back((Point) {newX, newY, newAngle, true});
                }
                prevX = newX;
                prevY = newY;
                prevAngle = shiftAngle;

                #ifdef vision_debug_output_more
                std::cout<<"-- hi x:"<<newX<<" y:"<<newY<<" pa:"<<shiftAngle<<" na:"<<newAngle<<std::endl;
                #endif

                break;
            }
        }
        if (std::isnan(newAngle)) {  // Can't find any edge, stop
            break;
        }
        count++;
    }

    if (count < 3) return false;  // Too short, hard to find reference
    if (prevY < 25) return false;  // Too far away, no need to worry about

    // Status: --- passing cross road (search other side) ---
    // determine the searching angle / slope
    float edgeSlope, c;
    bool isStraight;
    if (isRightEdge) {
        isStraight = rightEdge[rightEdge.size()-1].y == rightEdge[rightEdge.size()-3].y;
        if (isStraight) {  // check whether it is a vertical line
            c = rightEdge[rightEdge.size()-3].x;
        } else {
            edgeSlope = -((float)rightEdge[rightEdge.size()-1].x-(float)rightEdge[rightEdge.size()-3].x) / 
                          ((float)rightEdge[rightEdge.size()-1].y-(float)rightEdge[rightEdge.size()-3].y);
            c = (float)rightEdge[rightEdge.size()-3].y - edgeSlope*(float)rightEdge[rightEdge.size()-3].x;
        }
    } else {
        isStraight = leftEdge[leftEdge.size()-1].y == leftEdge[leftEdge.size()-3].y;
        if (isStraight) {  // check whether it is a vertical line
            c = leftEdge[leftEdge.size()-3].x;
        } else {
            edgeSlope = -((float)leftEdge[leftEdge.size()-1].x-(float)leftEdge[leftEdge.size()-3].x) / 
                          ((float)leftEdge[leftEdge.size()-1].y-(float)leftEdge[leftEdge.size()-3].y);
            c = (float)leftEdge[leftEdge.size()-3].y - edgeSlope*(float)leftEdge[leftEdge.size()-3].x;
        }
    }
    // search other side
    #ifdef vision_debug_output
    std::cout<<"search edge in cross road m:"<<edgeSlope<<" c:"<<c<<std::endl;
    #endif
    bool foundOtherSide = false;
    if (isStraight) {  // special case: vertical line search
        for (int y=prevY-3; y>5&&y<RAW_HEIGHT-2; y--) {
            int x = c;
            #ifdef vision_debug_output_more
            std::cout<<"-- check x:"<<x<<" y:"<<y<<std::endl;
            #endif
            if (!std::isnan(edgeGradient(x, y))) {
                prevX = x;
                prevY = y;
                if (isRightEdge) {
                    prevAngle = edgeGradient(x, y) + PI;
                } else {
                    prevAngle = edgeGradient(x, y);
                }

                foundOtherSide = true;

                #ifdef vision_debug_output
                std::cout<<"got the corresponding edge successfully x:"<<x<<" y:"<<y<<std::endl;
                #endif

                break;
            }
        }
    } else {
        for (int y=prevY-3; y>5&&y<RAW_HEIGHT-2; y--) {
            int x = ((float)y-c)/edgeSlope;
            if (x<=1 || x>=RAW_WIDTH-2) break;
            #ifdef vision_debug_output_more
            std::cout<<"-- check x:"<<x<<" y:"<<y<<std::endl;
            #endif
            if (!std::isnan(edgeGradient(x, y))) {
                prevX = x;
                prevY = y;
                if (isRightEdge) {
                    prevAngle = edgeGradient(x, y) + PI;
                } else {
                    prevAngle = edgeGradient(x, y);
                }
                
                foundOtherSide = true;

                #ifdef vision_debug_output
                std::cout<<"got the corresponding edge successfully x:"<<x<<" y:"<<y<<std::endl;
                #endif

                break;
            }
        }
    }

    if (!foundOtherSide) {
        #ifdef vision_debug_output
        std::cout<<"can't find other side"<<std::endl;
        #endif
        return false;
    }

    // Status: --- after passing cross road ---
    count = 0;
    while (count < 256) {
        for (int shiftCount=0; shiftCount<25; shiftCount++) {  // Try to adjust the angle for finding edge
            // Calculate xy-coord and try to find the gradient
            shiftAngle = prevAngle + powf(-1, shiftCount%2)*(PI/24)*((shiftCount+1)/2);
            newX = prevX - 3*sinf(shiftAngle);
            newY = prevY - 3*cosf(shiftAngle);
            if (newX<=1 || newX>=RAW_WIDTH-2 || newY<=1 || newY>=RAW_HEIGHT-2) return false;
            if (isRightEdge) {
                newAngle = edgeGradient(newX, newY);
                if (!std::isnan(newAngle)) newAngle += PI;
            } else {
                newAngle = edgeGradient(newX, newY);
            }
            #ifdef vision_debug_output_more
            std::cout<<"-- check x:"<<newX<<" y:"<<newY<<" pa:"<<shiftAngle<<" na:"<<newAngle<<std::endl;
            #endif
            // Check whether the gradient is match
            if (!std::isnan(newAngle) && isSameDirection(shiftAngle, newAngle)) {
                // return if it is wrong direction (going backward)
                if (isSimilarAngles(shiftAngle, PI)) return false;
                // check is it match to original angle before passing cross road
                if (isSimilarAngles(crossRoadPtBuffAngle, newAngle)) {
                    // back to normal edge tracking (without cross road tag)
                    #ifdef vision_debug_output
                    std::cout<<"keep searching, you passed cross road :)"<<newY<<std::endl;
                    #endif
                    trackStatus = TrackStatus::CrossRoad;
                    if (isRightEdge) {
                        rightEdge.emplace_back((Point) {newX, newY, newAngle, true});
                    } else {
                        leftEdge.emplace_back((Point) {newX, newY, newAngle, true});
                    }
                    return true;
                }
                // store the edge to the corresponding array
                if (isRightEdge) {
                    rightEdge.emplace_back((Point) {newX, newY, newAngle, true});
                } else {
                    leftEdge.emplace_back((Point) {newX, newY, newAngle, true});
                }
                prevX = newX;
                prevY = newY;
                prevAngle = shiftAngle;

                #ifdef vision_debug_output_more
                std::cout<<"-- hi x:"<<newX<<" y:"<<newY<<" pa:"<<shiftAngle<<" na:"<<newAngle<<std::endl;
                #endif
                break;
            }
        }
        if (std::isnan(newAngle)) {  // Can't find any edge, stop
            #ifdef vision_debug_output
            std::cout<<"stop tracking x:"<<newX<<" y:"<<newY<<std::endl;
            #endif
            break;
        }
        count++;
        if (count >= 30) break;  // Can't find the end of cross road in a long time
    }

    return false;
}

int ComputerVision::topPoint() {
    // Finding top point
    int halfWidth = RAW_WIDTH / 2;

    if (*(cameraPtr + RAW_WIDTH * (RAW_HEIGHT-3) + halfWidth-1) < 80 &&
        *(cameraPtr + RAW_WIDTH * (RAW_HEIGHT-3) + halfWidth) < 80) return 99999;  // Not found

    for (int y=RAW_HEIGHT-3; y>0; y--) {
        if (isEdge(halfWidth, y)) {
            return y;
        }
    }

    return 1;
}

int ComputerVision::topRightPoint() {
    // Finding top point
    int halfWidth = RAW_WIDTH - (RAW_WIDTH/4);

    if (*(cameraPtr + RAW_WIDTH * (RAW_HEIGHT-3) + halfWidth-1) < 80 &&
        *(cameraPtr + RAW_WIDTH * (RAW_HEIGHT-3) + halfWidth) < 80) return 99999;  // Not found

    for (int y=RAW_HEIGHT-3; y>0; y--) {
        if (isEdge(halfWidth, y)) {
            return y;
        }
    }

    return 1;
}

int ComputerVision::topLeftPoint() {
    // Finding top point
    int halfWidth = (RAW_WIDTH/4);

    if (*(cameraPtr + RAW_WIDTH * (RAW_HEIGHT-3) + halfWidth-1) < 80 &&
        *(cameraPtr + RAW_WIDTH * (RAW_HEIGHT-3) + halfWidth) < 80) return 99999;  // Not found

    for (int y=RAW_HEIGHT-3; y>0; y--) {
        if (isEdge(halfWidth, y)) {
            return y;
        }
    }

    return 1;
}

bool ComputerVision::ptIntersectLimit(float Ax, float Ay,float Bx, float By, float threshold, uint16_t searchLimit) {
    float AB, AP, PB;
    float Px, Py;
    AB = sqrt((Bx-Ax)*(Bx-Ax) + (By-Ay)*(By-Ay));
    //dynamic width
    threshold = threshold / AB * 120;

    for (int i=1; i<searchLimit-1; i++) {
        Px = leftLimit[i].first;
        Py = leftLimit[i].second;
        AP = sqrt((Px-Ax)*(Px-Ax) + (Py-Ay)*(Py-Ay));
        PB = sqrt((Bx-Px)*(Bx-Px) + (By-Py)*(By-Py));
        if (abs(AB - (AP + PB)) < threshold) {
            return true;
        }
    }
    for (int i=1; i<searchLimit-1; i++) {
        Px = rightLimit[i].first;
        Py = rightLimit[i].second;
        AP = sqrt((Px-Ax)*(Px-Ax) + (Py-Ay)*(Py-Ay));
        PB = sqrt((Bx-Px)*(Bx-Px) + (By-Py)*(By-Py));
        if (abs(AB - (AP + PB)) < threshold) {
            return true;
        }
    }
    return false;
}

bool ComputerVision::findBestRefEdge() {
    int leftCount = 0;
    for (size_t i=0; i<leftEdge.size(); i++) {
        if (!leftEdge[i].isExtraPoint) leftCount++;
    }
    
    int rightCount = 0;
    for (size_t i=0; i<rightEdge.size(); i++) {
        if (!rightEdge[i].isExtraPoint) rightCount++;
    }

    return rightCount >= leftCount;
}

bool ComputerVision::findBestRefEdge1() {
    int leftMaxHeight = 120;
    for (size_t i=0; i<leftEdge.size(); i++) {
        if (!leftEdge[i].isExtraPoint && leftMaxHeight > leftEdge[i].y) leftMaxHeight = leftEdge[i].y;
    }
    
    int rightMaxHeight = 120;
    for (size_t i=0; i<rightEdge.size(); i++) {
        if (!rightEdge[i].isExtraPoint && rightMaxHeight > rightEdge[i].y) rightMaxHeight = rightEdge[i].y;
    }

    return rightMaxHeight <= leftMaxHeight;
}

bool ComputerVision::findBestRefEdge2() {
    float leftDist = 0.0;
    if (leftEdge.size() >= 2) {
        int index = leftEdge.size() - 1;
        for (; index>0; index--) if (!leftEdge[index].isExtraPoint) break;
        leftDist = sqrtf(powf(leftEdge[0].x-leftEdge[index].x, 2) + powf(leftEdge[0].y-leftEdge[index].y, 2));
    }

    float rightDist = 0.0;
    if (rightEdge.size() >= 2) {
        int index = rightEdge.size() - 1;
        for (; index>0; index--) if (!rightEdge[index].isExtraPoint) break;
        rightDist = sqrtf(powf(rightEdge[0].x-rightEdge[index].x, 2) + powf(rightEdge[0].y-rightEdge[index].y, 2));
    }

    return rightDist >= leftDist;
}

int ComputerVision::failsave() {
    // Find right edge
    int right_1 = 0;
    for (int x=RAW_WIDTH-3; x>1; x--) {
        if (isEdge(x, RAW_HEIGHT-3)) {
            right_1 = x;
            break;
        }
    }
    int right_2 = 0;
    for (int x=RAW_WIDTH-3; x>1; x--) {
        if (isEdge(x, RAW_HEIGHT-7)) {
            right_2 = x;
            break;
        }
    }
    
    // Find left edge
    int left_1 = 0;
    for (int x=2; x<RAW_WIDTH-2; x++) {
        if (isEdge(x, RAW_HEIGHT-3)) {
            left_1 = x;
            break;
        }
    }
    int left_2 = 0;
    for (int x=2; x<RAW_WIDTH-2; x++) {
        if (isEdge(x, RAW_HEIGHT-7)) {
            left_2 = x;
            break;
        }
    }

    #ifdef vision_debug_output
    std::cout<<"failsave R1:"<<right_1<<" R2:"<<right_2<<" L1:"<<left_1<<" L2:"<<left_2<<std::endl;
    std::cout<<"failsave angle R1:"<<edgeGradient(right_1, RAW_HEIGHT-3)<<" R2:"<<edgeGradient(right_2, RAW_HEIGHT-7)<<" L1:"<<edgeGradient(left_1, RAW_HEIGHT-3)<<" L2:"<<edgeGradient(left_2, RAW_HEIGHT-7)<<std::endl;
    #endif

    int right_diff = right_2 - right_1;
    int left_diff = left_1 - left_2;
    if (right_diff > 20) right_diff = 0;
    if (left_diff > 20) left_diff = 0;
    if (right_diff > 0 && left_diff > 0) {
        if (right_diff == left_diff) {
            return 0;  // Straight
        } else if (right_diff > left_diff) {
            return 1;  // Right
        } else {
            return 2;  // Left
        }
    } else if (right_diff > 0) {
        return 1;  // Right
    } else if (left_diff > 0) {
        return 2;  // Left
    } else {
        return 0;  // Straight
    }
}

void ComputerVision::classifyTrackStatus() {
    if (distance == 0.0) {
        trackStatus = TrackStatus::NotSure;
        return;
    }

    bool hasLeftEdge = false;
    bool hasRightEdge = false;
    for (size_t i=0; i<leftLimit.size(); i++) {
        if (leftLimit[i].second < CAR_REAL_Y-PATH_PLAN_CUTOFF) {
            hasLeftEdge = true;
            break;
        }
    }
    for (size_t i=0; i<rightLimit.size(); i++) {
        if (rightLimit[i].second < CAR_REAL_Y-PATH_PLAN_CUTOFF) {
            hasRightEdge = true;
            break;
        }
    }

    if (trackStatus == TrackStatus::NotSure) {
        if (!hasLeftEdge && hasRightEdge) {
            trackStatus = TrackStatus::LeftNow;
        } else if (hasLeftEdge && !hasRightEdge) {
            trackStatus = TrackStatus::RightNow;
        } else {
            if (distance >= 100.0) {
                if (angle > 0.174532925) {
                    trackStatus = TrackStatus::LeftAhead;
                } else if (angle < -0.174532925) {
                    trackStatus = TrackStatus::RightAhead;
                } else {
                    trackStatus = TrackStatus::Straight;
                }
            } else {
                if (angle > 0) {
                    trackStatus = TrackStatus::LeftAhead;
                } else {
                    trackStatus = TrackStatus::RightAhead;
                }
            }
        }
    }
}

void ComputerVision::edgeVisualizer(uint8_t *processedDataPtr, bool drawLeft, bool drawRight, bool drawExtraPoint) {
    for (int x=0; x<RAW_WIDTH; x++) {  // Reset to 0
        for (int y=0; y<RAW_HEIGHT; y++) {
            *(processedDataPtr + RAW_WIDTH * y + x) = 0;
        }
    }

    // Draw left edge points
    if (drawLeft) {
        for (size_t i=0; i<leftEdge.size(); i++) {
            if (leftEdge[i].isExtraPoint) {
                if (drawExtraPoint) {
                    *(processedDataPtr + RAW_WIDTH * leftEdge[i].y + leftEdge[i].x) = 100;
                }
            } else {
                *(processedDataPtr + RAW_WIDTH * leftEdge[i].y + leftEdge[i].x) = 255;
            }
        }
    }
    
    // Draw right edge points
    if (drawRight) {
        for (size_t i=0; i<rightEdge.size(); i++) {
            if (rightEdge[i].isExtraPoint) {
                if (drawExtraPoint) {
                    *(processedDataPtr + RAW_WIDTH * rightEdge[i].y + rightEdge[i].x) = 100;
                }
            } else {
                *(processedDataPtr + RAW_WIDTH * rightEdge[i].y + rightEdge[i].x) = 255;
            }
        }
    }
}

void ComputerVision::edgeVisualizerPT(uint8_t *processedDataPtr, bool drawLeft, bool drawRight) {
    for (int x=0; x<LCD_WIDTH; x++) {  // Reset to 0
        for (int y=0; y<LCD_HEIGHT; y++) {
            *(processedDataPtr + LCD_WIDTH * y + x) = 0;
        }
    }

    // Draw middle points
    for (size_t i=0; i<midPoints.size(); i++) {
        int x = midPoints[i].first*REMAP_RATIO+REMAP_X_OFFSET;
        int y = midPoints[i].second*REMAP_RATIO+REMAP_Y_OFFSET;
        if (x<0 || x>=LCD_WIDTH || y<0 || y>=LCD_HEIGHT) continue;
        *(processedDataPtr + LCD_WIDTH * y + x) = 255;
    }

    // Draw left edge limit points
    if (drawLeft) {
        for (size_t i=0; i<leftLimit.size(); i++) {
            int x = leftLimit[i].first*REMAP_RATIO+REMAP_X_OFFSET;
            int y = leftLimit[i].second*REMAP_RATIO+REMAP_Y_OFFSET;
            if (x<0 || x>=LCD_WIDTH || y<0 || y>=LCD_HEIGHT) continue;
            *(processedDataPtr + LCD_WIDTH * y + x) = 255;
        }
    }
    
    // Draw right edge limit points
    if (drawRight) {
        for (size_t i=0; i<rightLimit.size(); i++) {
            int x = rightLimit[i].first*REMAP_RATIO+REMAP_X_OFFSET;
            int y = rightLimit[i].second*REMAP_RATIO+REMAP_Y_OFFSET;
            if (x<0 || x>=LCD_WIDTH || y<0 || y>=LCD_HEIGHT) continue;
            *(processedDataPtr + LCD_WIDTH * y + x) = 255;
        }
    }
}

void ComputerVision::pathVisualizerPT(uint8_t *processedDataPtr) {
    if (distance == 0.0) return;

    // Draw rear path
    float rearAngle = atan2f(CAR_REAL_X-midPoints[targetIndex].first, CAR_REAL_Y_REAR-midPoints[targetIndex].second);
    for (float y=CAR_REAL_Y_REAR-1; y>=midPoints[targetIndex].second; y--) {
        if (rearAngle == 0.0) {  // special case (vertical line)
            int newX = CAR_REAL_X*REMAP_RATIO+REMAP_X_OFFSET;
            int newY = y*REMAP_RATIO+REMAP_Y_OFFSET;
            if (newX<0 || newX>=LCD_WIDTH || newY<0 || newY>=LCD_HEIGHT) continue;
            *(processedDataPtr + LCD_WIDTH * newY + newX) = 100;
        } else {
            float x = CAR_REAL_X-(CAR_REAL_Y_REAR - y)*tanf(rearAngle);
            int newX = x*REMAP_RATIO+REMAP_X_OFFSET;
            int newY = y*REMAP_RATIO+REMAP_Y_OFFSET;
            if (newX<0 || newX>=LCD_WIDTH || newY<0 || newY>=LCD_HEIGHT) continue;
            *(processedDataPtr + LCD_WIDTH * newY + newX) = 100;
        }
    }

    // Draw front path
    for (float y=CAR_REAL_Y_FRONT-1; y>=midPoints[targetIndex].second; y--) {
        if (angle == 0.0) {  // special case (vertical line)
            int newX = CAR_REAL_X*REMAP_RATIO+REMAP_X_OFFSET;
            int newY = y*REMAP_RATIO+REMAP_Y_OFFSET;
            if (newX<0 || newX>=LCD_WIDTH || newY<0 || newY>=LCD_HEIGHT) continue;
            *(processedDataPtr + LCD_WIDTH * newY + newX) = 200;
        } else {
            float x = CAR_REAL_X-(CAR_REAL_Y_FRONT - y)*tanf(angle);
            int newX = x*REMAP_RATIO+REMAP_X_OFFSET;
            int newY = y*REMAP_RATIO+REMAP_Y_OFFSET;
            if (newX<0 || newX>=LCD_WIDTH || newY<0 || newY>=LCD_HEIGHT) continue;
            *(processedDataPtr + LCD_WIDTH * newY + newX) = 200;
        }
    }

    // Draw cutoff line
    int lineY = (CAR_REAL_Y-PATH_PLAN_CUTOFF)*REMAP_RATIO+REMAP_Y_OFFSET;
    for (int lineX=0; lineX<LCD_WIDTH; lineX++) {
        *(processedDataPtr + LCD_WIDTH * lineY + lineX) = 200;
    }
}

void ComputerVision::edgeVisualizerPT_Test(uint8_t *processedDataPtr, int width, int height, bool drawLeft, bool drawRight) {
    int x_offset = 250;
    int y_offset = 200;
    int ratio = 1;

    for (int x=0; x<width; x++) {  // Reset to 0
        for (int y=0; y<height; y++) {
            *(processedDataPtr + width * y + x) = 0;
        }
    }

    // Draw middle points
    for (size_t i=0; i<midPoints.size(); i++) {
        int x = midPoints[i].first*ratio+x_offset;
        int y = midPoints[i].second*ratio+y_offset;
        if (x<0 || x>=width || y<0 || y>=height) continue;
        *(processedDataPtr + width * y + x) = 255;
    }

    // Draw left edge limit points
    if (drawLeft) {
        for (size_t i=0; i<leftLimit.size(); i++) {
            int x = leftLimit[i].first*ratio+x_offset;
            int y = leftLimit[i].second*ratio+y_offset;
            if (x<0 || x>=width || y<0 || y>=height) continue;
            *(processedDataPtr + width * y + x) = 255;
        }
    }
    
    // Draw right edge limit points
    if (drawRight) {
        for (size_t i=0; i<rightLimit.size(); i++) {
            int x = rightLimit[i].first*ratio+x_offset;
            int y = rightLimit[i].second*ratio+y_offset;
            if (x<0 || x>=width || y<0 || y>=height) continue;
            *(processedDataPtr + width * y + x) = 255;
        }
    }
}

void ComputerVision::pathVisualizerPT_Test(uint8_t *processedDataPtr,int width, int height) {
    if (distance == 0.0) return;

    int x_offset = 250;
    int y_offset = 200;
    int ratio = 1;

    // Draw rear path
    float rearAngle = atan2f(CAR_REAL_X-midPoints[targetIndex].first, CAR_REAL_Y_REAR-midPoints[targetIndex].second);
    for (float y=CAR_REAL_Y_REAR-1; y>=midPoints[targetIndex].second; y--) {
        if (rearAngle == 0.0) {  // special case (vertical line)
            int newX = CAR_REAL_X*ratio+x_offset;
            int newY = y*ratio+y_offset;
            if (newX<0 || newX>=width || newY<0 || newY>=height) continue;
            *(processedDataPtr + width * newY + newX) = 100;
        } else {
            float x = CAR_REAL_X-(CAR_REAL_Y_REAR - y)*tanf(rearAngle);
            int newX = x*ratio+x_offset;
            int newY = y*ratio+y_offset;
            if (newX<0 || newX>=width || newY<0 || newY>=height) continue;
            *(processedDataPtr + width * newY + newX) = 100;
        }
    }

    // Draw front path
    for (float y=CAR_REAL_Y_FRONT-1; y>=midPoints[targetIndex].second; y--) {
        if (angle == 0.0) {  // special case (vertical line)
            int newX = CAR_REAL_X*ratio+x_offset;
            int newY = y*ratio+y_offset;
            if (newX<0 || newX>=width || newY<0 || newY>=height) continue;
            *(processedDataPtr + width * newY + newX) = 200;
        } else {
            float x = CAR_REAL_X-(CAR_REAL_Y_FRONT - y)*tanf(angle);
            int newX = x*ratio+x_offset;
            int newY = y*ratio+y_offset;
            if (newX<0 || newX>=width || newY<0 || newY>=height) continue;
            *(processedDataPtr + width * newY + newX) = 200;
        }
    }

    // Draw cutoff line
    int lineY = (CAR_REAL_Y-PATH_PLAN_CUTOFF)*ratio+y_offset;
    for (int lineX=0; lineX<width; lineX++) {
        *(processedDataPtr + width * lineY + lineX) = 200;
    }

    // Draw switch height line
    lineY = PP01_SWITCH_HEIGHT*ratio+y_offset;
    for (int lineX=0; lineX<width; lineX++) {
        *(processedDataPtr + width * lineY + lineX) = 100;
    }

    // Draw switch height line
    lineY = midPoints[targetIndex].second*ratio+y_offset;
    for (int lineX=0; lineX<width; lineX++) {
        *(processedDataPtr + width * lineY + lineX) = 50;
    }

    // Draw switch height line
    lineY = CAR_REAL_Y_FRONT*ratio+y_offset;
    for (int lineX=0; lineX<width; lineX++) {
        if (lineX%2) continue;
        *(processedDataPtr + width * lineY + lineX) = 255;
    }
}

void ComputerVision::pathVisualizerPT_Test1(uint8_t *processedDataPtr,int width, int height) {
    if (distance == 0.0) return;

    int x_offset = 250;
    int y_offset = 250;
    int ratio = 1;

    // Draw path
    for (float y=CAR_REAL_Y_MIDDLE-1; y>=midPoints[targetIndex].second; y--) {
        if (angle == 0.0) {  // special case (vertical line)
            int newX = CAR_REAL_X*ratio+x_offset;
            int newY = y*ratio+y_offset;
            if (newX<0 || newX>=width || newY<0 || newY>=height) continue;
            *(processedDataPtr + width * newY + newX) = 200;
        } else {
            float x = CAR_REAL_X-(CAR_REAL_Y_MIDDLE - y)*tanf(angle);
            int newX = x*ratio+x_offset;
            int newY = y*ratio+y_offset;
            if (newX<0 || newX>=width || newY<0 || newY>=height) continue;
            *(processedDataPtr + width * newY + newX) = 200;
        }
    }

    // Draw cutoff line
    int lineY = (CAR_REAL_Y-PATH_PLAN_CUTOFF)*ratio+y_offset;
    for (int lineX=0; lineX<width; lineX++) {
        *(processedDataPtr + width * lineY + lineX) = 200;
    }

    // Draw switch height line
    lineY = PP01_SWITCH_HEIGHT*ratio+y_offset;
    for (int lineX=0; lineX<width; lineX++) {
        *(processedDataPtr + width * lineY + lineX) = 100;
    }
}

void ComputerVision::pathVisualizerPTAvoid_Test(uint8_t *processedDataPtr,int width, int height) {
    if (distance == 0.0) return;

    for (int x=0; x<width; x++) {  // Reset to 0
        for (int y=0; y<height; y++) {
            *(processedDataPtr + width * y + x) = 0;
        }
    }

    int x_offset = 250;
    int y_offset = 250;
    int ratio = 1;

    // Draw path
    for (float y=CAR_REAL_Y_MIDDLE-1; y>=midPoints[targetIndex].second; y--) {
        if (angle == 0.0) {  // special case (vertical line)
            int newX = CAR_REAL_X*ratio+x_offset;
            int newY = y*ratio+y_offset;
            if (newX<0 || newX>=width || newY<0 || newY>=height) continue;
            *(processedDataPtr + width * newY + newX) = 200;
        } else {
            float x = CAR_REAL_X-(CAR_REAL_Y_MIDDLE - y)*tanf(angle);
            int newX = x*ratio+x_offset;
            int newY = y*ratio+y_offset;
            if (newX<0 || newX>=width || newY<0 || newY>=height) continue;
            *(processedDataPtr + width * newY + newX) = 200;
        }
    }

    // Draw cutoff line
    int lineY = (CAR_REAL_Y-PATH_PLAN_CUTOFF)*ratio+y_offset;
    for (int lineX=0; lineX<width; lineX++) {
        *(processedDataPtr + width * lineY + lineX) = 200;
    }

    // Draw switch height line
    lineY = PP01_SWITCH_HEIGHT*ratio+y_offset;
    for (int lineX=0; lineX<width; lineX++) {
        *(processedDataPtr + width * lineY + lineX) = 100;
    }
}
