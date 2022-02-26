#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include "GrayBMP.h"
#include "computer_vision.h"

// Version 1.0.4

void pathPlanningTest() {
    int inputCode = 4234;
    std::cout << "Type '9999' to generate all outputs / type specific image code (00XX): ";
    std::cin >> inputCode;
    

    // Image info
    int imgWidth = RAW_WIDTH;
    int imgHeight = RAW_HEIGHT;
    int newImgWidth = 500;
    int newImgHeight = 300;

    // Simulate real MT9V034 output
    uint8_t cameraData[imgHeight * imgWidth];
    uint8_t processedData[imgWidth * imgHeight];
    uint8_t processedDataPT[newImgWidth * newImgHeight];

    ComputerVision frame = ComputerVision();

    if (inputCode == 99999) {
        std::cout << "\nPlease wait...\n";
        // for (int i=1; i<=400; i++) {
        for (int i=4201; i<=4392; i++) {
            if (i % 50 == 0) {
                std::cout << i << "th of image is processed.\n";
            }

            // Input Image
            char inCode[18];
            sprintf(inCode, "Greyscale/%04d.bmp", i);
            GrayBMP bmp(inCode);
            // Simulate real MT9V034 output
            for (int x = 0; x < imgWidth; x++) {
                for (int y = 0; y < imgHeight; y++) {
                    cameraData[imgWidth * y + x] = bmp.getPixel(x, y);
                }
            }

            frame.receiveData(cameraData);
            frame.conversion();

            // frame.edgeVisualizer(processedData, true, true, true);
            // GrayBMP bmp1(imgWidth, imgHeight, 0);
            // for (int x = 0; x < imgWidth; x++) {
            //     for (int y = 0; y < imgHeight; y++) {
            //         bmp1.getPixel(x, y) = processedData[imgWidth * y + x];
            //     }
            // }


            frame.edgeVisualizerPT_Test(processedDataPT, newImgWidth, newImgHeight, true, true);
            frame.pathVisualizerPT_Test(processedDataPT, newImgWidth, newImgHeight);
            GrayBMP bmp1(newImgWidth, newImgHeight, 0);
            for (int x = 0; x < newImgWidth; x++) {
                for (int y = 0; y < newImgHeight; y++) {
                    bmp1.getPixel(x, y) = processedDataPT[newImgWidth * y + x];
                }
            }


            char outCode[15];
            sprintf(outCode, "Output/%04d.bmp", i);
            bmp1.save(outCode);
        }
    } else if (inputCode == 88888) {
        // for (int i=1; i<=400; i++) {
        string temp = "";
        // for (int i=4201; i<=4396; i++) {
        for (int i=6000; i<=6073; i++) {
            std::cout << "\nPress Enter to continue...\n";
            getline(cin, temp);
            if (i % 50 == 0) {
                std::cout << i << "th of image is processed.\n";
            }

            // Input Image
            char inCode[18];
            sprintf(inCode, "Greyscale/%04d.bmp", i);
            GrayBMP bmp(inCode);
            // Simulate real MT9V034 output
            for (int x = 0; x < imgWidth; x++) {
                for (int y = 0; y < imgHeight; y++) {
                    cameraData[imgWidth * y + x] = bmp.getPixel(x, y);
                }
            }

            frame.receiveData(cameraData);

            auto start = std::chrono::steady_clock::now();

            frame.conversionAvoidTest();

            std::chrono::duration<double, std::micro> diff = std::chrono::steady_clock::now()-start;
            std::cout << "Time taken " << diff.count() << " us\n";

            std::cout << "Failsave (0S, 1R, L2) " << frame.failsave() << "\n";

            std::cout << "Image ID " << i << "\n";

            // frame.edgeVisualizerPT(processedData, true, true, false);
            // frame.pathVisualizerPT(processedData);

            
            frame.edgeVisualizer(processedData, true, true, true);
            GrayBMP bmp2(imgWidth, imgHeight, 0);
            for (int x = 0; x < imgWidth; x++) {
                for (int y = 0; y < imgHeight; y++) {
                    bmp2.getPixel(x, y) = processedData[imgWidth * y + x];
                }
            }
            bmp2.save("out2.bmp");

            bmp.save("out3.bmp");


            frame.edgeVisualizerPT_Test(processedDataPT, newImgWidth, newImgHeight, true, true);
            frame.pathVisualizerPT_Test(processedDataPT, newImgWidth, newImgHeight);
            GrayBMP bmp1(newImgWidth, newImgHeight, 0);
            for (int x = 0; x < newImgWidth; x++) {
                for (int y = 0; y < newImgHeight; y++) {
                    bmp1.getPixel(x, y) = processedDataPT[newImgWidth * y + x];
                }
            }

            bmp1.save("out.bmp");
        }
    } else if (inputCode > 0) {  //  && inputCode <= 400
        int a = 0;

        // Input Image
        char inCode[18];
        sprintf(inCode, "Greyscale/%04d.bmp", inputCode);
        GrayBMP bmp(inCode);
        // Simulate real MT9V034 output
        for (int x = 0; x < imgWidth; x++) {
            for (int y = 0; y < imgHeight; y++) {
                cameraData[imgWidth * y + x] = bmp.getPixel(x, y);
            }
        }

        frame.receiveData(cameraData);

        auto start = std::chrono::steady_clock::now();

        frame.conversionAvoidTest();

        std::chrono::duration<double, std::micro> diff = std::chrono::steady_clock::now()-start;
        std::cout << "Time taken " << diff.count() << " us\n";

        std::cout << "Failsave (0S, 1R, L2) " << frame.failsave() << "\n";

        // frame.edgeVisualizerPT(processedData, true, true, false);
        // frame.pathVisualizerPT(processedData);

        
        frame.edgeVisualizer(processedData, true, true, true);
        GrayBMP bmp2(imgWidth, imgHeight, 0);
        for (int x = 0; x < imgWidth; x++) {
            for (int y = 0; y < imgHeight; y++) {
                bmp2.getPixel(x, y) = processedData[imgWidth * y + x];
            }
        }
        bmp2.save("out2.bmp");

        frame.edgeVisualizerPT_Test(processedDataPT, newImgWidth, newImgHeight, false, false);
        frame.pathVisualizerPT_Test(processedDataPT, newImgWidth, newImgHeight);
        GrayBMP bmp1(newImgWidth, newImgHeight, 0);
        for (int x = 0; x < newImgWidth; x++) {
            for (int y = 0; y < newImgHeight; y++) {
                bmp1.getPixel(x, y) = processedDataPT[newImgWidth * y + x];
            }
        }

        bmp1.save("out.bmp");

        GrayBMP bmp3(imgWidth, imgHeight, 0);
        for (int x = 0; x < imgWidth; x++) {
            for (int y = 0; y < imgHeight; y++) {
                bmp3.getPixel(x, y) = bmp.getPixel(x, y);
            }
        }
        bmp3.save("out3.bmp");
    }

    std::cout<<"Done"<<std::endl;
}

void drawFullImage() {
    // int x_offset = 250;
    // int y_offset = 250;
    // float ratio = 1.0;

    // int imgWidth = RAW_WIDTH;
    // int imgHeight = RAW_HEIGHT;
    // int newImgWidth = 500;
    // int newImgHeight = 300;

    int x_offset = 80;
    int y_offset = 75;
    float ratio = 0.5;

    int imgWidth = RAW_WIDTH;
    int imgHeight = RAW_HEIGHT;
    int newImgWidth = 160;
    int newImgHeight = 120;

    // Simulate real MT9V034 output
    uint8_t cameraData[imgHeight * imgWidth];

    int inputCode = 45;
    std::cout << "Type specific image code (00XX): ";
    std::cin >> inputCode;
    char inCode[18];
    sprintf(inCode, "Greyscale/%04d.bmp", inputCode);
    GrayBMP bmp(inCode);
    // Simulate real MT9V034 output
    for (int x = 0; x < imgWidth; x++) {
        for (int y = 0; y < imgHeight; y++) {
            cameraData[imgWidth * y + x] = bmp.getPixel(x, y);
        }
    }
    // Output Image
    GrayBMP bmp1(newImgWidth, newImgHeight, 0);
    for (int x = 0; x < imgWidth; x++) {
        for (int y = 0; y < imgHeight; y++) {
            int u = remapImg[y][x][0]*ratio+x_offset;
            int v = remapImg[y][x][1]*ratio+y_offset;
            if (u<0 || u>=newImgWidth || v<0 || v>=newImgHeight) continue;
            bmp1.getPixel(u, v) = cameraData[imgWidth * y + x];
        }
    }
    bmp1.save("out.bmp");

    std::cout<<"Bottom center 1 x: "<<remapImg[119][91][0]<<" y: "<<remapImg[119][91][1]<<std::endl;
    std::cout<<"Bottom center 2 x: "<<remapImg[119][92][0]<<" y: "<<remapImg[119][92][1]<<std::endl;
    std::cout<<"Bottom center 1 x: "<<remapImg[0][91][0]<<" y: "<<remapImg[0][91][1]<<std::endl;
    std::cout<<"Bottom center 2 x: "<<remapImg[0][92][0]<<" y: "<<remapImg[0][92][1]<<std::endl;

    std::cout<<"Done"<<std::endl;
}




int main() {
    
    // drawFullImage();

    pathPlanningTest();

    return 0;
}