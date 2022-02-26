#include "GrayBMP.h"

uint32_t getRowSize(uint16_t bitsPerPixel, uint32_t width) {
    return (bitsPerPixel * width + 31)/32 * 4;
}

uint32_t getRowSize(BMPHeader& header) {
    return getRowSize(header.bitsPerPixel, header.width);
}

std::vector<uint8_t> readBMP(const std::string &filename, BMPHeader& header, std::vector<uint8_t> &colorTable) {
    std::ifstream f(filename, std::ios::binary);

    if (f.fail()) {
        std::cout << "The file at the location does not exist." << std::endl;
        return{};
    }

    f.read((char *)&header, sizeof(header));

    if (header.magic != 19778) {
        std::cout << "The input file is not a BMP" << std::endl;
        return{};
    }

    if (header.bitsPerPixel > 8)
        return{};

    std::vector<uint8_t> img;
    uint8_t data[header.imageSize];

    colorTable = std::vector<uint8_t>(4 * header.numberOfColors);

    f.read((char *)&colorTable[0], 4 * header.numberOfColors);

    // find bits
    f.seekg(header.offset);

    // read data
    f.read((char *)&data[0], header.imageSize);

    // pixels are packed into 4-byte rows, filled bottom up, left to right, unused cells will be padded with some value

    int rowSize = getRowSize(header);
    int rowCount = header.imageSize / rowSize;

    for (int row = rowCount - 1; row >= 0 ; row--) {
        for (int x = 0; x < header.width; x++) {
            img.push_back(data[row*rowSize + x]);
        }
    }

    f.close();

    return img;
}

void writeBMP(std::string const &filename, BMPHeader &bmpHeader, std::vector<uint8_t> &img, std::vector<uint8_t> &colorTable) {
    std::ofstream f(filename, std::ios::binary);
    f.write((char *)&bmpHeader, sizeof(bmpHeader));

    f.seekp(bmpHeader.headerSize + 14);
    f.write((char *)&colorTable[0], colorTable.size() * sizeof(uint8_t));

    f.seekp(bmpHeader.offset);

    int rowSize = getRowSize(bmpHeader);
    int padSize = rowSize - bmpHeader.width;

    for (int y = bmpHeader.height - 1; y >= 0 ; y--) {
        for (int x = 0; x < bmpHeader.width; x++) {
            f.put(getPixel(img, x, y, bmpHeader.width));
        }
        for (int i = 0; i < padSize; i++) {
            f.put(0);
        }
    }

    f.close();
}

uint8_t getPixel(const std::vector <uint8_t> &img, int x, int y, int width) {
    return img[y*width+x];
}

/**
 * The 8-bit BMPs are indexed images. It means that there are 256 colors in its palette. The conventional 8-bit
 * gray scale images will populate its color table from black to white in a linear fashion.
 *
 * The color table consists of Red, Green, Blue, Alpha values, in the RGBA32 format. However, the A is set to 0 since
 * most OSs disallows this feature.
 *
 * @return color table section of data in bytes
 */
std::vector<uint8_t> makeColorTable() {
    std::vector<uint8_t> colorTable;

    for (int i = 0; i < 256; i++) {
        colorTable.push_back((uint8_t) i);
        colorTable.push_back((uint8_t) i);
        colorTable.push_back((uint8_t) i);
        colorTable.push_back(0);
    }

    return colorTable;
}


std::vector<uint8_t> GrayBMP::grayScaleColorTable;

GrayBMP::GrayBMP() {
    if (GrayBMP::grayScaleColorTable.empty()) {
        GrayBMP::grayScaleColorTable = makeColorTable();
    }
}

GrayBMP::GrayBMP(std::string const &filename): GrayBMP() {
    img = readBMP(filename, bmpHeader, colorTable);
}

void GrayBMP::save(std::string const &filename) {
    writeBMP(filename, bmpHeader, img, colorTable);
}

GrayBMP::GrayBMP(uint32_t width, uint32_t height, uint8_t backgroundColor): GrayBMP() {
    static uint32_t fileHeaderSize = 14;
    static uint32_t dibHeaderSize = 40;
    static uint32_t colorTableSize = 256 * 4;
    static uint32_t offset = fileHeaderSize + dibHeaderSize + colorTableSize;

    uint32_t rowSize = getRowSize(8, width);
    uint32_t imageSize = rowSize * height;
    uint32_t fileSize = imageSize + offset;

    colorTable = grayScaleColorTable;

    bmpHeader = {
            19778,
            fileSize,
            0,
            0,
            offset,
            dibHeaderSize,
            width,
            height,
            1,
            8,
            0,
            imageSize,
            2834,
            2834,
            256,
            256
    };

    for (int i = 0; i < width * height; i++) {
        img.push_back(backgroundColor);
    }
}

uint8_t &GrayBMP::getPixel(int x, int y) {
    return img[y*bmpHeader.width+x];
}
