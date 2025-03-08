#include <vector>

// Simpel kamera defintion
struct CameraInfo {
    double fx, fy;  // Focal lengths
    double cx, cy;  // Principal point
    std::vector<double> distortion;  // Distortion coefficients

    const unsigned int width = 100; // Ändra till deras korrekta värden
    const unsigned int height = 100;
};