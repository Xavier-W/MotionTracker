#include "motion_tracker.h"
#include <iostream>
#include <vector>

int main() {
    MotionTracker tracker;

    std::vector<Point2D> inputPoints = { {100, 100}, {200, 200}, {300, 300}, {400, 400} };

    // ³õÊ¼»¯¸ú×ÙÆ÷
    tracker.init(inputPoints[0]);

    for (const auto& point : inputPoints) {
        Point2D predicted = tracker.track(point);
        std::cout << "Measured: (" << point.x << ", " << point.y
            << ") Predicted: (" << predicted.x << ", " << predicted.y << ")\n";
    }

    return 0;
}
