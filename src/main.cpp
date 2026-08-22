#include "app/application.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char** argv) {
    tg::Application app;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--smoke-test") == 0) {
            const int frames = (i + 1 < argc) ? std::atoi(argv[i + 1]) : 3;
            app.setSmokeTest(frames > 0 ? frames : 3);
        } else if (std::strcmp(argv[i], "--screenshot") == 0 && i + 1 < argc) {
            app.setScreenshot(argv[++i], 4);
        } else if (std::strcmp(argv[i], "--camera") == 0 && i + 1 < argc) {
            float yaw = 0, pitch = 0, dist = 0;
            std::sscanf(argv[++i], "%f,%f,%f", &yaw, &pitch, &dist);
            app.setCamera(yaw, pitch, dist);
        } else if (std::strcmp(argv[i], "--empty") == 0) {
            app.setStartEmpty();
        } else if (std::strcmp(argv[i], "--no-grid") == 0) {
            app.setNoGrid();
        } else if (std::strcmp(argv[i], "--grid-probe") == 0 && i + 1 < argc) {
            float y0 = 0, y1 = 90; int steps = 181;
            std::sscanf(argv[++i], "%f,%f,%d", &y0, &y1, &steps);
            app.setGridProbe(y0, y1, steps);
        } else if (std::strcmp(argv[i], "--grid-align") == 0 && i + 1 < argc) {
            float y0 = 0, y1 = 90; int steps = 91;
            std::sscanf(argv[++i], "%f,%f,%d", &y0, &y1, &steps);
            app.setGridAlign(y0, y1, steps);
        } else if (std::strcmp(argv[i], "--select-face") == 0 && i + 1 < argc) {
            app.setPickFace(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--measure-demo") == 0) {
            app.setMeasureDemo();
        } else if (std::strcmp(argv[i], "--auto-extrude") == 0 && i + 1 < argc) {
            app.setAutoExtrude(static_cast<float>(std::atof(argv[++i])));
        } else if (std::strcmp(argv[i], "--help") == 0) {
            std::printf("tangent - 3D modelling for print design\n"
                        "  --smoke-test [frames]   render N frames and exit\n"
                        "  --screenshot <out.ppm>  capture the window to a PPM\n"
                        "  --camera y,p,d          place the camera (degrees, mm)\n"
                        "  --empty                 start with an empty scene\n"
                        "  --grid-probe y0,y1,n    sweep yaw, printing viewport luminance\n");
            return 0;
        }
    }

    if (!app.init()) {
        app.shutdown();
        return 1;
    }
    const int rc = app.run();
    app.shutdown();
    return rc;
}
