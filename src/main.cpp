#include "app/application.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char** argv) {
    tg::Application app;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--native-frame") == 0) {
            app.setNativeFrame();
        } else if (std::strcmp(argv[i], "--smoke-test") == 0) {
            const int frames = (i + 1 < argc) ? std::atoi(argv[i + 1]) : 3;
            app.setSmokeTest(frames > 0 ? frames : 3);
        } else if (std::strcmp(argv[i], "--screenshot") == 0 && i + 1 < argc) {
            // Late enough that a window opened by the harness has been
            // measured and drawn: ImGui hides an auto-sized window on its
            // first frame while it works out how big it is, so a capture any
            // earlier catches a menu that is open but not yet on screen.
            app.setScreenshot(argv[++i], 6);
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
        } else if (std::strcmp(argv[i], "--boolean-demo") == 0 && i + 1 < argc) {
            app.setBooleanDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--coplanar-demo") == 0 && i + 1 < argc) {
            app.setCoplanarDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--history-demo") == 0) {
            app.setHistoryDemo();
        } else if (std::strcmp(argv[i], "--hold-transform") == 0) {
            app.setHoldTransform();
        } else if (std::strcmp(argv[i], "--fillet-demo") == 0 && i + 2 < argc) {
            // The first number was a mesh bevel's segment count. An exact round
            // has none, so it is read past and the edge count is what counts.
            ++i;
            app.setFilletEdgesDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--round-all-demo") == 0) {
            app.setRoundAllDemo();
        } else if (std::strcmp(argv[i], "--file-prompt") == 0 && i + 1 < argc) {
            app.setFileDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--export-stl") == 0 && i + 1 < argc) {
            app.setHeadlessExport(argv[++i]);
        } else if (std::strcmp(argv[i], "--export-3mf") == 0 && i + 1 < argc) {
            app.setHeadlessExport3mf(argv[++i]);
        } else if (std::strcmp(argv[i], "--fillet-pair-demo") == 0 && i + 1 < argc) {
            app.setFilletDemo(static_cast<float>(std::atof(argv[++i])));
        } else if (std::strcmp(argv[i], "--ui-mouse") == 0 && i + 1 < argc) {
            float mx = 0, my = 0; int down = 0;
            std::sscanf(argv[++i], "%f,%f,%d", &mx, &my, &down);
            app.setUiMouse(mx, my, down != 0);
        } else if (std::strcmp(argv[i], "--preview-check") == 0 && i + 1 < argc) {
            app.setPreviewCheck(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--print-demo") == 0 && i + 1 < argc) {
            app.setPrintDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--face-stress") == 0) {
            app.setFaceStress();
        } else if (std::strcmp(argv[i], "--reduce-demo") == 0 && i + 1 < argc) {
            app.setReduceDemo(argv[++i]);
        } else if (std::strcmp(argv[i], "--mesh-bench") == 0 && i + 1 < argc) {
            app.setMeshBench(argv[++i]);
        } else if (std::strcmp(argv[i], "--dialog-demo") == 0 && i + 1 < argc) {
            app.setDialogDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--step-demo") == 0 && i + 1 < argc) {
            app.setStepDemo(argv[++i]);
        } else if (std::strcmp(argv[i], "--pattern-demo") == 0 && i + 1 < argc) {
            app.setPatternDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--face-demo") == 0 && i + 1 < argc) {
            app.setFaceDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--fillet-open") == 0) {
            app.setFilletOpen();
        } else if (std::strcmp(argv[i], "--profile-demo") == 0 && i + 1 < argc) {
            app.setProfileDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--frame-probe") == 0 && i + 1 < argc) {
            app.setFrameProbe(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--svg-demo") == 0 && i + 1 < argc) {
            // --svg-demo drawing.svg [step]
            const char* file = argv[++i];
            int step = 1;
            if (i + 1 < argc && argv[i + 1][0] != '-') step = std::atoi(argv[++i]);
            app.setSvgDemo(file, step);
        } else if (std::strcmp(argv[i], "--sketch-demo") == 0 && i + 1 < argc) {
            app.setSketchDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--revolve-demo") == 0 && i + 1 < argc) {
            app.setRevolveDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--snap-demo") == 0 && i + 1 < argc) {
            app.setSnapDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--shell-fillet-demo") == 0) {
            app.setShellFilletDemo(true);
        } else if (std::strcmp(argv[i], "--shell-extrude-demo") == 0 && i + 1 < argc) {
            app.setShellExtrudeDemo(static_cast<float>(std::atof(argv[++i])));
        } else if (std::strcmp(argv[i], "--split-demo") == 0 && i + 1 < argc) {
            app.setSplitDemo(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--inset-demo") == 0 && i + 1 < argc) {
            app.setInsetDemo(static_cast<float>(std::atof(argv[++i])));
        } else if (std::strcmp(argv[i], "--shell-demo") == 0 && i + 1 < argc) {
            app.setShellDemo(static_cast<float>(std::atof(argv[++i])));
        } else if (std::strcmp(argv[i], "--measure-demo") == 0) {
            app.setMeasureDemo();
        } else if (std::strcmp(argv[i], "--auto-extrude") == 0 && i + 1 < argc) {
            app.setAutoExtrude(static_cast<float>(std::atof(argv[++i])));
        } else if (std::strcmp(argv[i], "--help") == 0) {
            std::printf("tangent - 3D modelling for print design\n"
                        "  --smoke-test [frames]   render N frames and exit\n"
                        "  --screenshot <out.ppm>  capture the window to a PPM\n"
                        "  --frame-probe <n>       print where every n frames went\n"
                        "  --native-frame          the system's title bar instead of the app's\n"
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
