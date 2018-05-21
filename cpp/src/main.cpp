#include <vector>
#include <chrono>
#include <thread>

#include "geometry.h"
#include "optics.h"
#include "processhelper.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/error/en.h"
#include "rapidjson/pointer.h"


int main(int argc, char *argv[])
{
    using namespace rapidjson;

    if (argc != 2) {
        printf("USAGE: %s <config-file>\n", argv[0]);
        return -1;
    }

    auto start = std::chrono::system_clock::now();

    // Open configuration file
    FILE* fp = fopen(argv[1], "rb");
    if (!fp) {
        printf("ERROR: file %s cannot be open!\n", argv[1]);
        return -1;
    }

    // Parse configuration
    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    Document d;
    if (d.ParseStream(is).HasParseError()) {
        fprintf(stderr, "\nError(offset %u): %s\n", 
            (unsigned)d.GetErrorOffset(),
        GetParseError_En(d.GetParseError()));
        fclose(fp);
        return -1;
    }

    fclose(fp);

    SimulationContext context;

    // Parsing ray number
    int rayNumber = Pointer("/ray_number").Get(d)->GetInt();
    context.setTotalRayNum(rayNumber);

    // Parsing max recursion number
    int maxRecursionNum = Pointer("/max_recursion").Get(d)->GetInt();
    context.setMaxRecursionNum(maxRecursionNum);

    // Parsing sun altitude
    float sunAltitude = static_cast<float>(Pointer("/sun/altitude").Get(d)->GetDouble());
    context.envCtx->setSunPosition(90.0f, sunAltitude);

    // Parsing crystals
    int ci = 0;
    for (auto &c : Pointer("/crystal").Get(d)->GetArray()) {
        OrientationGenerator::Distribution axisDist, rollDist;
        float axisMean, rollMean;
        float axisStd, rollStd;

        if (*(Pointer("/axis/type").Get(c)) == "Gauss") {
            axisDist = OrientationGenerator::Distribution::GAUSS;
        } else if (*(Pointer("/axis/type").Get(c)) == "Uniform") {
            axisDist = OrientationGenerator::Distribution::UNIFORM;
        } else {
            fprintf(stderr, "\nError! <crystal[%d].axis.type> cannot recgonize! Exit.\n", ci);
            return -1;
        }

        if (*(Pointer("/roll/type").Get(c)) == "Gauss") {
            rollDist = OrientationGenerator::Distribution::GAUSS;
        } else if (*(Pointer("/roll/type").Get(c)) == "Uniform") {
            rollDist = OrientationGenerator::Distribution::UNIFORM;
        } else {
            fprintf(stderr, "\nError! <crystal[%d].roll.type> cannot recgonize! Exit.\n", ci);
            return -1;
        }

        axisMean = static_cast<float>(Pointer("/axis/mean").Get(c)->GetDouble());
        axisStd = static_cast<float>(Pointer("/axis/std").Get(c)->GetDouble());
        rollMean = static_cast<float>(Pointer("/roll/mean").Get(c)->GetDouble());
        rollStd = static_cast<float>(Pointer("/roll/std").Get(c)->GetDouble());

        float population = static_cast<float>(Pointer("/population").Get(c)->GetDouble());

        if (c["type"] == "HexCylindar") {
            /* HexCylindar */

            float h = static_cast<float>(Pointer("/parameter").Get(c)->GetDouble());
            context.crystalCtx->addGeometry(Geometry::createHexCylindar(h), population,
                axisDist, axisMean, axisStd,
                rollDist, rollMean, rollStd);

        } else if (c["type"] == "HexPyramid") {
            /* HexPyramid */

            const Value *p = Pointer("/parameter").Get(c);
            if (p->Size() == 3) {
                float h1 = static_cast<float>((*p)[0].GetDouble());
                float h2 = static_cast<float>((*p)[1].GetDouble());
                float h3 = static_cast<float>((*p)[2].GetDouble());
                context.crystalCtx->addGeometry(Geometry::createHexPyramid(h1, h2, h3), population,
                    axisDist, axisMean, axisStd,
                    rollDist, rollMean, rollStd);
            } else if (p->Size() == 5) {
                int i1 = p[0].GetInt();
                int i2 = p[1].GetInt();
                float h1 = static_cast<float>((*p)[2].GetDouble());
                float h2 = static_cast<float>((*p)[3].GetDouble());
                float h3 = static_cast<float>((*p)[4].GetDouble());
                context.crystalCtx->addGeometry(Geometry::createHexPyramid(i1, i2, h1, h2, h3), population,
                    axisDist, axisMean, axisStd,
                    rollDist, rollMean, rollStd);
            } else if (p->Size() == 7) {
                int upperIdx1 = p[0].GetInt();
                int upperIdx2 = p[1].GetInt();
                int lowerIdx1 = p[2].GetInt();
                int lowerIdx2 = p[3].GetInt();
                float h1 = static_cast<float>((*p)[2].GetDouble());
                float h2 = static_cast<float>((*p)[3].GetDouble());
                float h3 = static_cast<float>((*p)[4].GetDouble());
                context.crystalCtx->addGeometry(Geometry::createHexPyramid(upperIdx1, upperIdx2, lowerIdx1, 
                    lowerIdx2, h1, h2, h3), population,
                    axisDist, axisMean, axisStd,
                    rollDist, rollMean, rollStd);
            } else {
                fprintf(stderr, "\nERROR! <crystal[%d].parameter> number not match! Exit!\n");
                return -1;
            }
        } else if (c["type"] == "CubicPyramid") {
            /* CubicPyramid */

            const Value *p = Pointer("/parameter").Get(c);
            if (p->Size() == 2) {
                float h1 = static_cast<float>((*p)[0].GetDouble());
                float h2 = static_cast<float>((*p)[1].GetDouble());
                context.crystalCtx->addGeometry(Geometry::createCubicPyramid(h1, h2), population,
                    axisDist, axisMean, axisStd,
                    rollDist, rollMean, rollStd);
            } else {
                fprintf(stderr, "\nERROR! <crystal[%d].parameter> number not match! Exit!\n");
                return -1;
            }
        }

        ci++;
    }
    // context.crystalCtx->addGeometry(Geometry::createHexPyramidStackHalf(1, 1, 3, 2, 1.5f, 0.12f, 0.3f), 2.0f,
    //                                 OrientationGenerator::Distribution::GAUSS, 90.f, 15.0f,
    //                                 OrientationGenerator::Distribution::UNIFORM, 0.0f, 360.f);
    // // context.crystalCtx->addGeometry(Geometry::createHexPyramid(0.31f, 0.f, 1.30), 3.0f,
    // //                                 OrientationGenerator::Distribution::GAUSS, 0.0f, 3.4f,
    // //                                 OrientationGenerator::Distribution::UNIFORM, 0.0f, 360.f);
    // // context.crystalCtx->addGeometry(Geometry::createCubicPyramid(0.6f, 1.0f), 2.0f,
    // //                                 OrientationGenerator::Distribution::GAUSS, 90.0f-54.75f, 14.0f,
    // //                                 OrientationGenerator::Distribution::GAUSS, 0.0f, 2.6f);
    context.applySettings();

    auto t = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = t - start;
    printf("Initialization: %.2fms\n", diff.count() * 1.0e3);

    char filename[256];
    for (float wl = 440.0f; wl < 655.0f;) {
        printf("starting at wavelength: %.1f\n", wl);

        context.envCtx->setWavelength(wl);

        auto t0 = std::chrono::system_clock::now();
        Optics::traceRays(context);
        auto t1 = std::chrono::system_clock::now();
        diff = t1 - t0;
        printf("Ray tracing: %.2fms\n", diff.count() * 1.0e3);

        t0 = std::chrono::system_clock::now();
        std::sprintf(filename, "directions_%.1f_%lli.bin", wl, t0.time_since_epoch().count());
        context.writeFinalDirections(filename);

        // std::sprintf(filename, "paths_%.1f_%lli.bin", wl, t0.time_since_epoch().count());
        // context.writeRayInfo(filename, -104.5f*Geometry::PI/180.0f, -30.7f*Geometry::PI/180.0f, 0.5f/57.0f);
        t1 = std::chrono::system_clock::now();
        diff = t1 - t0;
        printf("Writing: %.2fms\n", diff.count() * 1.0e3);

        wl += 30.0f;
    }
    context.printCrystalInfo();

    auto end = std::chrono::system_clock::now();
    diff = end - start;
    printf("Total: %.3fs\n", diff.count());

    return 0;
}
