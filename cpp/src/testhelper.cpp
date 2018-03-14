#include "testhelper.h"
#include "linearalgebra.h"

TestContext::TestContext() :
    oriGen(Geometry::PI*2, Geometry::PI*2),
    incDirNum(100000), wl(0)
{
    this->g = Geometry::createHexCylindar(5.0f);

    float sunLon = -90.0f * Geometry::PI / 180.0f;
    float sunLat = 27.0f * Geometry::PI / 180.0f;
    setSunPosition(sunLon, sunLat);

    rayDir = new float[incDirNum * 3];
    mainAxRot = new float[incDirNum * 3];

    param.maxRecursion = 9;
    param.raysPerDirection = 20;
}

TestContext::~TestContext()
{
    delete[] rayDir;
    delete[] mainAxRot;
}

const Geometry * TestContext::getGeometry() const
{
    return this->g;
}

const float * TestContext::getRayDirections() const
{
    return this->rayDir;
}

const float * TestContext::getMainAxisRotations() const
{
    return this->mainAxRot;
}

void TestContext::setWavelength(float wl)
{
    float n = IceRefractiveIndex::n(wl);
    g->setRefractiveIndex(n);
    this->wl = wl;
}

int TestContext::getIncDirNum() const
{
    return incDirNum;
}

void TestContext::setIncDirNum(int incDirNum)
{
    this->incDirNum = incDirNum;
    this->initialized = false;
}

void TestContext::setSunPosition(float lon, float lat)
{
    sunDir[0] = -std::cos(lat) * std::cos(lon);
    sunDir[1] = -std::cos(lat) * std::sin(lon);
    sunDir[2] = -std::sin(lat);
}

bool TestContext::isSettingsApplied()
{
    return initialized;
}

void TestContext::applySettings()
{
    if (rayDir) {
        delete[] rayDir;
        rayDir = new float[3 * incDirNum];
    }
    if (mainAxRot) {
        delete[] mainAxRot;
        mainAxRot = new float[3 * incDirNum];
    }

    memset(rayDir, 0.0f, incDirNum*3*sizeof(float));
    memset(mainAxRot, 0.0f, incDirNum*3*sizeof(float));
    oriGen.fillData(sunDir, incDirNum, rayDir, mainAxRot);

    this->initialized = true;
}

void TestContext::writeFinalDirections(const std::vector<Ray*> &rays)
{
    int totalDir = 0;
    for (auto r : rays) {
        totalDir += r->totalNum();
    }

    printf("rays.size(): %lu, incDirNum: %d, totalDir: %d\n", rays.size(), incDirNum, totalDir);

    auto *finalDir = new float[totalDir * 4];
    int k = 0;
    for (int i = 0; i < rays.size(); i++) {
        auto r = rays[i];
        std::vector<RaySegment *> v;
        v.push_back(r->firstRaySeg);
        while (!v.empty()) {
            RaySegment *p = v.back();
            v.pop_back();
            if (p->nextReflect) {
                v.push_back(p->nextReflect);
            }
            if (p->nextRefract) {
                v.push_back(p->nextRefract);
            }
            if (p->isValidEnd()
                && LinearAlgebra::dot3(p->dir.val(), r->firstRaySeg->dir.val()) < 1.0f - 1e-5 ) {
                memcpy(finalDir + k*4, p->dir.val(), 3*sizeof(float));
                LinearAlgebra::rotateZBack(mainAxRot[i/param.raysPerDirection*3], 
                    mainAxRot[i/param.raysPerDirection*3+1], mainAxRot[i/param.raysPerDirection*3+2], 1, finalDir + k*4);
                finalDir[k*4+3] = p->w;
                k++;
            }
        }
    }

    char filename[128];
    std::sprintf(filename, "directions_%.1f.bin", wl);
    FILE * pFile;
    pFile = fopen(filename, "wb");
    fwrite(&totalDir, sizeof(int), 1, pFile);
    fwrite(finalDir, sizeof(float), totalDir * 4, pFile);
    fclose(pFile);

    delete[] finalDir;
}


void TestContext::writeGeometryInfo()
{
    int vtxNum = g->vtxNum();
    int faceNum = g->faceNum();

    auto *vtx_buffer = new float[vtxNum * 3];
    g->copyVertexData(vtx_buffer);

    auto *face_buffer = new int[faceNum * 3];
    g->copyFaceIdxData(face_buffer);

    FILE * pFile;
    pFile = fopen("geometry.bin", "wb");
    fwrite(&vtxNum, sizeof(int), 1, pFile);
    fwrite(vtx_buffer, sizeof(float), vtxNum * 3, pFile);
    fwrite(&faceNum, sizeof(int), 1, pFile);
    fwrite(face_buffer, sizeof(int), faceNum * 3, pFile);
    fclose(pFile);
}


void TestContext::writeRayPaths(const std::vector<Ray*> &rays)
{
    FILE * pFile;
    pFile = fopen("rays.bin", "wb");

    int totalRays = 0;
    for (auto r : rays) {
        totalRays += r->totalNum();
    }
    fwrite(&totalRays, sizeof(int), 1, pFile);

    for (auto r : rays) {
        std::vector<RaySegment *> v;
        v.push_back(r->firstRaySeg);
        while (!v.empty()) {
            RaySegment *p = v.back();
            v.pop_back();
            if (p->nextReflect) {
                v.push_back(p->nextReflect);
            }
            if (p->nextRefract) {
                v.push_back(p->nextRefract);
            }
            if (p->isValidEnd()
                && LinearAlgebra::dot3(p->dir.val(), r->firstRaySeg->dir.val()) < 1.0f - 1e-5 ) {
                std::vector<RaySegment *> tmpV;
                RaySegment *q = p;
                while (q->prev) {
                    tmpV.push_back(q);
                    q = q->prev;
                }
                tmpV.push_back(q);

                int tmpNum = tmpV.size();
                fwrite(&tmpNum, sizeof(int), 1, pFile);

                std::reverse(tmpV.begin(), tmpV.end());
                for (auto tmp_r : tmpV) {
                    float tmpRayData[7] = {tmp_r->pt.x(), tmp_r->pt.y(), tmp_r->pt.z(),
                        tmp_r->dir.x(), tmp_r->dir.y(), tmp_r->dir.z(),
                        tmp_r->w};
                    auto faceId = static_cast<float>(tmp_r->faceId);

                    fwrite(tmpRayData, sizeof(float), 7, pFile);
                    fwrite(&faceId, sizeof(float), 1, pFile);

                }
            }
        }
    }
    fclose(pFile);
}


OrientationGenerator::OrientationGenerator(float axStd, float rollStd,
        AxisDistribution ax, RollDistribution roll) :
    axDist(ax), rollDist(roll),
    axStd(axStd), rollStd(rollStd)
{
    unsigned int seed = static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count());
    // unsigned int seed = 2345;
    generator.seed(seed);
}


void OrientationGenerator::fillData(const float *sunDir, int num, float *rayDir, float *mainAxRot)
{
    for (int i = 0; i < num; i++) {
        float lon, lat, roll;

        switch (axDist) {
            case AxisDistribution::AX_SPH_UNIFORM : {
                float v[3] = {gaussDistribution(generator),
                              gaussDistribution(generator),
                              gaussDistribution(generator)};
                LinearAlgebra::normalize3(v);
                lon = std::atan2(v[1], v[0]);
                lat = std::asin(v[2] / LinearAlgebra::norm3(v));
            }
                break;
            case AxisDistribution::AX_HOR_GAUSS :
                lon = uniformDistribution(generator) * 2 * Geometry::PI;
                lat = gaussDistribution(generator) * axStd;
                break;
            case AxisDistribution::AX_ZENITHAL_GAUSS :
                // TODO this implementation may be NOT right
                lon = uniformDistribution(generator) * 2 * Geometry::PI;
                lat = Geometry::PI / 2 - std::abs(gaussDistribution(generator) * axStd);
                break;
        }

        switch (rollDist) {
            case RollDistribution::ROLL_HOR :
                roll = 0;
                break;
            case RollDistribution::ROLL_UNIFORM :
                roll = uniformDistribution(generator) * 2 * Geometry::PI;
                break;
        }

        mainAxRot[i*3+0] = lon;
        mainAxRot[i*3+1] = lat;
        mainAxRot[i*3+2] = roll;

        memcpy(rayDir+i*3, sunDir, 3*sizeof(float));
        LinearAlgebra::rotateZ(lon, lat, roll, 1, rayDir+i*3);
    }
}


void OrientationGenerator::setAxisOrientation(AxisDistribution ax, float axStd)
{
    this->axDist = ax;
    this->axStd = axStd;
}


void OrientationGenerator::setAxisRoll(RollDistribution roll, float rollStd)
{
    this->rollDist = roll;
    this->rollStd = rollStd;
}


