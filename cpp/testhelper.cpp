#include "testhelper.h"
#include "linearalgebra.h"
#include "optics.h"

#include <random>

TestContext::TestContext() :
    incDirNum(300000)
{
    // unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    unsigned int seed = 2345;
    std::default_random_engine generator(seed);
    std::normal_distribution<float> gaussDistribution;
    std::uniform_real_distribution<float> uniformDistribution(0, 2*Geometry::PI);

    this->g = Geometry::createHexCylindar(5.0f);

    float sunLon = -90.0f * Geometry::PI / 180.0f;
    float sunLat = 27.0f * Geometry::PI / 180.0f;
    float sunDir[3] = {-std::cos(sunLat) * std::cos(sunLon),
        -std::cos(sunLat) * std::sin(sunLon),
        -std::sin(sunLat)};

    rayDir = new float[incDirNum * 3];
    mainAxRot = new float[incDirNum * 3];
    for (int i = 0; i < incDirNum; i++)
    {
        float tmpAx[3];
        tmpAx[0] = gaussDistribution(generator);
        tmpAx[1] = gaussDistribution(generator);
        tmpAx[2] = gaussDistribution(generator);
        LinearAlgebra::normalize3(tmpAx);

        float lon = std::atan2(tmpAx[1], tmpAx[0]);
        float lat = std::asin(tmpAx[2] / LinearAlgebra::norm3(tmpAx));
        // float lon = uniformDistribution(generator);
        // float lat = 0;
        float roll = uniformDistribution(generator);
        // float roll = 0;
        
        mainAxRot[i*3+0] = lon;
        mainAxRot[i*3+1] = lat;
        mainAxRot[i*3+2] = roll;

        memcpy(rayDir+i*3, sunDir, 3*sizeof(float));
        LinearAlgebra::rotateZ(lon, lat, roll, 1, rayDir+i*3);
    }

    param.maxRecursion = 9;
    param.raysPerDirection = 20;
}

TestContext::~TestContext()
{
    delete[] rayDir;
    delete[] mainAxRot;
}

void TestContext::writeFinalDirections(const std::vector<Ray*> &rays)
{
    int totalDir = 0;
    for (auto r : rays)
    {
        totalDir += r->totalNum();
    }

    printf("rays.size(): %lu, incDirNum: %d, totalDir: %d\n", rays.size(), incDirNum, totalDir);

    float *finalDir = new float[totalDir * 4];
    int k = 0;
    for (int i = 0; i < rays.size(); i++)
    {
        auto r = rays[i];
        std::vector<RaySegment *> v;
        v.push_back(r->firstRaySeg);
        while (!v.empty())
        {
            RaySegment *p = v.back();
            v.pop_back();
            if (p->nextReflect)
            {
                v.push_back(p->nextReflect);
            }
            if (p->nextRefract)
            {
                v.push_back(p->nextRefract);
            }
            if (p->isValidEnd()
                && LinearAlgebra::dot3(p->dir.val(), r->firstRaySeg->dir.val()) < 1.0f - 1e-5 )
            {
                memcpy(finalDir + k*4, p->dir.val(), 3*sizeof(float));
                LinearAlgebra::rotateZBack(mainAxRot[i/param.raysPerDirection*3], 
                    mainAxRot[i/param.raysPerDirection*3+1], mainAxRot[i/param.raysPerDirection*3+2], 1, finalDir + k*4);
                finalDir[k*4+3] = p->w;
                k++;
            }
        }
    }

    FILE * pFile;
    pFile = fopen ("directions.bin", "wb");
    fwrite(&totalDir, sizeof(int), 1, pFile);
    fwrite(finalDir, sizeof(float), totalDir * 4, pFile);
    fclose(pFile);

    delete[] finalDir;
}


void TestContext::writeGeometryInfo()
{
    int vtxNum = g->vtxNum();
    int faceNum = g->faceNum();

    float *vtx_buffer = new float[vtxNum * 3];
    g->copyVertexData(vtx_buffer);

    int *face_buffer = new int[faceNum * 3];
    g->copyFaceIdxData(face_buffer);

    FILE * pFile;
    pFile = fopen ("geometry.bin", "wb");
    fwrite(&vtxNum, sizeof(int), 1, pFile);
    fwrite(vtx_buffer, sizeof(float), vtxNum * 3, pFile);
    fwrite(&faceNum, sizeof(int), 1, pFile);
    fwrite(face_buffer, sizeof(int), faceNum * 3, pFile);
    fclose(pFile);
}


void TestContext::writeRayPaths(const std::vector<Ray*> &rays)
{
    FILE * pFile;
    pFile = fopen ("rays.bin", "wb");

    int totalRays = 0;
    for (auto r : rays)
    {
        totalRays += r->totalNum();
    }
    fwrite(&totalRays, sizeof(int), 1, pFile);

    for (int i = 0; i < rays.size(); i++)
    {
        auto r = rays[i];
        std::vector<RaySegment *> v;
        v.push_back(r->firstRaySeg);
        while (!v.empty())
        {
            RaySegment *p = v.back();
            v.pop_back();
            if (p->nextReflect)
            {
                v.push_back(p->nextReflect);
            }
            if (p->nextRefract)
            {
                v.push_back(p->nextRefract);
            }
            if (p->isValidEnd()
                && LinearAlgebra::dot3(p->dir.val(), r->firstRaySeg->dir.val()) < 1.0f - 1e-5 )
            {
                std::vector<RaySegment *> tmpV;
                RaySegment *q = p;
                while (q->prev)
                {
                    tmpV.push_back(q);
                    q = q->prev;
                }
                tmpV.push_back(q);

                int tmpNum = tmpV.size();
                fwrite(&tmpNum, sizeof(int), 1, pFile);

                std::reverse(tmpV.begin(), tmpV.end());
                for (auto r : tmpV)
                {
                    float tmpRayData[7] = {r->pt.x(), r->pt.y(), r->pt.z(),
                        r->dir.x(), r->dir.y(), r->dir.z(),
                        r->w};
                    float faceId = static_cast<float>(r->faceId);

                    fwrite(tmpRayData, sizeof(float), 7, pFile);
                    fwrite(&faceId, sizeof(float), 1, pFile);

                }
            }
        }
    }
    fclose(pFile);
}


