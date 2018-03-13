#ifndef OPTICS_H
#define OPTICS_H

#include "geometry.h"

#include <random>


class RayTracingParam
{
public:

    int raysPerDirection;
    int maxRecursion;
    
};


class RaySegment
{
public:
    RaySegment(Vec3f &pt, Vec3f &dir, float w, int faceId = -1);
    ~RaySegment() = default;

    bool isValidEnd();

    RaySegment * nextReflect;
    RaySegment * nextRefract;
    RaySegment * prev;

    Vec3f pt;
    Vec3f dir;
    float w;
    int faceId;

    bool isFinished;
};


class Ray
{
public:
    Ray(Vec3f &pt, Vec3f &d, float w, int faceId = -1);
    ~Ray();

    size_t totalNum();

    RaySegment *firstRaySeg;
    
};


class Optics
{
public:
    static void initRays(int num, const float *dir, int face_num, const float *faces,
        float *ray_pt, int *face_id);
    static void hitSurface(float n, int num, const float *dir, const float *norm,
        float *reflect_dir, float *refract_dir, float *reflect_w);
    static void propagate(int num, const float *pt, const float *dir, int face_num, const float *faces,
        float *new_pt, int *new_face_id);
    static void traceRays(int dir_num, const float *dir, const RayTracingParam& param, const Geometry *g,
        std::vector<Ray*>& rays);

    static float getReflectRatio(float inc_angle, float n1, float n2);
    static void intersectLineFace(const float *pt, const float *dir, const float *face,
        float *p, float *t, float *alpha, float *beta);

private:
    static std::default_random_engine generator;
    static std::uniform_real_distribution<float> distribution;
    
};


#endif // OPTICS_H