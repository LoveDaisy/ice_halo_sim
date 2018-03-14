#include "optics.h"
#include "linearalgebra.h"

RaySegment::RaySegment(Vec3f &pt, Vec3f &dir, float w, int faceId) :
    nextReflect(nullptr),
    nextRefract(nullptr),
    prev(nullptr),
    pt(pt), dir(dir), w(w), faceId(faceId), isFinished(false)
{ }

bool RaySegment::isValidEnd()
{
    return nextReflect == nullptr && nextRefract == nullptr 
            && w > 0 && faceId >= 0 && isFinished;
}


Ray::Ray(Vec3f &pt, Vec3f &d, float w, int faceId)
{
    firstRaySeg = new RaySegment(pt, d, w, faceId);
}

Ray::~Ray()
{
    std::vector<RaySegment *> v;
    v.push_back(firstRaySeg);
    while (!v.empty()) {
        RaySegment *p = v.back();
        if (p->nextReflect) {
            v.push_back(p->nextReflect);
        }
        if (p->nextRefract) {
            v.push_back(p->nextRefract);
        }
        if (p->nextReflect == nullptr && p->nextRefract == nullptr) {
            v.pop_back();
            if (p->prev) {
                if (p->prev->nextReflect == p) {
                    p->prev->nextReflect = nullptr;
                }
                if (p->prev->nextRefract == p) {
                    p->prev->nextRefract = nullptr;
                }
            }
            delete p;
        }
    }
}

size_t Ray::totalNum()
{
    if (firstRaySeg == nullptr) {
        return 0;
    }

    std::vector<RaySegment*> v;
    v.push_back(firstRaySeg);

    size_t n = 0;
    while (!v.empty()) {
        RaySegment *p = v.back();
        v.pop_back();
        if (p->nextReflect) {
            v.push_back(p->nextReflect);
        }
        if (p->nextRefract) {
            v.push_back(p->nextRefract);
        }
        if (p->isValidEnd()) {
            n++;
        }
    }
    return n;
}


std::default_random_engine Optics::generator(std::chrono::system_clock::now().time_since_epoch().count());
std::uniform_real_distribution<float> Optics::distribution(0.0f, 1.0f);


void Optics::initRays(int num, const float *dir, int face_num, const float *faces,
       float *ray_pt, int *face_id)
{

    auto *frac = new float[face_num];
    for (int i = 0; i < face_num; i++) {
        float v1[3], v2[3];
        LinearAlgebra::vec3FromTo(faces + i*9, faces + i*9 + 3, v1);
        LinearAlgebra::vec3FromTo(faces + i*9, faces + i*9 + 6, v2);
        float norm[3];
        LinearAlgebra::cross3(v1, v2, norm);
        float c = LinearAlgebra::dot3(dir, norm);
        frac[i] = c < 0 ? LinearAlgebra::norm3(norm) / 2 * (-c) : 0;
    }
    for (int i = 1; i < face_num; i++) {
        frac[i] += frac[i-1];
    }
    for (int i = 0; i < face_num; i++) {
        frac[i] /= frac[face_num-1];
    }
    frac[face_num-1] = 1.0f;    // Force to 1.0f

    for (int i = 0; i < num; i++) {
        int idx = face_num-1;
        float p = distribution(generator);
        float *tmp_pt = ray_pt + i*3;
        for (int j = 0; j < face_num; j++) {
            if (p <= frac[j]) {
                idx = j;
                break;
            }
        }

        face_id[i] = idx;
        float a = distribution(generator);
        float b = distribution(generator);
        if (a + b > 1.0f) {
            a = 1.0f - a;
            b = 1.0f - b;
        }
        tmp_pt[0] = faces[idx*9+0] + a * (faces[idx*9+3] - faces[idx*9+0]) + 
            b * (faces[idx*9+6] - faces[idx*9+0]);
        tmp_pt[1] = faces[idx*9+1] + a * (faces[idx*9+4] - faces[idx*9+1]) + 
            b * (faces[idx*9+7] - faces[idx*9+1]);
        tmp_pt[2] = faces[idx*9+2] + a * (faces[idx*9+5] - faces[idx*9+2]) + 
            b * (faces[idx*9+8] - faces[idx*9+2]);
    }

    delete[] frac;
}

void Optics::hitSurface(float n, int num, const float *dir, const float *norm,
        float *reflect_dir, float *refract_dir, float *reflect_w)
{
    for (int i = 0; i < num; ++i) {
        const float *tmp_dir = dir + i*3;
        const float *tmp_norm = norm + i*3;
        
        float cos_theta = LinearAlgebra::dot3(tmp_dir, tmp_norm);
        float inc_angle = std::acos(std::abs(cos_theta));
        float nf = cos_theta > 0 ? 1.0f : -1.0f;

        bool is_inner = cos_theta > 0;
        float tmp_n1 = is_inner ? n : 1;
        float tmp_n2 = is_inner ? 1 : n;

        reflect_w[i] = getReflectRatio(inc_angle, tmp_n1, tmp_n2);

        for (int j = 0; j < 3; ++j) {
            reflect_dir[i*3+j] = tmp_dir[j] - 2 * std::abs(cos_theta) * nf * tmp_norm[j];
        }

        float rr = tmp_n1 / tmp_n2;
        float d = 1.0f - rr * rr * (1.0f - cos_theta * cos_theta);
        for (int j = 0; j < 3; j++) {
            refract_dir[i*3+j] = d <= 0.0f ? reflect_dir[i*3+j] :
                rr * tmp_dir[j] - (rr * std::abs(cos_theta) - std::sqrt(d)) * (nf * tmp_norm[j]);
        }
    }
}

void Optics::propagate(int num, const float *pt, const float *dir, int face_num, const float *faces,
        float *new_pt, int *new_face_id)
{
    for (int i = 0; i < num; ++i) {
        const float *tmp_pt = pt + i*3;
        const float *tmp_dir = dir + i*3;

        float min_t = std::numeric_limits<float>::max();
        new_face_id[i] = -1;
        for (int j = 0; j < face_num; j++) {
            const float *tmp_face = faces + j*9;
            
            float p[3];
            float t, alpha, beta;

            intersectLineFace(tmp_pt, tmp_dir, tmp_face, &p[0], &t, &alpha, &beta);
            if (t > 1e-6 && t < min_t && alpha >= 0 && beta >= 0 && alpha + beta <= 1) {
                min_t = t;

                new_pt[i*3+0] = p[0];
                new_pt[i*3+1] = p[1];
                new_pt[i*3+2] = p[2];

                new_face_id[i] = j;
            }
        }
    }
}

void Optics::traceRays(int dir_num, const float *dir, const RayTracingParam& param, const Geometry *g,
        std::vector<Ray*>& rays)
{
    int max_ray_num = dir_num * param.maxRecursion * param.raysPerDirection * 3;
    auto *ray_pt_buffer = new float[max_ray_num * 3];
    auto *ray_pt_buffer2 = new float[max_ray_num * 3];
    auto *ray_dir_buffer = new float[max_ray_num * 3];
    auto *face_norm_buffer = new float[max_ray_num * 3];
    auto *face_id_buffer = new int[max_ray_num];

    auto *reflect_dir_buffer = new float[max_ray_num * 3];
    auto *refract_dir_buffer = new float[max_ray_num * 3];
    auto *reflect_w_buffer = new float[max_ray_num * 3];

    auto *face_data = new float[g->faceNum() * 9];
    g->copyFaceData(face_data);
    for (auto i = 0; i < dir_num; i++) {
        float *tmp_pt = ray_pt_buffer + i * param.raysPerDirection * 3;
        float *tmp_norm = face_norm_buffer + i * param.raysPerDirection * 3;
        float *tmp_dir = ray_dir_buffer + i * param.raysPerDirection * 3;
        int *tmp_id = face_id_buffer + i * param.raysPerDirection;

        initRays(param.raysPerDirection, dir + i*3, g->faceNum(), face_data,
            tmp_pt, tmp_id);
        for (auto j = 0; j < param.raysPerDirection; j++) {
            memcpy(tmp_dir + j*3, dir + i*3, 3*sizeof(float));
        }
        g->copyNormalData(param.raysPerDirection, tmp_id, tmp_norm);
    }

    int currentRayNumbers = dir_num * param.raysPerDirection;
    std::vector<RaySegment*> activeRaySegments;
    std::vector<RaySegment*> tmpRaySegs;
    for (auto i = 0; i < currentRayNumbers; i++) {
        Vec3f p(ray_pt_buffer + i * 3);
        Vec3f d(ray_dir_buffer + i * 3);
        Ray *r = new Ray(p, d, 1.0f, face_id_buffer[i]);
        rays.push_back(r);
        activeRaySegments.push_back(r->firstRaySeg);
    }

    // Start loop
    int recursion = 0;
    while (!activeRaySegments.empty() && recursion < param.maxRecursion) {
        hitSurface(g->refractiveIndex(), currentRayNumbers, ray_dir_buffer, face_norm_buffer,
            reflect_dir_buffer, refract_dir_buffer, reflect_w_buffer);
        for (auto i = 0; i < currentRayNumbers; i++) {
            Vec3f p(ray_pt_buffer + i * 3);
            Vec3f d(reflect_dir_buffer + i * 3);
            RaySegment *lastSeg = activeRaySegments[i];
            RaySegment *seg = new RaySegment(p, d, lastSeg->w * reflect_w_buffer[i], face_id_buffer[i]);
            lastSeg->nextReflect = seg;
            seg->prev = lastSeg;
            tmpRaySegs.push_back(seg);

            p.val(ray_pt_buffer + i * 3);
            d.val(refract_dir_buffer + i * 3);
            seg = new RaySegment(p, d, lastSeg->w * (1.0f - reflect_w_buffer[i]), face_id_buffer[i]);
            lastSeg->nextRefract = seg;
            seg->prev = lastSeg;
            tmpRaySegs.push_back(seg);
        }
        activeRaySegments.clear();
        for (int i = 0; i < tmpRaySegs.size(); i+=2) {
            activeRaySegments.push_back(tmpRaySegs[i]);
        }
        for (int i = 1; i < tmpRaySegs.size(); i+=2) {
            activeRaySegments.push_back(tmpRaySegs[i]);
        }
        tmpRaySegs.clear();

        if (currentRayNumbers > max_ray_num / 2) {
            printf("OVERFLOW! currentRayNumbers: %d\n", currentRayNumbers);
            break;
        }

        memcpy(ray_pt_buffer + currentRayNumbers*3, ray_pt_buffer, currentRayNumbers*3*sizeof(float));
        memcpy(ray_dir_buffer, reflect_dir_buffer, currentRayNumbers*3*sizeof(float));
        memcpy(ray_dir_buffer + currentRayNumbers*3, refract_dir_buffer, currentRayNumbers*3*sizeof(float));

        propagate(currentRayNumbers * 2, ray_pt_buffer, ray_dir_buffer, g->faceNum(), face_data,
            ray_pt_buffer2, face_id_buffer);
        int k = 0;
        for (auto i = 0; i < currentRayNumbers * 2; i++) {
            if (face_id_buffer[i] >= 0 && activeRaySegments[i]->w > 0.001) {
                tmpRaySegs.push_back(activeRaySegments[i]);
                memcpy(ray_pt_buffer + k*3, ray_pt_buffer2 + i*3, 3*sizeof(float));
                if (i != k) {
                    memcpy(ray_dir_buffer + k*3, ray_dir_buffer + i*3, 3*sizeof(float));
                }
                face_id_buffer[k] = face_id_buffer[i];
                g->copyNormalData(1, face_id_buffer + i, face_norm_buffer + k*3);
                k++;
            } else {
                activeRaySegments[i]->isFinished = true;
            }
        }
        currentRayNumbers = k;
        activeRaySegments.swap(tmpRaySegs);
        tmpRaySegs.clear();

        recursion++;
    }

    delete[] face_data;
    delete[] reflect_w_buffer;
    delete[] refract_dir_buffer;
    delete[] reflect_dir_buffer;
    delete[] face_id_buffer;
    delete[] face_norm_buffer;
    delete[] ray_dir_buffer;
    delete[] ray_pt_buffer2;
    delete[] ray_pt_buffer;
    
}

float Optics::getReflectRatio(float inc_angle, float n1, float n2)
{
    float c = std::cos(inc_angle);
    float s = std::sin(inc_angle);
    float d = 1.0f - (n1 / n2 * s) * (n1 / n2 * s);
    d = d <= 0.0f ? 0.0f : d;
    float d_sqrt = std::sqrt(d);

    float Rs = (n1 * c - n2 * d_sqrt) / (n1 * c + n2 * d_sqrt);
    Rs *= Rs;
    float Rp = (n1 * d_sqrt - n2 * c) / (n1 * d_sqrt + n2 * c);
    Rp *= Rp;

    return (Rs + Rp) / 2;
}

void Optics::intersectLineFace(const float *pt, const float *dir, const float *face,
    float *p, float *t, float *alpha, float *beta)
{
    const float *face_point = face;
    float face_base[6];
    LinearAlgebra::vec3FromTo(&face[0], &face[3], &face_base[0]);
    LinearAlgebra::vec3FromTo(&face[0], &face[6], &face_base[3]);

    float a = face_base[0]*face_base[4]*face_point[2] + face_base[1]*face_base[5]*face_point[0] +
        face_base[2]*face_base[3]*face_point[1] - face_base[2]*face_base[4]*face_point[0] -
        face_base[1]*face_base[3]*face_point[2] - face_base[0]*face_base[5]*face_point[1];
    float b = pt[0]*face_base[1]*face_base[5] + pt[1]*face_base[2]*face_base[3] +
        pt[2]*face_base[0]*face_base[4] - pt[0]*face_base[2]*face_base[4] -
        pt[1]*face_base[0]*face_base[5] - pt[2]*face_base[1]*face_base[3];
    float c = dir[0]*face_base[1]*face_base[5] + dir[1]*face_base[2]*face_base[3] +
        dir[2]*face_base[0]*face_base[4] - dir[0]*face_base[2]*face_base[4] -
        dir[1]*face_base[0]*face_base[5] - dir[2]*face_base[1]*face_base[3];
    *t = (a - b) / c;

    a = dir[0]*pt[1]*face_base[5] + dir[1]*pt[2]*face_base[3] +
        dir[2]*pt[0]*face_base[4] - dir[0]*pt[2]*face_base[4] -
        dir[1]*pt[0]*face_base[5] - dir[2]*pt[1]*face_base[3];
    b = dir[0]*face_base[4]*face_point[2] + dir[1]*face_base[5]*face_point[0] +
        dir[2]*face_base[3]*face_point[1] - dir[0]*face_base[5]*face_point[1] -
        dir[1]*face_base[3]*face_point[2] - dir[2]*face_base[4]*face_point[0];
    *alpha = (a + b) / c;

    a = dir[0]*pt[1]*face_base[2] + dir[1]*pt[2]*face_base[0] +
        dir[2]*pt[0]*face_base[1] - dir[0]*pt[2]*face_base[1] -
        dir[1]*pt[0]*face_base[2] - dir[2]*pt[1]*face_base[0];
    b = dir[0]*face_base[1]*face_point[2] + dir[1]*face_base[2]*face_point[0] +
        dir[2]*face_base[0]*face_point[1] - dir[0]*face_base[2]*face_point[1] -
        dir[1]*face_base[0]*face_point[2] - dir[2]*face_base[1]*face_point[0];
    *beta = -(a + b) / c;

    p[0] = pt[0] + *t * dir[0];
    p[1] = pt[1] + *t * dir[1];
    p[2] = pt[2] + *t * dir[2];
}



constexpr float IceRefractiveIndex::_wl[];
constexpr float IceRefractiveIndex::_n[];

float IceRefractiveIndex::n(float waveLength)
{
    if (waveLength < _wl[0]) {
        return 1.0f;
    }

    float nn = 1.0f;
    for (int i = 0; i < sizeof(_wl)/sizeof(float); i++) {
        if (waveLength < _wl[i]) {
            float w1 = _wl[i-1];
            float w2 = _wl[i];
            float n1 = _n[i-1];
            float n2 = _n[i];

            nn = n1 + (n2 - n1) / (w2 - w1) * (waveLength - w1);
            break;
        }
    }

    return nn;
}
