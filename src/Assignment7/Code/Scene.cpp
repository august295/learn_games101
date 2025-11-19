//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p       = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray&                  ray,
    const std::vector<Object*>& objects,
    float&                      tNear,
    uint32_t&                   index,
    Object**                    hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float    tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear      = tNearK;
            index      = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // TODO Implement Path Tracing Algorithm here
    /**
     * Lecture 16 Ray Tracing 4
     * 00:21:00
     * 蒙特卡洛路径追踪算法
     */
    Intersection inter = intersect(ray);
    if (!inter.happened)
    {
        return backgroundColor;
    }

    if (inter.m->hasEmission())
    {
        return inter.m->getEmission();
    }

    float epsilon = 0.0005f;
    // 观察目标点
    Vector3f p = inter.coords;
    // 入射方向
    Vector3f wo = ray.direction;
    // 直接光照
    Vector3f L_dir(0, 0, 0);
    {
        Intersection lightInter;
        float        pdf_light = 0.0f;
        sampleLight(lightInter, pdf_light);
        // 光源位置
        Vector3f x = lightInter.coords;
        // 光源方向
        Vector3f ws = (x - p).normalized();
        // 检查光线是否被遮挡
        Intersection temp = intersect(Ray(p, ws));
        if ((temp.distance - (x - p).norm()) > -epsilon)
        {
            Vector3f L_i             = lightInter.emit;
            Vector3f f_r             = inter.m->eval(wo, ws, inter.normal);
            float    cos_theta       = dotProduct(ws, inter.normal);
            float    cos_theta_light = dotProduct(-ws, lightInter.normal);
            float    dist2           = dotProduct(x - p, x - p);
            // L
            L_dir = L_i * f_r * cos_theta * cos_theta_light / dist2 / pdf_light;
        }
    }

    // 间接光照
    Vector3f L_indir(0, 0, 0);
    {
        if (get_random_float() < RussianRoulette)
        {
            // 采样新的方向
            Vector3f wi = inter.m->sample(wo, inter.normal);
            // 构造新的光线
            Ray newRay(p, wi);
            // 递归计算该光线的入射辐射度
            Intersection interNew = intersect(newRay);
            if (interNew.happened && !interNew.m->hasEmission())
            {
                // 计算BRDF
                Vector3f f_r       = inter.m->eval(wo, wi, inter.normal);
                float    cos_theta = dotProduct(wi, inter.normal);
                float    pdf       = inter.m->pdf(wo, wi, inter.normal);
                // L
                L_indir = castRay(newRay, depth + 1) * f_r * cos_theta / pdf / RussianRoulette;
            }
        }
    }

    return L_dir + L_indir;
}