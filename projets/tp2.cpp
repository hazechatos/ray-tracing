#include "vec.h"
#include "color.h"
#include "image.h"
#include "image_io.h"
#include "materials.h"
#include <cmath>

struct Ray
{
    Point o;                // origine
    Vector d;               // direction
    float tmax;             // position de l'extremite, si elle existe. le rayon est un segment / un intervalle [0 tmax]
};

struct Plan
{
    Point a;
    Vector n;
    Material material;
};

struct Sphere
{
    Point c;
    float r;
    Material material;
};

struct Scene
{
    std::vector<Plan> plans;
    std::vector<Sphere> spheres;
    Color bg_color;
};

struct Hit
{
    float t;
    Point p;
    Vector n;
    Material material;
};

struct Emission
{
    Point p;
    Color color;
};

float intersect(const Plan& plan, const Ray& ray)
{
    float t = dot(plan.n, Vector(ray.o, plan.a)) / dot(plan.n, ray.d);
    if (t >= 0 && t < ray.tmax) {
        return t;
    }
    return INFINITY;
};

float intersect(const Sphere& sphere, const Ray& ray)
{
    float a = dot(ray.d, ray.d);
    float b = dot(2 * ray.d, Vector(sphere.c, ray.o));
    float k = dot(Vector(sphere.c, ray.o), Vector(sphere.c, ray.o)) - sphere.r * sphere.r;

    float d = b * b - 4 * a * k;
    if (d <= 0) {
        return INFINITY;
    }
    d = std::sqrt(d);
    float t1 = (-b + d) / (2 * a);
    float t2 = (-b - d) / (2 * a);

    float t = INFINITY;
    if (t1 >= 0 && t1 < t)
        t = t1;
    if (t2 >= 0 && t2 < t)
        t = t2;
    if (t > ray.tmax) {
        return INFINITY;
    }
    return t;
};


Hit intersectScene(const Scene& scene, const Ray& ray)
{
    Hit hit;
    hit.t = INFINITY;

    // plans
    for (const auto& plan : scene.plans)
    {
        float t = intersect(plan, ray);
        if (t < hit.t)
        {
            hit.t = t;
            hit.p = Point(ray.o.x + t * ray.d.x,
                          ray.o.y + t * ray.d.y,
                          ray.o.z + t * ray.d.z);
            hit.n = plan.n;
            hit.material = plan.material;
        }
    }

    // spheres
    for (const auto& sphere : scene.spheres)
    {
        float t = intersect(sphere, ray);
        if (t < hit.t)
        {
            hit.t = t;
            hit.p = Point(ray.o.x + t * ray.d.x,
                          ray.o.y + t * ray.d.y,
                          ray.o.z + t * ray.d.z);
            hit.n = normalize(Vector(sphere.c, hit.p));
            hit.material = sphere.material;
        }
    }

    return hit;
}



float epsilon_point(const Point& p)
{
    // plus grande erreur
    float pmax = std::max(std::abs(p.x), std::max(std::abs(p.y), std::abs(p.z)));
        
    // evalue l'epsilon relatif du point d'intersection
    float pe = pmax * std::numeric_limits<float>::epsilon();
    return pe;
};

Color compute_lr(Point p, Material material, Color emission, Vector n, Point p_light, const Scene& scene) // Calcul de L_r, la lumière réfléchie par le point p        
{
    Point p_eps = { p + epsilon_point(p)*n };
    Vector shadow_v = Vector( p, p_light);
    Ray shadow_ray = { p_eps, shadow_v, length(shadow_v)};
    Hit shadow_cast_hit = intersectScene(scene, shadow_ray);   
    
    float V = 1;
    if (shadow_cast_hit.t < INFINITY) {
        V = 0;  // renvoie 0, si il y a une intersection dans la direction l  
    };

    float cos_theta = std::max(float(0), dot(normalize(n), normalize(shadow_v)));
    Color color = material.diffuse / M_PI * V * emission * cos_theta;
    return color;
}

int main( )
{
    Image image(512, 512);

    // Init scene
    Scene scene;
    scene.plans.push_back({Point(0,-1,0),Vector(0,1,0), Material(Color(0.7)) });
    scene.spheres.push_back({Point(0,0,-3.5),2, Material(Color(Red())) });
    scene.bg_color = Color(0.5);
    
    for(int py= 0; py < image.height(); py++)
    for(int px= 0; px < image.width(); px++)
    {
        // origine du rayon
        Point o= Point(0, 0, 0);
  
        // extremite du rayon pour le pixel (px, py)
        Point e= Point(float(px)/float(image.height())*2-1,float(py)/float(image.width())*2-1, -1);
        
        // Point d'emission de lumiere
        Emission sun = {Point(-2,1,-0.25), Color(5)};
        
        // Direction du rayon et rayon associe
        Vector d= Vector(o, e);
        Ray ray = {o,d,INFINITY};

        // Intersection et calcul de lumiere reflechie
        Hit hit = intersectScene(scene, ray);
        
        if (hit.t < INFINITY) {
            Color l_r = compute_lr(hit.p, hit.material, sun.color, hit.n, sun.p, scene);
            image(px, py) = Color(l_r, 1);
        } else {
            image(px, py) = scene.bg_color;  // Background color
        }
    }
    
    write_image_png(image, "render.png");
    return 0;
}