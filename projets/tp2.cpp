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

bool visible(const Scene& scene, Point p, Vector l) {
    Ray ray = { p, l, INFINITY };
    Hit shadow_cast_hit = intersectScene(scene, ray);
    return !(shadow_cast_hit.t < INFINITY);
}

Color compute_L_r(Point p, Material material, Color L_i, Vector n, Point p_light, const Scene& scene) // Calcul de L_r, la lumière réfléchie par le point p        
{
    Point p_eps = { p + epsilon_point(p)*n };
    Vector l = Vector( p_eps, p_light);
    bool V = visible(scene, p_eps, l);
    float cos_theta = std::max(float(0), dot(normalize(n), normalize(l)));
    Color L_r = { material.diffuse / M_PI * V * L_i * cos_theta , 1};
    return L_r;
};

float fract(const float v) { return v - std::floor(v); }

// renvoie la ieme direction parmi n
Vector fibonacci(const int i, const int N)
{
    const float ratio = (std::sqrt(5) + 1) / 2;

    float phi = float(2 * M_PI) * fract(i / ratio);
    float cos_theta = 1 - float(2 * i + 1) / float(N * 2);
    float sin_theta = std::sqrt(1 - cos_theta * cos_theta);

    return Vector(std::cos(phi) * sin_theta, std::sin(phi) * sin_theta, cos_theta);
}

Color compute_L_r_sky(Point p, Material material, Color L_i, Vector n, Point p_light, const Scene& scene) // Calcul de L_r, la lumière réfléchie par le point p        
{
    Color L_r;
    int N = 256;
    for (int i = 0; i < N; i++) {
        Vector f = fibonacci(i, N);
        Point p_eps = { p + epsilon_point(p) * n };
        bool V = visible(scene, p_eps, f);
        float cos_theta = std::max(float(0), dot(normalize(n), normalize(f)));
        L_r = L_r + material.diffuse / M_PI * V * L_i * cos_theta;
}
    L_r = L_r / float(N);
    return Color(L_r,1);
}



struct Camera {
    Point o;
    Vector d;
    int imageHeight;
    int imageWidth;

    Camera(const Point& a, const Vector& u, const int i, const int j) : o(a), d(u), imageHeight(i), imageWidth(j) {};
    
    Point imagePlanePoint(int px, int py, int imH, int imW) {
        Vector dir = normalize(d);
        Vector up, right;
        
        // Choose up vector that's not parallel to direction
        if (std::abs(dir.z) < 0.9f) {
            // dir is NOT pointing up/down, so using Z-axis as reference is safe
            right = normalize(cross(dir, Vector(0, 0, 1)));
            up = normalize(cross(right, dir));
        } else {
            // dir IS pointing up/down (dir.z ≈ ±1), so use Y-axis instead
            up = normalize(cross(dir, Vector(0, 1, 0)));
            right = normalize(cross(dir, up));
        }
               
        float aspect = float(imW) / float(imH);
        float x = (2.0f * (px + 0.5f) / imW - 1.0f) * aspect;
        float y = 2.0f * (py + 0.5f) / imH - 1.0f;
        
        // Image plane at distance 1.0 from camera
        Point imagePoint = o + dir + x * right + y * up;
        
        return imagePoint;
    }
};

int main( )
{
    Image image(512, 512);
    Point camera_origin = Point(0, 0, 1);
    Vector camera_dir = Vector(1, 0, 0);
    Camera camera(camera_origin, camera_dir, image.height(), image.width());

    // Init scene
    Scene scene;
    scene.plans.push_back({Point(0,0,-1),Vector(0,0,1), Material(Color(0.9)) });
    scene.spheres.push_back({Point(7,0,0),2, Material(Color(Red())) });
    scene.bg_color = Color(1);
    
    for(int py= 0; py < image.height(); py++)
    for(int px= 0; px < image.width(); px++)
    {
        // origine du rayon
        Point o= Point(0, 0, 0);
  
        // extremite du rayon pour le pixel (px, py)
        Point e = camera.imagePlanePoint(px, py, image.height(), image.width());
        
        // Point d'emission de lumiere
        Emission sun = {Point(-2,6,10), Color(2)};
        Color sky = Color(2);
        
        // Direction du rayon et rayon associe
        Vector d= Vector(camera_origin, e);
        Ray ray = {camera_origin,d,INFINITY};

        // Intersection et calcul de lumiere reflechie
        Hit hit = intersectScene(scene, ray);
        
        if (hit.t < INFINITY) {
            Color l_r = compute_L_r_sky(hit.p, hit.material, sky, hit.n, sun.p, scene);
            Color l_r_sun = compute_L_r(hit.p, hit.material, sun.color, hit.n, sun.p, scene);
            image(px, py) = srgb(l_r+l_r_sun);
        } else {
            image(px, py) = scene.bg_color;
        }
    }
    
    write_image_png(image, "render.png");
    return 0;
}