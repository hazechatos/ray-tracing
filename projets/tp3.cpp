#include "vec.h"
#include "color.h"
#include "image.h"
#include "image_io.h"
#include "materials.h"
#include "mesh_io.h"
#include "mat.h"
#include <cmath>
#include <random>

struct Ray
{
    Point o;                // origine
    Vector d;               // direction
    float tmax;             // position de l'extremite, si elle existe. le rayon est un segment / un intervalle [0 tmax]
};

struct Source
{
    Point a, b, c;
    Vector n;
    Color emission;
    float area;

    Source (const Point& pa, const Point&pb, const Point&pc, const Color& pemission) : a(pa), b(pb), c(pc), emission(pemission)
    {
        Vector g = cross(Vector(a,b), Vector(a,c));
        float l = length(g);
        area = l/2;
        n=g/2;

        assert(area > 0);
    }
};


struct Hit
{
    int triangle_id;
    float t;
    float u, v;
    
    Hit( ) : triangle_id(-1), t(FLT_MAX), u(0), v(0) {}       // init par defaut, pas d'intersection
    Hit( const int _id, const float _t, const float _u, const float _v ) : triangle_id(_id), t(_t), u(_u), v(_v) {}
    
    operator bool( ) const { return (triangle_id != -1); }    // renvoie vrai si l'intersection est initialisee...
};

struct Triangle
{
    Point p;
    Vector e1, e2;
    int id;
    
    Triangle( const Point& a, const Point& b, const Point& c, const int i ) : p(a), e1(Vector(a, b)), e2(Vector(a, c)), id(i) {}
    
    /* cf Optimizing Ray-Triangle Intersection via Automated Search
        https://perso.univ-lyon1.fr/jean-claude.iehl/Public/educ/M1IMAGE/kensler_triangle.pdf
        
        equations 2b, page 3, colonne de droite
    */
    Hit intersect( const Ray& ray, const float tmax ) const
    {
        Vector op= Vector(ray.o, p);
        float V=  dot( cross(e1, -e2), ray.d);
        float Vp= dot( cross(e1, -e2), op);
        float V1= dot( cross(op, ray.d), -e2);
        float V2= dot( cross(op, ray.d), e1);
        
        float t= Vp / V;
        if(t < 0 || t > tmax) return {};
        
        float u= V1 / V;
        if(u < 0 || u > 1) return {};
        
        float v= V2 / V;
        if(v < 0 || u+v > 1) return {};
        
        return Hit(id, t, u, v);
    }
};

struct Scene
{
    std::vector<Triangle> triangles;
    
    Scene( const char *file, const Transform& transform) // Charger les triangles d'un objet .obj
    {
        std::vector<Point> positions;
        if (!read_positions(file,positions))
            exit(1);
        
        for (unsigned i=0; i< positions.size(); i+=3)
        {
            Point a = transform(positions[i]);
            Point b = transform(positions[i+1]);
            Point c = transform(positions[i+2]);

            Vector n = normalize(cross(Vector(a,b), Vector(a,c)));
            triangles.push_back( Triangle(a, b, c, i/3) );
        }
    }
    
    Hit intersect( const Point&o, const Vector& d, const float tmax )
    {
        Ray ray = { o, d, tmax };
        Hit min_hit;
        min_hit.t = tmax;

        for (const auto& triangle : triangles)
        {
            Hit h = triangle.intersect(ray, tmax);
            if (h && h.t < tmax && h.t < min_hit.t)
            {
                min_hit = h;
            }
        }
        return min_hit;
    }

    bool visible( const Point& p, const Point& q ){
        Vector v = Vector(p,q); 
        Hit h = intersect(p, v, length(v));
        return !(h && h.t < length(v));
    };
};

struct Emission
{
    Point p;
    Color color;
};

float epsilon_point(const Point& p)
{
    // plus grande erreur
    float pmax = std::max(std::abs(p.x), std::max(std::abs(p.y), std::abs(p.z)));
        
    // evalue l'epsilon relatif du point d'intersection
    float pe = pmax * std::numeric_limits<float>::epsilon();
    return pe;
};



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

int main( )
{
    Image image(512, 512);
    Point camera_origin = Point(0, 0, 0);

    // Init scene
    Scene scene("data/robot.obj", Transform());
    
    for(int py= 0; py < image.height(); py++)
    for(int px= 0; px < image.width(); px++)
    {
        // extremite du rayon pour le pixel (px, py)
        Point e= Point(float(px)/float(image.height())*2-1,float(py)/float(image.width())*2-1, -1);
        Vector d= Vector(camera_origin, e);

        // Point d'emission de lumiere
        Emission sun = {Point(-2,6,-0.25), Color(2)};
        Color sky = Color(2);

        // Intersection et calcul de lumiere reflechie
        Hit hit = scene.intersect(camera_origin, d, INFINITY);
        
        if (hit.t < INFINITY) {
            Triangle tri = scene.triangles[hit.triangle_id];
            Point p = { tri.p.x + hit.u * tri.e1.x + hit.v * tri.e2.x,
                        tri.p.y + hit.u * tri.e1.y + hit.v * tri.e2.y,
                        tri.p.z + hit.u * tri.e1.z + hit.v * tri.e2.z };
            Vector n = normalize(cross(tri.e1, tri.e2));
            Color l_r = compute_L_r_sky(p, Color(1), sky, n, sun.p, scene);
            Color l_r_sun = compute_L_r(p, Color(1), sun.color, n, sun.p, scene);
            image(px, py) = srgb(l_r+l_r_sun);
        } else {
            image(px, py) = Color(1);
        }
    }
    
    write_image_png(image, "render.png");
    return 0;
}