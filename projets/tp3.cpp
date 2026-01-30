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
    Material material;
    
    Triangle( const Point& a, const Point& b, const Point& c, const int i, const Material m ) : p(a), e1(Vector(a, b)), e2(Vector(a, c)), id(i), material(m) {}
    
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

float epsilon_point(const Point& p) // Helper function to compute a point at the surface without float errors.
{
    // plus grande erreur
    float pmax = std::max(std::abs(p.x), std::max(std::abs(p.y), std::abs(p.z)));

    // evalue l'epsilon relatif du point d'intersection
    float pe = pmax * std::numeric_limits<float>::epsilon();
    return pe;
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
            
            Material defaultMat = Material(Color(0.9));
            
            triangles.push_back( Triangle(a, b, c, i/3, defaultMat) );
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

    Color compute_L_r(Hit hit, Color L_i, Point p_light) // Calcul de L_r, la lumière réfléchie par le point p        
    {
        // Compute global coordinates and normal
        Triangle tri = triangles[hit.triangle_id];
        Point p = { tri.p.x + hit.u * tri.e1.x + hit.v * tri.e2.x,
                    tri.p.y + hit.u * tri.e1.y + hit.v * tri.e2.y,
                    tri.p.z + hit.u * tri.e1.z + hit.v * tri.e2.z };
        Vector n = normalize(cross(tri.e1, tri.e2));

        // Compute L_r
        Color L_r;
        Color diffusion_color = tri.material.diffuse;
        Point p_eps = { p + epsilon_point(p) * n };
        if (visible(p_eps, p_light)) {
            Vector l = Vector(p_eps, p_light);
            float cos_theta = std::max(float(0), dot(normalize(n), normalize(l)));
            L_r = Color(diffusion_color / M_PI * L_i * cos_theta, 1.f);
        }   
        else { L_r = Color(); }
        return L_r;
    };

};

struct Emission
{
    Point p;
    Color color;
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
        }
        else {
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
    Point camera_origin = Point(1, 0, 2.7);
    Vector camera_dir = Vector(0, 0, -1);
    Camera camera(camera_origin, camera_dir, image.height(), image.width());

    // Init scene
    Transform transform = RotationZ(-90);
    Scene scene("data/cornell.obj", transform);
    
    for(int py= 0; py < image.height(); py++)
    for(int px= 0; px < image.width(); px++)
    {
        // extremite du rayon pour le pixel (px, py)
        Point e = camera.imagePlanePoint(px, py, image.height(), image.width());

        // Point d'emission de lumiere
        Emission sun = {Point(0.5, 0.4, 5), Color(1)};
        //Color sky = Color(2);

        // Intersection et calcul de lumiere reflechie
        Vector d = Vector(camera_origin, e);
        Hit hit = scene.intersect(camera_origin, d, INFINITY);
        
        if (hit.t < INFINITY) {
            //Color l_r = scene compute_L_r_sky(p, Color(1), sky, n, sun.p, scene);
            Color l_r_sun = scene.compute_L_r(hit, sun.color, sun.p);
            image(px, py) = srgb(l_r_sun);
        } else {
            image(px, py) = Color(0.5);
        }
    }
    
    write_image_png(image, "render3.png");
    return 0;
}