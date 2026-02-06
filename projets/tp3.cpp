#include "vec.h"
#include "color.h"
#include "image.h"
#include "image_io.h"
#include "materials.h"
#include "mesh_io.h"
#include "mat.h"
#include <cmath>
#include <random>
#include <iostream>

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

float epsilon_point(const Point& p)
{
    float pmax = std::max({ std::abs(p.x), std::abs(p.y), std::abs(p.z) });
    return std::max(1e-4f, 1e-5f * pmax);
}


struct Scene
{
    std::vector<Triangle> triangles;
    std::vector<Source> sources;
    Materials materials;
    std::vector<int> material_indices;   // indices des matieres


    Scene( const char *file, const Transform& transform) // Charger les triangles d'un objet .obj
    {
        // Load positions
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

        // Load materials
        if (!read_materials(file, materials, material_indices))
            exit(1);
        
        // Load sources (emissive triangles)
        for (unsigned j = 0; j < triangles.size(); j++)
        {
            Color emission = materials(material_indices[j]).emission;
            if (emission.r > 0 || emission.g > 0 || emission.b > 0) {
                sources.push_back(Source(triangles[j].p, triangles[j].p + triangles[j].e1, triangles[j].p + triangles[j].e2, emission));
                std::cout << "[DEBUG] Source found with material: " << emission.r << emission.g << emission.b << "\n";
            }
        }
        std::cout << "[DEBUG] Scene loaded: " << triangles.size() << " triangles, " << sources.size() << " light sources\n";
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
        // Don't block visibility if the hit is a light source (emissive triangle)
        if (h) {
            int tri_id = h.triangle_id;
            Color emission = materials(material_indices[tri_id]).emission;
            if (emission.r > 0 || emission.g > 0 || emission.b > 0) {
                return true;  // Light sources don't block
            }
            return !(h && h.t < length(v));
        }
        return true;  // No hit = visible
    };


    Color compute_L_r_one_source(Point p, Vector n, Material material, Source source,
                                int N_iter,
                                std::default_random_engine& rng,
                                std::uniform_real_distribution<float>& dist)
    {
        Color L = {};

        Vector x = Vector(source.a, source.b);
        Vector y = Vector(source.a, source.c);
        Vector n_source = normalize(cross(x, y));
        float area = 0.5f * length(cross(x, y));
        float pdf = 1.0f / area;

        for (int i = 0; i < N_iter; i++) {
            float u1 = dist(rng);
            float u2 = dist(rng);

            // uniform triangle sample
            float su1 = std::sqrt(u1);
            float b0 = 1.0f - su1;
            float b1 = u2 * su1;
            float b2 = 1.0f - b0 - b1;

            Point q = b0 * source.a + b1 * source.b + b2 * source.c;

            Point p_eps = p + epsilon_point(p) * n;
            Point q_eps = q + epsilon_point(q) * n_source;

            if (visible(p_eps, q_eps)) {
                Vector l = q_eps - p_eps;
                float dist2 = dot(l, l);
                Vector wi = normalize(l);

                float cos_theta = std::max(0.0f, dot(n, wi));
                float cos_theta_source = std::max(0.0f, dot(n_source, -wi));

                Color contrib = (material.diffuse / M_PI) * source.emission
                              * cos_theta * cos_theta_source / dist2 / pdf;

                L = L + contrib;
            }
        }

        return L / N_iter;
    }


    Color compute_L_r_sources(Hit hit) 
    {
        // Compute global coordinates and normal
        Triangle tri = triangles[hit.triangle_id];
        Point p = { tri.p.x + hit.u * tri.e1.x + hit.v * tri.e2.x,
                    tri.p.y + hit.u * tri.e1.y + hit.v * tri.e2.y,
                    tri.p.z + hit.u * tri.e1.z + hit.v * tri.e2.z };
        Vector n = normalize(cross(tri.e1, tri.e2));
        int material_id = material_indices[hit.triangle_id];
        Material& material = materials(material_id);
        
        Color L_r = {};
        int N_iter = 32;
        
        // Random number generator
        static std::random_device hwseed;
        static std::default_random_engine rng(hwseed());
        std::uniform_real_distribution<float> uniform(0.0f, 1.0f);
        
        // Monte Carlo integration over light sources
        for (const auto& source : sources) 
        {
            L_r = L_r + compute_L_r_one_source(p, n, material, source, N_iter, rng, uniform);
        }

        // Add material's own emission
        L_r = L_r + material.emission;

        return L_r;
    }
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
    
    std::cout << "[DEBUG] Starting rendering: " << image.width() << " x " << image.height() << " pixels\n";
    
    for(int py= 0; py < image.height(); py++)
    {
        if (py % 4 == 0) std::cout << "[DEBUG] Rendering progress: " << py << "/" << image.height() << " rows completed\n";
        
        for(int px= 0; px < image.width(); px++)
        {
            // extremite du rayon pour le pixel (px, py)
            Point e = camera.imagePlanePoint(px, py, image.height(), image.width());

            // Point d'emission de lumiere
            Emission sun = {Point(0.5, 0.4, 5), Color(1)};

            // Intersection et calcul de lumiere reflechie
            Vector d = Vector(camera_origin, e);
            Hit hit = scene.intersect(camera_origin, d, INFINITY);
            
            if (hit.t < INFINITY) {
                
                Color L_r = scene.compute_L_r_sources(hit);
                //Color l_r_sun = scene.compute_L_r(hit, sun.color, sun.p);
                image(px, py) = srgb(L_r);
            } else {
                image(px, py) = Color(0.2);
            }
        }
    }
    
    write_image_png(image, "render3.png");
    std::cout << "[DEBUG] Rendering complete! Image saved to render3.png\n";
    return 0;
}