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
#include <chrono>

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
    Triangle( const Point& a, const Point& b, const Point& c, const int i) : p(a), e1(Vector(a, b)), e2(Vector(a, c)), id(i), material() {}

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

// intersection avec un englobant
struct BBoxHit
{
    float tmin, tmax;
    
    operator bool( ) const { return tmin <= tmax; }
};


// englobant aligne sur les axes
struct BBox
{
    Point pmin, pmax;
    
    BBox( ) : pmin(INFINITY, INFINITY, INFINITY), pmax(-INFINITY, -INFINITY, -INFINITY) {}
    BBox( const Point& p ) : pmin(p), pmax(p) {}
    BBox( const BBox& a, const BBox& b ) : pmin( min(a.pmin, b.pmin) ), pmax( max(a.pmax, b.pmax) ) {}
    
    BBox& grow( const Point& p ) { pmin= min(p, pmin); pmax= max(p, pmax); return *this; }
    BBox& grow( const BBox& b ) { pmin= min(pmin, b.pmin); pmax= max(pmax, b.pmax); return *this; }
    
    float area( ) const // aire de la surface de l'englobant
    {
        Vector d= Vector(pmin, pmax);
        assert(d.x >= 0); assert(d.y >= 0); assert(d.z >= 0);
        
        return 2*d.x*d.y + 2*d.x*d.z + 2*d.y*d.z;
    }
    
    Point centroid( ) const { return (pmin + pmax) / 2; }
    float centroid( const int axis ) const { return (pmin(axis) + pmax(axis)) / 2; }
    
    BBoxHit intersect( const Ray& ray, const float htmax ) const
    {
        Point rmin= pmin;
        Point rmax= pmax;
        
        // verifier la direction du rayon, sur chaque axe
        // echanger tmin, tmax, si la direction est < 0
        if(ray.d.x < 0) std::swap(rmin.x, rmax.x);      // rmin.x <= rmax.x
        if(ray.d.y < 0) std::swap(rmin.y, rmax.y);      // rmin.y <= rmax.y
        if(ray.d.z < 0) std::swap(rmin.z, rmax.z);      // rmin.z <= rmax.z
        
        // intersection avec les plans
        Vector invd= Vector(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);
        Vector dmin= (rmin - ray.o) * invd;
        Vector dmax= (rmax - ray.o) * invd;
        
        // intersection des 4 intervalles... 3 pour les paires de plans + intervalle du rayon
        float tmin= std::max(dmin.z, std::max(dmin.y, std::max(dmin.x, float(0))));
        float tmax= std::min(dmax.z, std::min(dmax.y, std::min(dmax.x, htmax)));
        
        // si l'intervalle est valide, il y a intersection
        return { tmin, tmax };
    }    
};


// renvoie l'englobant du triangle
BBox triangle_bounds( const Triangle& triangle )
{
    BBox bounds(triangle.p);
    bounds.grow(triangle.p + triangle.e1);
    bounds.grow(triangle.p + triangle.e2);
    return bounds;
}

// renvoie le centre de l'englobant du triangle
Point triangle_centroid( const Triangle& triangle )  { return triangle_bounds(triangle).centroid(); }
// idem, mais renvoie la coordonnee sur l'axe
float triangle_centroid( const Triangle& triangle, const int axis ) { return triangle_bounds(triangle).centroid(axis); }	// todo : plus rapide !


// representation d'un noeud ou d'une feuille du bvh
struct Node
{
    BBox bounds;                                                // englobant
    int left;                                                   // fils gauche / premier triangle
    int right;                                                  // fils droit / dernier triangle
    // astuce d'encodage : left/right négatifs pour les feuilles

    bool leaf( ) const { return right < 0; }                    // renvoie vrai si le noeud est une feuille
    int leaf_begin( ) const { assert(leaf()); return -left; }   // renvoie l'indice du premier triangle de la feuille
    int leaf_end( ) const { assert(leaf()); return -right; }    // renvoie l'indice du dernier triangle de la feuille
    
    bool node( ) const { return !leaf(); }                      // renvoie vrai si le noeud est un noud interne / pas une feuille
    int node_left( ) const { assert(node()); return left; }     // renvoie l'indice du fils gauche
    int node_right( ) const { assert(node()); return right; }   // renvoie l'indice du fils droit
};

// renvoie un noeud interne
Node make_node( const BBox& bounds, const int left, const int right ) 
{ 
    Node node= { bounds, left, right }; 
    assert(node.node());
    return node;
}

// renvoie une feuille
Node make_leaf( const BBox& bounds, const int begin, const int end )
{
    Node node= { bounds, -begin, -end };
    assert(node.leaf());
    return node;
}


struct BVH
{
    std::vector<Triangle> triangles;
    std::vector<Node> nodes;
    int root;
    int last_node;
    
    BVH( ) : triangles(), nodes(), root(-1), last_node(-1) {}
    
    void build( const std::vector<Triangle>& ptriangles )
    {
        triangles= ptriangles;				// copie les triangles pour les trier
        nodes.resize(triangles.size()*2);		// alloue tous les noeuds
        
        auto cpu_start= std::chrono::high_resolution_clock::now();
            root= 0;
            last_node= 0;
            build( root, 0, triangles.size() );
        
        auto cpu_stop= std::chrono::high_resolution_clock::now();
        auto cpu_time= std::chrono::duration_cast<std::chrono::milliseconds>(cpu_stop - cpu_start).count();
        printf("cpu  %ds %03dms\n", int(cpu_time / 1000), int(cpu_time % 1000));
        printf("BVH: build %d nodes, %u triangles\n", last_node, unsigned(triangles.size()));
    }
    
    Hit intersect( const Ray& ray )
    {
        Hit hit;
        hit.t= ray.tmax;
        intersect( nodes[root], ray, hit );
        return hit;
    }
    
    bool occluded( const Ray& ray ) { return occluded(nodes[root], ray); }
    
protected:
    void intersect( const Node& node, const Ray& ray, Hit& hit )
    {
        if(node.bounds.intersect(ray, hit.t))
        {
            if(node.leaf())
            {
                for(int i= node.leaf_begin(); i < node.leaf_end(); i++)
                    if(Hit h= triangles[i].intersect(ray, hit.t))
                        hit= h;
            }
            else
            {
                BBoxHit left_hit=  nodes[node.left].bounds.intersect(ray, hit.t);
                BBoxHit right_hit= nodes[node.right].bounds.intersect(ray, hit.t);

                // visiter le fils le plus proche en premier
                if(left_hit && right_hit)
                {
                    if(left_hit.tmin > right_hit.tmin)
                    {
                        intersect(nodes[node.right], ray, hit);
                        if(left_hit.tmin < hit.t)
                            intersect(nodes[node.left], ray, hit);
                    }
                    else
                    {
                        intersect(nodes[node.left], ray, hit);
                        if(right_hit.tmin < hit.t)
                            intersect(nodes[node.right], ray, hit);
                    }
                }
                else if(left_hit)
                    intersect(nodes[node.left], ray, hit);
                else if(right_hit)
                    intersect(nodes[node.right], ray, hit);
            }
        }
    }
    
    bool occluded( const Node& node, const Ray& ray )
    {
        Hit h = {};
        intersect(node, ray, h);
        return h; // true if there is an intersection, false otherwise
    }
    
    void build( const int index, const int begin, const int end )
    {
        assert(unsigned(index) < nodes.size());
        
        // englobant des centres des triangles
        BBox centroid_bounds;
        for(int i= begin; i < end; i++)
            centroid_bounds.grow( triangle_centroid(triangles[i]) );
        
        // axe le plus etire de l'englobant
        Vector d= Vector( centroid_bounds.pmin, centroid_bounds.pmax );
        int axis;
        if(d.x > d.y && d.x > d.z)      // x plus grand que y et z ?
            axis= 0;
        else if(d.y > d.z)              // y plus grand que z ?
            axis= 1;
        else                            // ni x, ni y
            axis= 2;
        
        // construit une feuille si l'englobant est degenere ou s'il reste peu de triangles
        if(d(axis) == 0 || end - begin <= 4)
        {
            // englobant des triangles
            BBox bounds;
            for(int i= begin; i < end; i++)
                bounds.grow( triangle_bounds( triangles[i]) );
            
            nodes[index]= make_leaf( bounds, begin, end );
            return;
        }
        
        // repartit les triangles par rapport au centre de l'englobant des centroides
        float mid_value= centroid_bounds.centroid(axis);
        Triangle *mid_ptr= std::partition( triangles.data() + begin, triangles.data() + end,
            [axis, mid_value]( const Triangle& t ) { return triangle_centroid(t, axis) < mid_value; } );
        int mid= int(mid_ptr - triangles.data());

        // si tous les triangles sont du même côté (centroides confondus), coupe en deux
        if(mid == begin || mid == end)
            mid= (begin + end) / 2;

        // reserve les fils
        int left= ++last_node;
        int right= ++last_node;
	
        // et on recommence, tant qu'il reste des triangles
        build(left, begin, mid);
        build(right, mid, end);
        
        // construit le noeud
        nodes[index]= make_node( BBox( nodes[left].bounds, nodes[right].bounds ), left, right );
    }
};



struct Scene
{
    std::vector<Source> sources;
    
    BVH bvh;    
    MeshIOData mesh;
    std::vector<Image> images;
   
    Scene( const char *file, const Transform& transform = Identity() )
    {
        if(!read_meshio_data(file, mesh))
            exit(1);
        read_images(mesh.materials, images);

        // applique la transformation aux positions et normales
        for(auto& p : mesh.positions)
            p = transform(p);
        for(auto& n : mesh.normals)
            n = transform(n);

        // construit les triangles et le bvh
        std::vector<Triangle> triangles;
        for(unsigned i= 0; i + 2 < mesh.indices.size(); i+= 3)
        {
            unsigned a= mesh.indices[ i ];
            unsigned b= mesh.indices[ i +1 ];
            unsigned c= mesh.indices[ i +2 ];
            Triangle triangle( mesh.positions[ a ], mesh.positions[ b ], mesh.positions[ c ], i/3 );
            
            Vector n= cross( triangle.e1, triangle.e2 );
            if(length(n) > 0) // ne conserve que les triangles non degeneres !
                triangles.push_back( triangle );
        }
        
        printf("%u/%u triangles\n", unsigned(triangles.size()), unsigned(mesh.indices.size()/3));
        bvh.build(triangles);
        
        // verifie les textures...
        if(mesh.texcoords.size() != mesh.positions.size())
            mesh.texcoords.clear();     // todo : bug dans le parser si tous les objets n'ont pas les memes attributs
        
        for(unsigned i= 0; i < images.size(); i++)
            if(images[i].size() == 0)
            {
                // desactiver l'utilisation des textures s'il manque des images
                images.clear();
                mesh.texcoords.clear();
                break;
            }
    }
    
    // intersections
    Hit intersect( const Ray& ray ) { return bvh.intersect(ray); }
    bool visible( const Ray& ray ) { return !bvh.occluded(ray); }
    bool occluded( const Ray& ray ) { return bvh.occluded(ray); }
    
    // matiere du point d'intersection
    const Material& material( const Hit& hit ) { assert(hit == true); return mesh.materials( mesh.material_indices[hit.triangle_id] ); }
    Color diffuse( const Hit& hit ) { return material(hit).diffuse; }   // couleur au point d'intersection
    Color emisison( const Hit& hit ) {return material(hit).emission; }    // emission
    
    // normale 
    // todo : calculer la normale geometrique du triangle si les normales des sommets ne sont pas chargees...
    Vector normal( const Hit& hit )
    {
        assert(hit == true);
        if(mesh.normals.size() == 0)
            return {};
        
        unsigned ia= mesh.indices[ 3*hit.triangle_id ];
        unsigned ib= mesh.indices[ 3*hit.triangle_id +1 ];
        unsigned ic= mesh.indices[ 3*hit.triangle_id +2 ];
        
        float w= 1 - hit.u - hit.v;
        return w * mesh.normals[ia] + hit.u * mesh.normals[ib] + hit.v * mesh.normals[ic];
    }
    
    // coordonnees de texture
    // todo : renvoyer une couleur par defaut / blanc si les textures ne sont pas chargees...
    Point texcoord( const Hit& hit )
    {
        assert(hit == true);
        if(mesh.texcoords.size() == 0)
            return {};
        
        unsigned ia= mesh.indices[ 3*hit.triangle_id ];
        unsigned ib= mesh.indices[ 3*hit.triangle_id +1 ];
        unsigned ic= mesh.indices[ 3*hit.triangle_id +2 ];
        
        float w= 1 - hit.u - hit.v;
        return w * mesh.texcoords[ia] + hit.u * mesh.texcoords[ib] + hit.v * mesh.texcoords[ic];
    }


    Color compute_L_r_one_source(Point p, Vector n, Material material, Source source,
                            int N_iter,
                            std::default_random_engine& rng,
                            std::uniform_real_distribution<float>& dist)
    {
        Color L = {};

        Vector n_source = normalize(source.n);
        float pdf = 1.0f / source.area;

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

            Vector shadow = Vector(p_eps, q_eps);
            float shadow_dist = length(shadow);
            if (!occluded(Ray{p_eps, shadow / shadow_dist, shadow_dist})) {
                Vector l = shadow;
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
        int id = hit.triangle_id;
        Point a= mesh.positions[ mesh.indices[3*id] ];
        Vector e1= Vector(a, mesh.positions[ mesh.indices[3*id+1] ]);
        Vector e2= Vector(a, mesh.positions[ mesh.indices[3*id+2] ]);
        Point p = { a.x + hit.u * e1.x + hit.v * e2.x,
                    a.y + hit.u * e1.y + hit.v * e2.y,
                    a.z + hit.u * e1.z + hit.v * e2.z };

        // normale interpolée si disponible, sinon normale géométrique
        Vector n = normal(hit);
        if (dot(n, n) == 0.0f)
            n = normalize(cross(e1, e2));
        else
            n = normalize(n);

        const Material& mat = material(hit);


        Color L_r = {};
        int N_iter = 32;
        
        // Random number generator
        static std::random_device hwseed;
        static std::default_random_engine rng(hwseed());
        std::uniform_real_distribution<float> uniform(0.0f, 1.0f);
        
        // Monte Carlo integration over light sources
        for (const auto& source : sources) 
        {
            L_r = L_r + compute_L_r_one_source(p, n, mat, source, N_iter, rng, uniform);
        }

        // Add material's own emission
        L_r = L_r + mat.emission;

        return L_r;
    }

    Color compute_L_r_sky_sample(Point p, Vector n, Material material, Color sky_color,
                                 int N_iter,
                                 std::default_random_engine& rng,
                                 std::uniform_real_distribution<float>& dist)
    {
        Color L = {};

        // base orthonormée avec n comme axe Z
        Vector tangent   = (std::abs(n.x) < 0.9f) ? normalize(cross(n, Vector(1, 0, 0))) : normalize(cross(n, Vector(0, 1, 0)));
        Vector bitangent = normalize(cross(n, tangent));

        Point p_eps = p + epsilon_point(p) * n;

        for (int i = 0; i < N_iter; i++) {
            float u1 = dist(rng);
            float u2 = dist(rng);

            // échantillonnage cosinus-pondéré : pdf = cos_theta / pi
            float cos_theta = std::sqrt(u1);
            float sin_theta = std::sqrt(1.0f - u1);
            float phi = 2.0f * M_PI * u2;

            Vector wi = sin_theta * std::cos(phi) * tangent +
                        sin_theta * std::sin(phi) * bitangent +
                        cos_theta * n;

            // contrib = (diffuse/pi) * sky * cos_theta / pdf
            //         = (diffuse/pi) * sky * cos_theta / (cos_theta/pi)
            //         = diffuse * sky
            if (!occluded(Ray{p_eps, wi, 1000.0f}))
                L = L + material.diffuse * sky_color;
        }

        return L / N_iter;
    }

    Color compute_L_r_sky(Hit hit, Color sky_color)
    {
        int id = hit.triangle_id;
        Point a  = mesh.positions[ mesh.indices[3*id] ];
        Vector e1= Vector(a, mesh.positions[ mesh.indices[3*id+1] ]);
        Vector e2= Vector(a, mesh.positions[ mesh.indices[3*id+2] ]);
        Point p = { a.x + hit.u * e1.x + hit.v * e2.x,
                    a.y + hit.u * e1.y + hit.v * e2.y,
                    a.z + hit.u * e1.z + hit.v * e2.z };

        Vector n = normal(hit);
        if (dot(n, n) == 0.0f)
            n = normalize(cross(e1, e2));
        else
            n = normalize(n);

        const Material& mat = material(hit);

        int N_iter = 64;
        static std::random_device hwseed;
        static std::default_random_engine rng(hwseed());
        std::uniform_real_distribution<float> uniform(0.0f, 1.0f);

        return compute_L_r_sky_sample(p, n, mat, sky_color, N_iter, rng, uniform);
    }
};

// todo : re-construire les englobants des triangles constamment est tres long, il est plus rapide de les stocker au debut de la construction...

struct Emission
{
    Point p;
    Color color;
};

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
    Image image(1920, 1080);
    Point camera_origin = Point(1045.03, -192.667, 120.173);
    Vector camera_dir = Vector(-1, 0, 0);
    Camera camera(camera_origin, camera_dir, image.height(), image.width());

    // Init scene
    Transform transform = RotationX( 90 );
    Scene scene("data/PipersAlley/PipersAlley.obj", transform);
    
    std::cout << "[DEBUG] Starting rendering: " << image.width() << " x " << image.height() << " pixels\n";
    
    for(int py= 0; py < image.height(); py++)
    {
        if (py % 4 == 0) std::cout << "[DEBUG] Rendering progress: " << py << "/" << image.height() << " rows completed\n";
        
        for(int px= 0; px < image.width(); px++)
        {
            // extremite du rayon pour le pixel (px, py)
            Point e = camera.imagePlanePoint(px, py, image.height(), image.width());

            // Intersection et calcul de lumiere reflechie
            Vector d = Vector(camera_origin, e);
            Hit hit = scene.intersect(Ray {camera_origin, d, INFINITY});
            
            if (hit.t < INFINITY) {
                
                Color L_r = scene.compute_L_r_sources(hit);
                Color L_sky = scene.compute_L_r_sky(hit, Color(0.5));  // Sky color
                image(px, py) = srgb(L_r + L_sky);
            } else {
                image(px, py) = Color(0.2);
            }
        }
    }
    
    write_image_png(image, "render4.png");
    std::cout << "[DEBUG] Rendering complete! Image saved to render4.png\n";
    return 0;
}