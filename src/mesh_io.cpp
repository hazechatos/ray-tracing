
#include <cstdlib>
#include <cassert>
#include <map>

#include "mesh_io.h"

#include "image.h"
#include "image_io.h"


bool read_positions( const char *filename, std::vector<Point>& positions )
{
    positions.clear();
    
    FILE *in= fopen(filename, "rt");
    if(!in)
    {
        printf("[error] loading mesh '%s'...\n", filename);
        return false;
    }
    
    printf("loading mesh '%s'...\n", filename);
    
    std::vector<Point> wpositions;
    std::vector<int> wp;
    
    char line_buffer[1024];
    bool error= true;
    for(;;)
    {
        // charge une ligne du fichier
        if(!fgets(line_buffer, sizeof(line_buffer), in))
        {
            error= false;       // fin du fichier, pas d'erreur detectee
            break;
        }
        
        // force la fin de la ligne, au cas ou
        line_buffer[sizeof(line_buffer) -1]= 0;
        
        // saute les espaces en debut de ligne
        char *line= line_buffer;
        while(*line && isspace(*line))
            line++;
        
        if(line[0] == 'v')
        {
            float x, y, z;
            if(line[1] == ' ')          // position x y z
            {
                if(sscanf(line, "v %f %f %f", &x, &y, &z) != 3)
                    break;
                wpositions.push_back( Point(x, y, z) );
            }
        }
        
        else if(line[0] == 'f')         // triangle a b c, les sommets sont numerotes a partir de 1 ou de la fin du tableau (< 0)
        {
            wp.clear();
            
            int next= 0;
            for(line= line +1; ; line= line + next)
            {
                wp.push_back(0); 
                
                // analyse les attributs du sommet : p/t/n ou p//n ou p/t ou p...
                next= 0;
                int wt= 0;
                int wn= 0;
                if(sscanf(line, " %d/%d/%d %n", &wp.back(), &wt, &wn, &next) == 3) 
                    continue;
                else if(sscanf(line, " %d/%d %n", &wp.back(), &wt, &next) == 2)
                    continue;
                else if(sscanf(line, " %d//%d %n", &wp.back(), &wn, &next) == 2)
                    continue;
                else if(sscanf(line, " %d %n", &wp.back(), &next) == 1)
                    continue;
                else if(next == 0)      // fin de ligne
                    break;
            }
            
            // triangule la face
            for(unsigned v= 2; v +1 < wp.size(); v++)
            {
                unsigned idv[3]= { 0, v -1, v };
                for(unsigned i= 0; i < 3; i++)
                {
                    unsigned k= idv[i];
                    int p= (wp[k] < 0) ? int(wpositions.size()) + wp[k] : wp[k] -1;
                    if(p < 0) break; // error
                    
                    // et duplique les positions...
                    assert(p < int(wpositions.size()));
                    positions.push_back(wpositions[p]);
                }
            }
        }
    }
    
    fclose(in);
    
    if(error)
        printf("[error] loading mesh '%s'...\n%s\n\n", filename, line_buffer);
    else
        printf("mesh '%s': %d positions\n", filename, int(positions.size()));

    return !error;
}


bool read_indexed_positions( const char *filename, std::vector<Point>& positions, std::vector<unsigned>& indices )
{
    positions.clear();
    indices.clear();
    
    FILE *in= fopen(filename, "rt");
    if(!in)
    {
        printf("[error] loading indexed mesh '%s'...\n", filename);
        return false;
    }
    
    printf("loading indexed mesh '%s'...\n", filename);
    
    std::vector<int> wp;
    
    char line_buffer[1024];
    bool error= true;
    for(;;)
    {
        // charge une ligne du fichier
        if(!fgets(line_buffer, sizeof(line_buffer), in))
        {
            error= false;       // fin du fichier, pas d'erreur detectee
            break;
        }
        
        // force la fin de la ligne, au cas ou
        line_buffer[sizeof(line_buffer) -1]= 0;
        
        // saute les espaces en debut de ligne
        char *line= line_buffer;
        while(*line && isspace(*line))
            line++;
        
        if(line[0] == 'v')
        {
            float x, y, z;
            if(line[1] == ' ')          // position x y z
            {
                if(sscanf(line, "v %f %f %f", &x, &y, &z) != 3)
                    break;
                positions.push_back( Point(x, y, z) );
            }
        }
        
        else if(line[0] == 'f')         // triangle a b c, les sommets sont numerotes a partir de 1 ou de la fin du tableau (< 0)
        {
            wp.clear();
            
            int next= 0;
            for(line= line +1; ; line= line + next)
            {
                wp.push_back(0); 
                
                // analyse les attributs du sommet : p/t/n ou p//n ou p/t ou p...
                next= 0;
                int wt= 0;
                int wn= 0;
                if(sscanf(line, " %d/%d/%d %n", &wp.back(), &wt, &wn, &next) == 3) 
                    continue;
                else if(sscanf(line, " %d/%d %n", &wp.back(), &wt, &next) == 2)
                    continue;
                else if(sscanf(line, " %d//%d %n", &wp.back(), &wn, &next) == 2)
                    continue;
                else if(sscanf(line, " %d %n", &wp.back(), &next) == 1)
                    continue;
                else if(next == 0)      // fin de ligne
                    break;
            }
            
            // triangule la face
            for(unsigned v= 2; v +1 < wp.size(); v++)
            {
                unsigned idv[3]= { 0, v -1, v };
                for(unsigned i= 0; i < 3; i++)
                {
                    unsigned k= idv[i];
                    int p= (wp[k] < 0) ? int(positions.size()) + wp[k] : wp[k] -1;
                    if(p < 0) break; // error
                    
                    assert(p < int(positions.size()));
                    indices.push_back(p);
                }
            }
        }
    }
    
    fclose(in);
    
    if(error)
        printf("[error] loading indexed mesh '%s'...\n%s\n\n", filename, line_buffer);
    else
        printf("indexed mesh '%s': %d positions, %d indices\n", filename, int(positions.size()), int(indices.size()));

    return !error;
}


bool read_materials_mtl( const char *filename, Materials& materials  )
{
    FILE *in= fopen(filename, "rt");
    if(!in)
    {
        printf("[error] loading materials '%s'...\n", filename);
        return false;
    }
    
    printf("loading materials '%s'...\n", filename);
    
    Material *material= nullptr;
    char tmp[1024];
    char line_buffer[1024];
    bool error= true;
    for(;;)
    {
        // charge une ligne du fichier
        if(!fgets(line_buffer, sizeof(line_buffer), in))
        {
            error= false;       // fin du fichier, pas d'erreur detectee
            break;
        }
        
        // force la fin de la ligne, au cas ou
        line_buffer[sizeof(line_buffer) -1]= 0;
        
        // saute les espaces en debut de ligne
        char *line= line_buffer;
        while(*line && isspace(*line))
            line++;
        
        if(line[0] == 'n')
        {
            if(sscanf(line, "newmtl %[^\r\n]", tmp) == 1)
            {
                int id= materials.insert(Material(Black()), tmp);
                material= &materials.material(id);
            }
        }
        
        if(material == nullptr)
            continue;
        
        if(line[0] == 'K')
        {
            float r, g, b;
            if(sscanf(line, "Kd %f %f %f", &r, &g, &b) == 3)
                material->diffuse= Color(r, g, b);
            else if(sscanf(line, "Ks %f %f %f", &r, &g, &b) == 3)
                material->specular= Color(r, g, b);
            else if(sscanf(line, "Ke %f %f %f", &r, &g, &b) == 3)
                material->emission= Color(r, g, b);
        }
        
        else if(line[0] == 'N')
        {
            float n;
            if(sscanf(line, "Ns %f", &n) == 1)          // Ns, puissance / concentration du reflet, modele blinn phong
                material->ns= n;
            if(sscanf(line, "Ni %f", &n) == 1)          // Ni, indice de refraction / fresnel
                material->ni= n;
        }
        else if(line[0] == 'T')
        {
            float r, g, b;
            if(sscanf(line, "Tf %f %f %f", &r, &g, &b) == 3)          // Tf, couleur de l'objet transparent
                material->transmission= Color(r, g, b);
        }
        
        else if(line[0] == 'm')
        {
            if(sscanf(line, "map_Kd %[^\r\n]", tmp) == 1)
                material->diffuse_texture= materials.insert_texture( absolute_filename(pathname(filename), tmp).c_str() );
                
            else if(sscanf(line, "map_Ks %[^\r\n]", tmp) == 1)
                material->specular_texture= materials.insert_texture( absolute_filename(pathname(filename), tmp).c_str() );
                
            else if(sscanf(line, "map_Ns %[^\r\n]", tmp) == 1)
                material->ns_texture= materials.insert_texture( absolute_filename(pathname(filename), tmp).c_str() );
        }
    }
    
    fclose(in);
    
    if(error)
        printf("[error] parsing line :\n%s\n", line_buffer);
    
    return !error;
}

bool read_materials( const char *filename, Materials& materials, std::vector<int>& indices )
{
    indices.clear();
    
    FILE *in= fopen(filename, "rt");
    if(!in)
    {
        printf("[error] loading materials '%s'...\n", filename);
        return false;
    }
    
    printf("loading materials '%s'...\n", filename);
    
    std::vector<int> wp;
    int material_id= -1;
    
    char tmp[1024];
    char line_buffer[1024];
    bool error= true;
    for(;;)
    {
        // charge une ligne du fichier
        if(!fgets(line_buffer, sizeof(line_buffer), in))
        {
            error= false;       // fin du fichier, pas d'erreur detectee
            break;
        }
        
        // force la fin de la ligne, au cas ou
        line_buffer[sizeof(line_buffer) -1]= 0;
        
        // saute les espaces en debut de ligne
        char *line= line_buffer;
        while(*line && isspace(*line))
            line++;
        
        if(line[0] == 'f')         // triangle a b c
        {
            wp.clear();
            
            int next= 0;
            for(line= line +1; ; line= line + next)
            {
                wp.push_back(0); 
                
                // analyse les attributs du sommet : p/t/n ou p//n ou p/t ou p...
                next= 0;
                int wt= 0;
                int wn= 0;
                if(sscanf(line, " %d/%d/%d %n", &wp.back(), &wt, &wn, &next) == 3) 
                    continue;
                else if(sscanf(line, " %d/%d %n", &wp.back(), &wt, &next) == 2)
                    continue;
                else if(sscanf(line, " %d//%d %n", &wp.back(), &wn, &next) == 2)
                    continue;
                else if(sscanf(line, " %d %n", &wp.back(), &next) == 1)
                    continue;
                else if(next == 0)      // fin de ligne
                    break;
            }
            
            // force une matiere par defaut, si necessaire
            if(material_id == -1)
                material_id= materials.default_material_index();
            
            // triangule la face
            for(unsigned v= 2; v +1 < wp.size(); v++)
                // indice de la matiere de chaque triangle
                indices.push_back(material_id);
        }
        else if(line[0] == 'm')
        {
            if(sscanf(line, "mtllib %[^\r\n]", tmp) == 1)
            {
                std::string materials_filename;
                if(tmp[0] != '/' && tmp[1] != ':')   // windows c:\ pour les chemins complets...
                    materials_filename= normalize_filename(pathname(filename) + tmp);
                else
                    materials_filename= std::string(tmp);
                
                // charge les matieres
                if(!read_materials_mtl( materials_filename.c_str(), materials ))
                    break;
            }
        }
        
        else if(line[0] == 'u')
        {
           if(sscanf(line, "usemtl %[^\r\n]", tmp) == 1)
               material_id= materials.find(tmp);
        }
    }
    
    fclose(in);
    
    if(error)
        printf("[error] loading materials '%s'...\n%s\n\n", filename, line_buffer);

    return !error;
}


// representation de l'indexation complete d'un sommet .obj / wavefront
struct vertex
{
    int material;
    int position;
    int texcoord;
    int normal;
    
    vertex( ) : material(-1), position(-1), texcoord(-1), normal(-1) {}
    vertex( const int m, const int p, const int t, const int n ) : material(m), position(p), texcoord(t), normal(n) {}
    
    // comparaison lexicographique de 2 sommets / des indices de leurs attributs
    bool operator< ( const vertex& b ) const
    {
        if(material != b.material) return material < b.material;
        if(position != b.position) return position < b.position;
        if(texcoord != b.texcoord) return texcoord < b.texcoord;
        if(normal != b.normal) return normal < b.normal;
        return false;
    }
};


int MeshIOData::find_object( const char *name )
{
    for(unsigned i= 0; i < object_names.size(); i++)
        if(object_names[i] == name)
            return i;
    
    return -1;
}

std::vector<MeshIOGroup> MeshIOData::groups( const std::vector<int>& properties )
{
    std::vector<int> remap( properties.size() );
    for(unsigned i= 0; i < remap.size(); i++)
        remap[i]= i;
        
    std::stable_sort(remap.begin(), remap.end(), 
        [&]( const int a, const int b) 
        {
            return properties[a] < properties[b];
        });
    
    //~ for(unsigned i= 0; i < remap.size(); i++)
        //~ printf("%d ", properties[remap[i]]);
    //~ printf("\n");
    
    // re-organise l'index buffer
    std::vector<unsigned> tmp;
    tmp.reserve(indices.size());
    for(unsigned i= 0; i < remap.size(); i++)
    {
        int id= remap[i];
        tmp.push_back( indices[3*id] );
        tmp.push_back( indices[3*id+1] );
        tmp.push_back( indices[3*id+2] );
    }
    std::swap(indices, tmp);
    
    std::vector<MeshIOGroup> groups;
    // first group
    int id= properties[remap[0]];
    unsigned first= 0;
    unsigned count= 1;
    for(unsigned i= 1; i < remap.size(); i++)
    {
        if(properties[remap[i]] != id)
        {
            groups.push_back({ id, first, count });
            // restart
            id= properties[remap[i]];
            first= i;
            count= 0;
        }
        
        count++;
    }
    
    // last group
    groups.push_back({ id, first, count });
    return groups;
}

bool read_meshio_data( const char *filename, MeshIOData& data )
{
    FILE *in= fopen(filename, "rt");
    if(!in)
    {
        printf("[error] loading indexed mesh '%s'...\n", filename);
        return false;
    }
    
    printf("loading indexed mesh '%s'...\n", filename);
    
    std::vector<Point> wpositions;
    std::vector<Point> wtexcoords;
    std::vector<Vector> wnormals;
    
    std::vector<int> wp;
    std::vector<int> wt;
    std::vector<int> wn;
    
    std::map<vertex, unsigned> remap;
    int material_id= -1;
    int object_id= -1;
    
    char tmp[1024];
    char line_buffer[1024];
    bool error= true;
    for(;;)
    {
        // charge une ligne du fichier
        if(!fgets(line_buffer, sizeof(line_buffer), in))
        {
            error= false;       // fin du fichier, pas d'erreur detectee
            break;
        }
        
        // force la fin de la ligne, au cas ou
        line_buffer[sizeof(line_buffer) -1]= 0;
        
        // saute les espaces en debut de ligne
        char *line= line_buffer;
        while(*line && isspace(*line))
            line++;
        
        if(line[0] == 'v')
        {
            float x, y, z;
            if(line[1] == ' ')          // position x y z
            {
                if(sscanf(line, "v %f %f %f", &x, &y, &z) != 3)
                    break;
                wpositions.push_back( Point(x, y, z) );
            }
            else if(line[1] == 'n')     // normal x y z
            {
                if(sscanf(line, "vn %f %f %f", &x, &y, &z) != 3)
                    break;
                wnormals.push_back( Vector(x, y, z) );
            }
            else if(line[1] == 't')     // texcoord x y
            {
                if(sscanf(line, "vt %f %f", &x, &y) != 2)
                    break;
                wtexcoords.push_back( Point(x, y, 0) );
            }
        }
        else if(line[0] == 'f')         // triangle a b c, les sommets sont numerotes a partir de 1 ou de la fin du tableau (< 0)
        {
            wp.clear();
            wt.clear();
            wn.clear();
            
            int next;
            for(line= line +1; ; line= line + next)
            {
                wp.push_back(0); 
                wt.push_back(0); 
                wn.push_back(0);         // 0: invalid index
                
                // analyse les attributs du sommet : p/t/n ou p//n ou p/t ou p...
                next= 0;
                if(sscanf(line, " %d/%d/%d %n", &wp.back(), &wt.back(), &wn.back(), &next) == 3) 
                    continue;
                else if(sscanf(line, " %d/%d %n", &wp.back(), &wt.back(), &next) == 2)
                    continue;
                else if(sscanf(line, " %d//%d %n", &wp.back(), &wn.back(), &next) == 2)
                    continue;
                else if(sscanf(line, " %d %n", &wp.back(), &next) == 1)
                    continue;
                else if(next == 0)      // fin de ligne
                    break;
            }
            
            // force une matiere par defaut, si necessaire
            if(material_id == -1)
                material_id= data.materials.default_material_index();
            
            if(object_id == -1)
            {
                object_id= data.find_object("default");
                if(object_id == -1)
                {
                    object_id= data.object_names.size();
                    data.object_names.push_back("default");
                }
            }
            
            // triangule la face
            for(unsigned v= 2; v +1 < wp.size(); v++)
            {
                data.material_indices.push_back(material_id);
                data.object_indices.push_back(object_id);
                
                unsigned idv[3]= { 0, v -1, v };
                for(unsigned i= 0; i < 3; i++)
                {
                    unsigned k= idv[i];
                    // indices des attributs du sommet
                    int p= (wp[k] < 0) ? int(wpositions.size()) + wp[k] : wp[k] -1;
                    int t= (wt[k] < 0) ? int(wtexcoords.size()) + wt[k] : wt[k] -1;
                    int n= (wn[k] < 0) ? int(wnormals.size())   + wn[k] : wn[k] -1;
                    
                    if(p < 0) break; // error
                    
                    // recherche / insere le sommet 
                    auto found= remap.insert( std::make_pair(vertex(material_id, p, t, n), unsigned(remap.size())) );
                    if(found.second)
                    {
                        // pas trouve, copie les nouveaux attributs
                        if(t != -1) data.texcoords.push_back(wtexcoords[t]);
                        if(n != -1) data.normals.push_back(wnormals[n]);
                        data.positions.push_back(wpositions[p]);
                    }
                    
                    // construit l'index buffer
                    assert(found.first->second < data.positions.size());
                    data.indices.push_back(found.first->second);
                }
            }
        }
        
        else if(line[0] == 'm')
        {
            if(sscanf(line, "mtllib %[^\r\n]", tmp) == 1)
            {
                std::string materials_filename;
                if(tmp[0] != '/' && tmp[1] != ':')   // windows c:\ pour les chemins complets...
                    materials_filename= normalize_filename(pathname(filename) + tmp);
                else
                    materials_filename= std::string(tmp);
                
                // charge les matieres, ou pas... 
                read_materials_mtl( materials_filename.c_str(), data.materials );
            }
        }
        
        else if(line[0] == 'u')
        {
           if(sscanf(line, "usemtl %[^\r\n]", tmp) == 1)
               material_id= data.materials.find(tmp);
        }
        
        else if(line[0] == 'o')
        {
            if(sscanf(line, "o %s", tmp) == 1)
            {
                object_id= data.find_object(tmp);
                if(object_id == -1)
                {
                    object_id= data.object_names.size();
                    data.object_names.push_back(tmp);
                }
                
                printf("object '%s': %d\n", tmp, object_id);
            }
        }
    #if 0
        else if(line[0] == 'g')
        {
            // ne lit que le 1er groupe 
            if(sscanf(line, "g %s", tmp) == 1)
            {
                object_id= data.find_object(tmp);
                if(object_id == -1)
                {
                    object_id= data.object_names.size();
                    data.object_names.push_back(tmp);
                }
                
                printf("object '%s': %d\n", tmp, object_id);
            }
        }
    #endif
    }
    
    fclose(in);
    
    if(error)
    {
        printf("[error] loading indexed mesh '%s'...\n%s\n\n", filename, line_buffer);
        return false;
    }

    printf("  %d indices, %d positions %d texcoords %d normals\n", 
        int(data.indices.size()), int(data.positions.size()), int(data.texcoords.size()), int(data.normals.size()));
    printf("  %d materials, %d textures\n", data.materials.count(), data.materials.filename_count());
    return true;
}


bool read_images( const Materials& materials, std::vector<Image>& images )
{
    int n= materials.filename_count();
    if(n == 0)  // pas de textures
        return true;
    
    images.resize(n);
    
#pragma omp parallel for
    for(int i= 0; i < n; i++)
    {
        //~ printf("loading '%s'...\n", materials.filename(i));
        images[i]= read_image(materials.filename(i));
    }
    
    return true;
}


bool read_images( MeshIOData& data )
{
    return read_images(data.materials, data.images);
}

