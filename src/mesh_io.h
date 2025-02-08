
#ifndef _MESH_IO_H
#define _MESH_IO_H

#include <vector>

#include "vec.h"
#include "materials.h"
#include "image.h"
#include "files.h"

//! \addtogroup objet3D utilitaires pour manipuler des objets 3d
///@{

/*! charge les positions des sommets des triangles d'un objet. format .obj / wavefront. un triangle est represente par 3 positions successives.

exemple : charger un fichier .obj et parcourir tous les triangles
\code
    std::vector<Point> positions;
    if(!read_positions("data/robot.obj", positions))
        return "erreur";
    
    // parcours les triangles
    for(unsigned i= 0; i +2 < positions.size(); i+= 3)
    {
        // triangle abc
        Point a= positions[ i];
        Point b= positions[ i +1 ];
        Point c= positions[ i +2 ];
        
        // par exemple, calcule la normale du triangle
        Vector n= normalize( cross( Vector(a, b), Vector(a, c) ) );
        ...
    }
\endcode

on peut facilement recuperer les sommets du triangle numero id : 
\code
    std::vector<Point> positions;
    if(!read_positions("data/robot.obj", positions))
        return "erreur";
    
    int id= ... ;
    Point a= positions[ 3*id ];
    Point b= positions[ 3*id +1 ];
    Point c= positions[ 3*id +2 ];
\endcode
 */
bool read_positions( const char *filename, std::vector<Point>& positions );

/*! version indexee de read_positions(). un triangle est represente par 3 indices successifs dans indices[].

exemple : charger un fichier .obj et parcourir tous les triangles
\code
    std::vector<int> indices;
    std::vector<Point> positions;
    if(!read_indexed_positions("data/robot.obj", positions, indices))
        return "erreur";
    
    // parcours les triangles indexes
    for(unsigned i= 0; i +2 < indices.size(); i+= 3)
    {
        // triangle abc
        Point a= positions[ indices[i] ];
        Point b= positions[ indices[i+1] ];
        Point c= positions[ indices[i+2] ];
        
        // par exemple, calcule la normale du triangle
        Vector n= normalize( cross( Vector(a, b), Vector(a, c) ) );
        ...
    }
\endcode

on peut recuperer directement les sommets du triangle numero id : 
\code
    std::vector<int> indices;
    std::vector<Point> positions;
    if(!read_indexed_positions("data/robot.obj", positions, indices))
        return "erreur";
    
    int id= ... ;
    Point a= positions[ indices[ 3*id ] ];
    Point b= positions[ indices[ 3*id +1 ] ];
    Point c= positions[ indices[ 3*id +2 ] ];
\endcode
*/
bool read_indexed_positions( const char *filename, std::vector<Point>& positions, std::vector<unsigned>& indices );


/*! charge les matieres associees aux triangles d'un fichier .obj / wavefront. renvoie l'ensemble de matieres et l'indice de la matiere pour chaque triangle.

exemple :
\code
    #include "materials.h"
    
    Materials materials;
    std::vector<int> material_indices;   // indices des matieres
    if(!read_materials("data/robot.obj", materials, material_indices))  // !! oui c'est bien le fichier .obj !!
        return "erreur";
        
    // recuperer la matiere du triangle numero id :
    int material_id= material_indices[ id ];
    Material& material= materials( material_id );
\endcode

exemple plus complet, charge un objet, ses matieres et recupere la couleur de chaque triangle
\code
    #include "mesh_io.h"
    #include "materials.h"
    
    std::vector<Point> positions;
    if(!read_positions("data/robot.obj", positions))
        return "erreur";
    
    Materials materials;
    std::vector<int> material_indices;   // indices des matieres
    if(!read_materials(data/robot.obj", materials, material_indices))
        return "erreur";

    // recuperer les sommets du triangle numero id
    int id= ... ;
    Point a= positions[ 3*id ];
    Point b= positions[ 3*id +1 ];
    Point c= positions[ 3*id +2 ];
    
    // recuperer la matiere du triangle numero id
    int material_id= material_indices[ id ];
    Material& material= materials( material_id );
    
    // recuperer la couleur (de la matiere) du triangle
    Color color= material.diffuse;
\endcode
*/
bool read_materials( const char *filename, Materials& materials, std::vector<int>& indices );

/*! charge les images / textures referencees par les matieres d'un objet. 

exemple :
\code
    #include "mesh_io.h"
    #include "materials.h"
    
    std::vector<Point> positions;
    if(!read_positions("data/robot.obj", positions))
        return "erreur";
    
    Materials materials;
    std::vector<int> material_indices;   // indices des matieres
    if(!read_materials("data/robot.obj", materials, material_indices))  // !! oui c'est bien le fichier .obj !!
        return "erreur";
        
    std::vector<Image> images;
    if(!read_images(materials, images))
        return "erreur";
\endcode
*/
bool read_images( const Materials& materials, std::vector<Image>& images );


struct MeshIOGroup
{
    int id;
    unsigned first;
    unsigned count;
};

struct MeshIOData
{
    std::vector<Point> positions;
    std::vector<Point> texcoords;
    std::vector<Vector> normals;
    std::vector<unsigned> indices;
    std::vector<int> material_indices;
    
    Materials materials;
    std::vector<Image> images;
    
    int find_object( const char *name );
    std::vector<std::string> object_names;
    std::vector<int> object_indices;
  
    std::vector<MeshIOGroup> sort_by_material( ) { return groups(material_indices); }
    std::vector<MeshIOGroup> sort_by_object( ) { return groups(object_indices); }
    std::vector<MeshIOGroup> groups( const std::vector<int>& properties );
};

/*! charge tous les attributs et les matieres. en une seule fois.

exemple : 
\code
    MeshIOData data= read_meshio_data( "data/robot.obj" );
    if(data.positions.empty())
        return "erreur";
    
    // recuperer les sommets du triangle d'indice id :
    // triangle abc
    Point a= data.positions[ data.indices[3*id] ];
    Point b= data.positions[ data.indices[3*id+1] ];
    Point c= data.positions[ data.indices[3*id+2] ];
    
    // et recuperer sa matiere / couleur
    int material_id= data.material_indices[ id ];
    Material& material= data.materials( material_id );
    
    // recuperer la couleur (de la matiere) du triangle
    Color color= material.diffuse;    
    ...    
\endcode

utiliser read_meshio_data() est equivalent a :
\code
    const char *filename= "... .obj";
    
    std::vector<Point> positions;
    std::vector<int> indices;
    read_indexed_positions( filename, positions, indices );

    Materials materials;
    std::vector<int> material_indices;
    read_materials( filename, materials, materials_indices );

    std::vector<Image> images;
    read_images( filename, images );
\endcode

    mais toutes les infos sont chargees en seule fois, et sont stockees dans une seule structure, cf MeshIOData, plus simple a manipuler.
*/
bool read_meshio_data( const char *filename, MeshIOData& data  );

//! charge les images referencees par les matieres de l'objet. 
bool read_images( MeshIOData& data );

///@}

#endif
