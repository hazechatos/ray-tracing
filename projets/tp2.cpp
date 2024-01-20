
#include "vec.h"
#include "color.h"
#include "image.h"
#include "image_io.h"


int main( )
{
    Image image(512, 512);
    
    for(int py= 0; py < image.height(); py++)
    for(int px= 0; px < image.width(); px++)
    {
        // origine du rayon
        Point o= Point(0, 0, 0);
        // extremite du rayon pour le pixel (px, py)
        float x= 0;
        float y= 0;
        float z= 0;
        Point e= Point(x, y, z);
        // direction du rayon
        Vector d= Vector(o, e);
        
        // couleur du pixel
        Color pixel;
        
        // a calculer ...
        
        image(px, py)= Color(pixel, 1); // force une couleur opaque dans l'image
    }
    
    write_image_png(image, "render.png");
    return 0;
}

    
