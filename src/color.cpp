
#include <cmath>
#include <algorithm>

#include "color.h"


float Color::power( ) const
{
    return (r+g+b) / 3;
}

float Color::max( ) const
{
    return std::max(r, std::max(g, std::max(b, float(0))));
}

Color Black( )
{
    return Color(0, 0, 0);
}

Color White( )
{
    return Color(1, 1, 1);
}

Color Red( )
{
    return Color(1, 0, 0);
}

Color Green( )
{
    return Color(0, 1, 0);
}

Color Blue( )
{
    return Color(0, 0, 1);
}

Color Yellow( )
{
    return Color(1, 1, 0);
}


float srgb( const float x )
{
    if(x < 0.00031308f) return 12.92f * x;
    else return std::pow(1.055f * x, 1.0f / 2.4f) - 0.055f;
}

float linear( const float x )
{
    if(x < 0.04045f) return x / 12.92f;
    else return std::pow((x + 0.055f) / 1.055f, 2.4f);
}

Color srgb( const Color& color, const float g )
{
    return Color(srgb(color.r), srgb(color.g), srgb(color.b), color.a);
    
}

Color linear( const Color& color, const float g )
{
    return Color(linear(color.r), linear(color.g), linear(color.b), color.a);
}


Color operator+ ( const Color& a, const Color& b )
{
    return Color(a.r + b.r, a.g + b.g, a.b + b.b, a.a + b.a);
}

Color operator- ( const Color& c )
{
    return Color(-c.r, -c.g, -c.b, -c.a);
}

Color operator- ( const Color& a, const Color& b )
{
    return a + (-b);
}

Color operator* ( const Color& a, const Color& b )
{
    return Color(a.r * b.r, a.g * b.g, a.b * b.b, a.a * b.a);
}

Color operator* ( const float k, const Color& c )
{
    return Color(c.r * k, c.g * k, c.b * k, c.a * k);
}

Color operator* ( const Color& c, const float k )
{
    return k * c;
}

Color operator/ ( const Color& a, const Color& b )
{
    return Color(a.r / b.r, a.g / b.g, a.b / b.b, a.a / b.a);
}

Color operator/ ( const float k, const Color& c )
{
    return Color(k / c.r, k / c.g, k / c.b, k / c.a);
}

Color operator/ ( const Color& c, const float k )
{
    float kk= 1 / k;
    return kk * c;
}

Color operator+ ( const float k, const Color& c )
{
    return Color(k + c.r, k + c.g, k + c.b, k + c.a);
}

Color operator+ ( const Color& c, const float k )
{
    return k + c;
}

Color abs( const Color& color )
{
    return Color( std::abs(color.r), std::abs(color.g), std::abs(color.b), std::abs(color.a) ); 
}
