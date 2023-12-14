#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>
#include <algorithm>

using namespace std;

typedef float AngleRad;
typedef float AngleDeg;

#define Bool bool
#define FLOAT_EPS 1e-10

/**
 * Value used for floating point equality tests.
 */
#define EPSILON     1e-10

/**
 * Auxiliar numeric functions
 */
#define Max(A,B)        ((A)>(B)?(A):(B))
#define Min(A,B)        ((A)<(B)?(A):(B))
#define Sqr(A)          ((A)*(A))
#define Sign(A)         ((A)>(0)?(1):(-1))

/**
 * Useful functions to operate with angles in degrees
 */
AngleDeg ACos(float x);
AngleDeg Deg2Rad(AngleRad x);
AngleRad Rad2Deg(AngleDeg x);
float Cos(AngleDeg x);
float Sin(AngleDeg x);
float Tan(AngleDeg x);
AngleDeg ATan(float x);
AngleDeg ATan2(float x, float y);
bool isAngInInterval(AngleDeg ang, AngleDeg angMin, AngleDeg angMax);
AngleDeg getBisectorTwoAngles(AngleDeg angMin, AngleDeg angMax);
AngleDeg GetNormalizeAngleDeg(AngleDeg ang);
AngleRad GetNormalizeAngleRad(AngleRad ang);
double normalizeAngle(double angle);

/**
 * Coordination system: CARTESIAN/POLAR
 */
enum CoordSystemT {
    CARTESIAN, POLAR
};

/**
 * @class Vector
 *
 * @brief This class represents a position in the 2d space
 *
 * A position is represented by a x-axis coordinate and a
 * y-axis coordinate or in polar coordinates (r, phi)
 *
 * @author Hugo Picado (hugopicado@ua.pt)
 * @author Nuno Almeida (nuno.alm@ua.pt)
 */
class Vector {
public:
    Vector(float vx = 0, float vy = 0, CoordSystemT cs = CARTESIAN);

    // overloaded arithmetic operators
    Vector operator-() const;
    Vector operator+(const float &d) const;
    Vector operator+(const Vector &p) const;
    Vector operator-(const float &d) const;
    Vector operator-(const Vector &p) const;
    Vector operator*(const float &d) const;
    Vector operator*(const Vector &p) const;
    Vector operator/(const float &d) const;
    Vector operator/(const Vector &p) const;
    void operator=(const float &d);
    void operator+=(const Vector &p);
    void operator+=(const float &d);
    void operator-=(const Vector &p);
    void operator-=(const float &d);
    void operator*=(const Vector &p);
    void operator*=(const float &d);
    void operator/=(const Vector &p);
    void operator/=(const float &d);
    bool operator!=(const Vector &p);
    bool operator!=(const float &d);
    bool operator==(const Vector &p);
    bool operator==(const float &d);


    // set- and get methods for private member variables
    bool setX(float dX);
    float getX() const;
    bool setY(float dY);
    float getY() const;

    // set- and get methods for derived position information
    void setVector(float dX = 0, float dY = 0, CoordSystemT cs = CARTESIAN);
    float getDistanceTo(const Vector p);
    float crossProduct(const Vector p);

    Vector setLength(float d);
    float length() const;
    AngleDeg getDirection() const;
    float innerProduct(const Vector &p) const;

    // comparison methods for positions
    bool isInFrontOf(const Vector &p);
    bool isInFrontOf(const float &d);
    bool isBehindOf(const Vector &p);
    bool isBehindOf(const float &d);
    bool isLeftOf(const Vector &p);
    bool isLeftOf(const float &d);
    bool isRightOf(const Vector &p);
    bool isRightOf(const float &d);
    bool isBetweenX(const Vector &p1, const Vector &p2);
    bool isBetweenX(const float &d1, const float &d2);
    bool isBetweenY(const Vector &p1, const Vector &p2);
    bool isBetweenY(const float &d1, const float &d2);

    // conversion methods for positions
    Vector normalize();
    Vector rotate(AngleDeg angle);
    Vector globalToRelative(Vector orig, AngleDeg ang);
    Vector relativeToGlobal(Vector orig, AngleDeg ang);
    Vector getVectorOnLineFraction(Vector &p, float dFrac);

    // static class methods
    static Vector getVectorFromPolar(float dMag, AngleDeg ang);

public:
    float x;
    float y;
};

inline Vector Polar2Vector(float mod, AngleDeg ang) {
    return Vector(mod * Cos(ang), mod * Sin(ang));
}

/**
 * @class Geometry
 *
 * @brief This class contains several static methods dealing with geometry
 *
 * @author Hugo Picado (hugopicado@ua.pt)
 */
class Geometry {
public:
    // geometric series
    static float getLengthGeomSeries(float dFirst, float dRatio, float dSum);
    static float getSumGeomSeries(float dFirst, float dRatio, float dLen);
    static float getSumInfGeomSeries(float dFirst, float dRatio);
    static float getFirstGeomSeries(float dSum, float dRatio, float dLen);
    static float getFirstInfGeomSeries(float dSum, float dRatio);

    // abc formula
    static int abcFormula(float a, float b, float c, float *s1, float *s2);

    // 2D cross product. Return a positive value, if ABC makes a counter-clockwise
    // turn, negative for clockwise turn, and zero if the points are collinear.

    static float cross(Vector A, Vector B, Vector C) {
        Vector ab = B - A;
        Vector ac = C - A;
        return ab.x * ac.y - ab.y * ac.x;
    }
    //Compute the distance from segment AB to point C

    static float segmentPointDist(Vector endPointA, Vector endPointB, Vector point) {
        // check first if the point C is closer to one of the endpoints
        float dotp = (point - endPointB).innerProduct(endPointB - endPointA);
        if (dotp > 0)
            return endPointB.getDistanceTo(point);
        dotp = (point - endPointA).innerProduct(endPointA - endPointB);
        if (dotp > 0)
            return endPointA.getDistanceTo(point);

        return fabs(cross(endPointA, endPointB, point) / endPointB.getDistanceTo(endPointA));
    }
};

/**
 * @class Circle
 *
 * @brief This class represents a circle
 *
 * A circle is represented by a center position and a radius
 *
 * @author Hugo Picado (hugopicado@ua.pt)
 * @author Nuno Almeida (nuno.alm@ua.pt)
 */
class Circle {
public:
    Circle();
    Circle(Vector pos, float dR);


    // get and set methods
    bool setCircle(Vector pos, float dR);
    bool setRadius(float dR);
    float getRadius();
    bool setCenter(Vector pos);
    Vector getCenter();
    float getCircumference();
    float getArea();

    // calculate intersection points and area with other circle
    bool isInside(Vector pos);
    int getIntersectionPoints(Circle c, Vector *p1, Vector *p2);
    float getIntersectionArea(Circle c);

private:
    Vector m_posCenter;
    float m_dRadius;
};

class Line;
class Rectangle;

class Ray {
public:

    Ray() {
        origin = Vector(0, 0);
        direction = Vector(1, 0);
    }
    Ray(Vector orig, Vector dir);

    Ray(Vector orig, float ang) {
        origin = orig;
        direction = Polar2Vector(1.0, ang);
    }

    Bool OnRay(Vector pt);
    AngleDeg dir(void);
    Bool InRightDir(Vector pt); // more lenient than above about distance off ray
    Bool InRightSide(Vector pt); // UNTESTED

    Bool intersection(Ray r, Vector *pPt);
    Bool intersection(Line l, Vector *pPt);

    int CircleIntersect(float rad, Vector center, Vector* psol1, Vector* psol2);

    Vector GetClosestPoint(Vector pt);

    Vector RectangleIntersection(Rectangle R);

    float dist(Vector pt);

    Vector getOrigin() {
        return origin;
    }
    ;

    Vector getDir() {
        return direction;
    }
    ;

protected:
    friend class Line;
    friend class Rectangle;
    Vector origin;
    Vector direction;
};

/**
 * @class Line
 *
 * @brief This class represents a line.
 *
 * A line is defined by the equation ay + bx + c = 0
 *
 * @author Hugo Picado (hugopicado@ua.pt)
 * @author Nuno Almeida (nuno.alm@ua.pt)
 */
class Line {
public:
    Line(float a = 0, float b = 0, float c = 0);
    Line(Vector v1, Vector v2);
    Line(Vector v, AngleDeg dir);

    Line(Ray r) {
        LineFromRay(r);
    }


    inline Vector ProjectPoint(Vector pt) {
        return getIntersection(getPerpendicular(pt));
    } //LPR 2011

    // get intersection points with this line
    Vector getIntersection(Line line);
    int getCircleIntersectionPoints(Circle circle, Vector *posSolution1, Vector *posSolution2);
    bool RayIntersection(Ray r, Vector &pInt);

    void LineFromRay(Ray r);
    void LineFromTwoPoints(Vector pt1, Vector pt2);
    Line getTangentLine(Vector pos);
    Vector getPointOnLineClosestTo(Vector pos);
    float getDistanceWithPoint(Vector pos);
    bool isInBetween(Vector pos, Vector point1, Vector point2);
    int getSidePoint(Vector pos);

    Line getPerpendicular(Vector pos) const;

    void setLineFromTwoPoints(Vector pos1, Vector pos2);

    // calculate associated variables in the line
    float getYGivenX(float x);
    float getXGivenY(float y);
    float getACoefficient() const;
    float getBCoefficient() const;
    float getCCoefficient() const;

private:
    float m_a;
    float m_b;
    float m_c;
};

/**
 * @brief This class represents a rectangle
 *
 * A rectangle is defined by two vectors: the one at the upper left corner and
 * the one at the right bottom corner
 *
 * @author Hugo Picado (hugopicado@ua.pt)
 * @author Nuno Almeida (nuno.alm@ua.pt)
 */
class Rect {
public:
    Rect();
    Rect(Vector pos, Vector pos2);


    // checks whether point lies inside the rectangle
    bool isInside(Vector pos);

    // standard get and set methosd
    void setRectanglePoints(Vector pos1, Vector pos2);
    bool setPosLeftTop(Vector pos);
    Vector getPosLeftTop();
    bool setPosRightBottom(Vector pos);
    Vector getPosRightBottom();

private:
    Vector m_posLeftTop;
    Vector m_posRightBottom;
};

class Rectangle;

#endif // GEOMETRY_H
