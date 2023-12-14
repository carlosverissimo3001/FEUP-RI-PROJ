#ifndef VECTOR3F_H
#define VECTOR3F_H

#include <cmath>
#include "Geometry.h"
#include <math.h>
//! Describes a vector of three floats

/*!
 * \author Hugo Picado (hugopicado@ua.pt)
 * \author Nuno Almeida (nuno.alm@ua.pt)
 */
class Vector3f {
public:
    //! Default constructor
    Vector3f();

    float angle(Vector3f other) {
        float dp = innerProduct(other);
        return acos(dp / (length() * other.length()));
    }

    /*!
     * Constructor
     *
     * \param x x-axis coordinate
     * \param y y-axis coordinate
     * \param z z-axis coordinate
     */
    Vector3f(float x, float y, float z);

    //! Copy constructor
    Vector3f(const Vector3f& other);

    Vector3f(const Vector& other);

    //! Destructor
    ~Vector3f();

    //! getX
    float getX() const;
    void setX(float x);

    //! getY
    float getY() const;
    void setY(float y);

    //! getZ
    float getZ() const;
    void setZ(float z);

    //! Access X Y Z through indexes 0 1 2
    float operator[](const int) const;

    //! Sums this vector to another
    Vector3f operator+(const Vector3f& other) const;

    //! Subtracts another vector from this
    Vector3f operator-(const Vector3f& other) const;

    //! Negates the vector
    Vector3f operator-() const;

    //! Multiples this vector by another
    Vector3f operator*(const Vector3f& other) const;

    //! Divides this vector by another
    Vector3f operator/(const Vector3f& other) const;

    bool operator==(const Vector3f& other) const;

    /*!
     * Multiples this vector by a scalar
     *
     * \param factor Scalar number
     */
    Vector3f operator*(float factor) const;

    /*!
     * Divides this vector by a scalar
     *
     * \param factor Scalar number
     */
    Vector3f operator/(float factor) const;

    /*!
     * Add this vector to a scalar
     *
     * \param factor Scalar number
     */
    Vector3f operator+(float factor) const;

    /*!
     * Integer remainder this vector by a scalar
     *
     * \param factor Scalar number
     */
    Vector3f operator%(float factor) const;

    /**
     * Sums this vector to another assuming the value of the result
     * 
     * \param other Vector3f
     */
    Vector3f operator+=(const Vector3f& other);

    /**
     * Add this vector to a scalar assuming the value of the result
     * 
     * \param factor Scalar number
     */
    Vector3f operator+=(float factor);

    /**
     * Subtracts other vector from this vector assuming the value of the result
     * 
     * \param other Vector3f
     */
    Vector3f operator-=(const Vector3f& other);

    /**
     * Subtracts a scalar from this vector assuming the value of the result
     * 
     * \param factor Scalar number
     */
    Vector3f operator-=(float factor);

    /*!
     * Divides this vector by a scalar assuming the value of the result
     *
     * \param factor Scalar number
     */
    Vector3f operator/=(float factor);


    /*!
     * Sets the coordinates of the vector from separate floats
     *
     * \param x x-axis coordinate
     * \param y y-axis coordinate
     * \param z z-axis coordinate
     */
    void setCoordinates(float x, float y, float z);

    /*!
     * Sets the 2d coordinates of the vector from separate floats.
     * The z-axis coordinate remains the same
     */
    void set2dCoordinates(float x, float y);

    /*!
     * Sets the coordinates of the vector from another vector
     *
     * \param v Vector
     */
    void setCoordinates(const Vector3f& v);

    /*!
     * Computes the inner product of this vector with another
     *
     * \param other Vector to compute the inner product
     * \return Resultant vector
     */
    float innerProduct(const Vector3f& other) const;

    /*!
     * Computes the cross product of this vector with another
     *
     * \param other Vector to compute the cross product
     * \return Resultant vector
     */
    Vector3f crossProduct(const Vector3f& other) const;

    /*!
     * Gets the length of the vector
     *
     * \return Length of the vector
     */
    float length() const;

    /*!
     * Normalizes the vector to an arbitrary length (default is 1)
     *
     * \param len Length
     */
    Vector3f normalize(float len = 1) const;
    /*!
     * Normalizes the vector to an arbitrary range (default is 180)
     *
     * \param maxDegree -maxDegree< Vector Degrees <maxDegree
     */
    Vector3f normalizeToMaxDegree(int max = 180) const;
    /*!
     * Converts the vector coordinates from polar to cartesian
     * It is assumed that the vector has the angular coordinates in degrees
     */
    Vector3f toCartesian() const;

    //! Converts the vector coordinates from cartesian to polar
    Vector3f toPolar() const;

    //! Converts the 3d vector to a 2d vector
    Vector to2d() const;

    //! Gets the distance between this vector and another
    float dist(const Vector3f& other) const;

    //! Gets the distance between this vector a line that passes through other1 and other2
    float distToLine(const Vector3f& other1, const Vector3f &other2) const;


    /* !Determines the midpoint between 2 points in a 3D space
     *
     **/
    static Vector3f determineMidpoint(Vector3f a, Vector3f b);

public:
    float x; //!< x-axis coordinate
    float y; //!< y-axis coordinate
    float z; //!< z-axis coordinate
    //bool isInPolar; //!< TRUE if storing polar coordinates, FALSE if storing cartesian coordinates


};

#endif // VECTOR3F_H
