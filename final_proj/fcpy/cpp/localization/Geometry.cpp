#include "Geometry.h"


/**
 * This function converts an angle in radians to the corresponding angle
 * in degrees.
 *
 * @param x an angle in radians
 * @return the corresponding angle in degrees
 */
AngleDeg Rad2Deg(AngleRad x) {
    return ( x * 180 / M_PI);
}

/**
 * This function converts an angle in degrees to the corresponding angle
 * in radians.
 *
 * @param x an angle in degrees
 * @return the corresponding angle in radians
 */
AngleRad Deg2Rad(AngleDeg x) {
    return ( x * M_PI / 180);
}

/**
 * This function returns the cosine of a given angle in degrees using the
 * built-in cosine function that works with angles in radians.
 *
 * @param x an angle in degrees
 * @return the cosine of the given angle
 */
float Cos(AngleDeg x) {
    return ( cos(Deg2Rad(x)));
}

/**
 * This function returns the sine of a given angle in degrees using the
 * built-in sine function that works with angles in radians.
 *
 * @param x an angle in degrees
 * @return the sine of the given angle
 */
float Sin(AngleDeg x) {
    return ( sin(Deg2Rad(x)));
}

/**
 * This function returns the tangent of a given angle in degrees using the
 * built-in tangent function that works with angles in radians.
 *
 * @param x an angle in degrees
 * @return the tangent of the given angle
 */
float Tan(AngleDeg x) {
    return ( tan(Deg2Rad(x)));
}

/**
 * This function returns the principal value of the arc tangent of x
 * in degrees using the built-in arc tangent function which returns
 * this value in radians.
 *
 * @param x a float value
 * @return the arc tangent of the given value in degrees
 */
AngleDeg ATan(float x) {
    return ( Rad2Deg(atan(x)));
}

/**
 * This function returns the principal value of the arc tangent of y/x in
 * degrees using the signs of both arguments to determine the quadrant of the
 * return value. For this the built-in 'atan2' function is used which returns
 * this value in radians.
 *
 * @param x a float value
 * @param y a float value
 * @return the arc tangent of y/x in degrees taking the signs of x and y into
 * account
 */
float ATan2(float x, float y) {
    if (fabs(x) < EPSILON && fabs(y) < EPSILON)
        return ( 0.0);

    return ( Rad2Deg(atan2(x, y)));
}

/**
 * This function returns the principal value of the arc cosine of x in degrees
 * using the built-in arc cosine function which returns this value in radians.
 *
 * @param x a float value
 * @return the arc cosine of the given value in degrees
 */
AngleDeg ACos(float x) {
    if (x >= 1)
        return ( 0.0);
    else if (x <= -1)
        return ( 180.0);

    return ( Rad2Deg(acos(x)));
}

/**
 * This function returns the principal value of the arc sine of x in degrees
 * using the built-in arc sine function which returns this value in radians.
 *
 * @param x a float value
 * @return the arc sine of the given value in degrees
 */
AngleDeg ASin(float x) {
    if (x >= 1)
        return ( 90.0);
    else if (x <= -1)
        return ( -90.0);

    return ( Rad2Deg(asin(x)));
}

/**
 * This function returns a boolean value which indicates whether the value
 * 'ang' (from interval [-180..180] lies in the interval [angMin..angMax].<br>
 * <br>
 * Examples: isAngInInterval( -100, 4, -150) returns false
 *           isAngInInterval(   45, 4, -150) returns true<br>
 *
 * @param ang angle that should be checked
 * @param angMin minimum angle in interval
 * @param angMax maximum angle in interval
 * @return boolean indicating whether ang lies in [angMin..angMax]
 */
bool isAngInInterval(AngleDeg ang, AngleDeg angMin, AngleDeg angMax) {
    // convert all angles to interval 0..360
    if ((ang + 360) < 360) ang += 360;
    if ((angMin + 360) < 360) angMin += 360;
    if ((angMax + 360) < 360) angMax += 360;

    if (angMin < angMax) // 0 ---false-- angMin ---true-----angMax---false--360
        return angMin < ang && ang < angMax;
    else // 0 ---true--- angMax ---false----angMin---true---360
        return !(angMax < ang && ang < angMin);
}

/**
 * This method returns the bisector (average) of two angles. It deals
 * with the boundary problem, thus when 'angMin' equals 170 and 'angMax'
 * equals -100, -145 is returned.
 *
 * @param angMin minimum angle [-180,180]
 * @param angMax maximum angle [-180,180]
 * @return average of angMin and angMax.
 */
AngleDeg getBisectorTwoAngles(AngleDeg angMin, AngleDeg angMax) {
    // separate sine and cosine part to circumvent boundary problem
    return GetNormalizeAngleDeg(ATan2((Sin(angMin) + Sin(angMax)) / 2.0,
            (Cos(angMin) + Cos(angMax)) / 2.0));
}

/**
 * This method normalizes an angle. This means that the resulting
 * angle lies between -180 and 180 degrees.
 *
 * @param angle the angle which must be normalized
 * @return the result of normalizing the given angle
 */
double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;

    return ( angle);
}

AngleDeg GetNormalizeAngleDeg(AngleDeg ang) {
    while (ang >= 180) ang -= 360;
    while (ang < -180) ang += 360;

    return ang;
}

/**
 * This method normalizes an angle in radians. This means that the resulting
 * angle lies between -pi and pi degrees.
 *
 * @param angle the angle which must be normalized
 * @return the result of normalizing the given angle
 */
AngleRad GetNormalizeAngleRad(AngleRad ang) {
    while (ang >= M_PI) ang -= 2 * M_PI;
    while (ang < -M_PI) ang += 2 * M_PI;

    return ang;
}



/************************************************************************/
/*******************   CLASS VECTOR   ***********************************/
/************************************************************************/

/*! Constructor for the Vector class. When the supplied
    Coordinate System type equals CARTESIAN, the arguments x and y
    denote the x- and y-coordinates of the new position. When it
    equals POLAR however, the arguments x and y denote the polar
    coordinates of the new position; in this case x is thus equal to
    the distance r from the origin and y is equal to the angle phi
    that the polar vector makes with the x-axis.
    \param x the x-coordinate of the new position when cs == CARTESIAN; the
    distance of the new position from the origin when cs = POLAR
    \param y the y-coordinate of the new position when cs = CARTESIAN; the
    angle that the polar vector makes with the x-axis when cs = POLAR
    \param cs a CoordSystemT indicating whether x and y denote cartesian
    coordinates or polar coordinates
    \return the Vector corresponding to the given arguments */
Vector::Vector(float x, float y, CoordSystemT cs) {
    setVector(x, y, cs);
}

/*! Overloaded version of unary minus operator for Vectors. It returns the
    negative Vector, i.e. both the x- and y-coordinates are multiplied by
    -1. The current Vector itself is left unchanged.
    \return a negated version of the current Vector */
Vector Vector::operator-() const{
    return ( Vector(-x, -y));
}

/*! Overloaded version of the binary plus operator for adding a given float
    value to a Vector. The float value is added to both the x- and
    y-coordinates of the current Vector. The current Vector itself is
    left unchanged.
    \param d a float value which has to be added to both the x- and
    y-coordinates of the current Vector
    \return the result of adding the given float value to the current
    Vector */
Vector Vector::operator+(const float &d) const{
    return ( Vector(x + d, y + d));
}

/*! Overloaded version of the binary plus operator for Vectors. It returns
    the sum of the current Vector and the given Vector by adding their
    x- and y-coordinates. The Vectors themselves are left unchanged.
    \param p a Vector
    \return the sum of the current Vector and the given Vector */
Vector Vector::operator+(const Vector &p) const{
    return ( Vector(x + p.x, y + p.y));
}

/*! Overloaded version of the binary minus operator for subtracting a
    given float value from a Vector. The float value is
    subtracted from both the x- and y-coordinates of the current
    Vector. The current Vector itself is left unchanged.
    \param d a float value which has to be subtracted from both the x- and
    y-coordinates of the current Vector
    \return the result of subtracting the given float value from the current
    Vector */
Vector Vector::operator-(const float &d) const{
    return ( Vector(x - d, y - d));
}

/*! Overloaded version of the binary minus operator for
    Vectors. It returns the difference between the current
    Vector and the given Vector by subtracting their x- and
    y-coordinates. The Vectors themselves are left unchanged.

    \param p a Vector
    \return the difference between the current Vector and the given
    Vector */
Vector Vector::operator-(const Vector &p) const {
    return ( Vector(x - p.x, y - p.y));
}

/*! Overloaded version of the multiplication operator for multiplying a
    Vector by a given float value. Both the x- and y-coordinates of the
    current Vector are multiplied by this value. The current Vector
    itself is left unchanged.
    \param d the multiplication factor
    \return the result of multiplying the current Vector by the given
    float value */
Vector Vector::operator*(const float &d) const{
    return ( Vector(x * d, y * d));
}

/*! Overloaded version of the multiplication operator for
    Vectors. It returns the product of the current Vector
    and the given Vector by multiplying their x- and
    y-coordinates. The Vectors themselves are left unchanged.

    \param p a Vector
    \return the product of the current Vector and the given Vector */
Vector Vector::operator*(const Vector &p) const{
    return ( Vector(x * p.x, y * p.y));
}

/*! Overloaded version of the division operator for dividing a
    Vector by a given float value. Both the x- and y-coordinates
    of the current Vector are divided by this value. The current
    Vector itself is left unchanged.

    \param d the division factor
    \return the result of dividing the current Vector by the given float
    value */
Vector Vector::operator/(const float &d) const{
    return ( Vector(x / d, y / d));
}

/*! Overloaded version of the division operator for Vectors. It
    returns the quotient of the current Vector and the given
    Vector by dividing their x- and y-coordinates. The
    Vectors themselves are left unchanged.

    \param p a Vector
    \return the quotient of the current Vector and the given one */
Vector Vector::operator/(const Vector &p) const{
    return ( Vector(x / p.x, y / p.y));
}

/*! Overloaded version of the assignment operator for assigning a given float
    value to both the x- and y-coordinates of the current Vector. This
    changes the current Vector itself.
    \param d a float value which has to be assigned to both the x- and
    y-coordinates of the current Vector */
void Vector::operator=(const float &d) {
    x = d;
    y = d;
}

/*! Overloaded version of the sum-assignment operator for Vectors. It
    returns the sum of the current Vector and the given Vector by
    adding their x- and y-coordinates. This changes the current Vector
    itself.
    \param p a Vector which has to be added to the current Vector */
void Vector::operator+=(const Vector &p) {
    x += p.x;
    y += p.y;
}

/*! Overloaded version of the sum-assignment operator for adding a given float
    value to a Vector. The float value is added to both the x- and
    y-coordinates of the current Vector. This changes the current
    Vector itself.
    \param d a float value which has to be added to both the x- and
    y-coordinates of the current Vector */
void Vector::operator+=(const float &d) {
    x += d;
    y += d;
}

/*! Overloaded version of the difference-assignment operator for
    Vectors.  It returns the difference between the current
    Vector and the given Vector by subtracting their x- and
    y-coordinates. This changes the current Vector itself.

    \param p a Vector which has to be subtracted from the current
    Vector */
void Vector::operator-=(const Vector &p) {
    x -= p.x;
    y -= p.y;
}

/*! Overloaded version of the difference-assignment operator for
    subtracting a given float value from a Vector. The float
    value is subtracted from both the x- and y-coordinates of the
    current Vector. This changes the current Vector itself.

    \param d a float value which has to be subtracted from both the x- and
    y-coordinates of the current Vector */
void Vector::operator-=(const float &d) {
    x -= d;
    y -= d;
}

/*! Overloaded version of the multiplication-assignment operator for
    Vectors. It returns the product of the current Vector
    and the given Vector by multiplying their x- and
    y-coordinates. This changes the current Vector itself.

    \param p a Vector by which the current Vector has to be
    multiplied */
void Vector::operator*=(const Vector &p) {
    x *= p.x;
    y *= p.y;
}

/*! Overloaded version of the multiplication-assignment operator for
    multiplying a Vector by a given float value. Both the x- and
    y-coordinates of the current Vector are multiplied by this
    value. This changes the current Vector itself.

    \param d a float value by which both the x- and y-coordinates of the
    current Vector have to be multiplied */
void Vector::operator*=(const float &d) {
    x *= d;
    y *= d;
}

/*! Overloaded version of the division-assignment operator for
    Vectors. It returns the quotient of the current Vector
    and the given Vector by dividing their x- and
    y-coordinates. This changes the current Vector itself.

    \param p a Vector by which the current Vector is divided */
void Vector::operator/=(const Vector &p) {
    x /= p.x;
    y /= p.y;
}

/*! Overloaded version of the division-assignment operator for
    dividing a Vector by a given float value. Both the x- and
    y-coordinates of the current Vector are divided by this
    value. This changes the current Vector itself.

    \param d a float value by which both the x- and y-coordinates of the
    current Vector have to be divided */
void Vector::operator/=(const float &d) {
    x /= d;
    y /= d;
}

/*! Overloaded version of the inequality operator for Vectors. It
    determines whether the current Vector is unequal to the given
    Vector by comparing their x- and y-coordinates.

    \param p a Vector
    \return true when either the x- or y-coordinates of the given Vector
    and the current Vector are different; false otherwise */
bool Vector::operator!=(const Vector &p) {
    return ( (x != p.x) || (y != p.y));
}

/*! Overloaded version of the inequality operator for comparing a
    Vector to a float value. It determines whether either the x-
    or y-coordinate of the current Vector is unequal to the given
    float value.

    \param d a float value with which both the x- and y-coordinates of the
    current Vector have to be compared.
    \return true when either the x- or y-coordinate of the current Vector
    is unequal to the given float value; false otherwise */
bool Vector::operator!=(const float &d) {
    return ( (x != d) || (y != d));
}

/*! Overloaded version of the equality operator for Vectors. It
    determines whether the current Vector is equal to the given
    Vector by comparing their x- and y-coordinates.

    \param p a Vector
    \return true when both the x- and y-coordinates of the given
    Vector and the current Vector are equal; false
    otherwise */
bool Vector::operator==(const Vector &p) {
    return ( (x == p.x) && (y == p.y));
}

/*! Overloaded version of the equality operator for comparing a
    Vector to a float value. It determines whether both the x-
    and y-coordinates of the current Vector are equal to the
    given float value.

    \param d a float value with which both the x- and y-coordinates of the
    current Vector have to be compared.
    \return true when both the x- and y-coordinates of the current Vector
    are equal to the given float value; false otherwise */
bool Vector::operator==(const float &d) {
    return ( (x == d) && (y == d));
}



/*! Set method for the x-coordinate of the current Vector.

    \param dX a float value representing a new x-coordinate
    \return a boolean indicating whether the update was successful */
bool Vector::setX(float dX) {
    x = dX;
    return ( true);
}

/*! Get method for the x-coordinate of the current Vector.

    \return the x-coordinate of the current Vector */
float Vector::getX() const {
    return ( x);
}

/*! Set method for the y-coordinate of the current Vector.

    \param dY a float value representing a new y-coordinate
    \return a boolean indicating whether the update was successful */
bool Vector::setY(float dY) {
    y = dY;
    return ( true);
}

/*! Get method for the y-coordinate of the current Vector.

    \return the y-coordinate of the current Vector */
float Vector::getY() const {
    return ( y);
}

/*! This method (re)sets the coordinates of the current
    Vector. The given coordinates can either be polar or
    Cartesian coordinates. This is indicated by the value of the third
    argument.

    \param dX a float value indicating either a new Cartesian
    x-coordinate when cs=CARTESIAN or a new polar r-coordinate
    (distance) when cs=POLAR

    \param dY a float value indicating either a new Cartesian
    y-coordinate when cs=CARTESIAN or a new polar phi-coordinate
    (angle) when cs=POLAR

    \param cs a CoordSystemT indicating whether x and y denote
    cartesian coordinates or polar coordinates */
void Vector::setVector(float dX, float dY, CoordSystemT cs) {
    if (cs == CARTESIAN) {
        x = dX;
        y = dY;
    } else
        *this = getVectorFromPolar(dX, dY);
}

/*! This method determines the distance between the current
    Vector and a given Vector. This is equal to the
    magnitude (length) of the vector connecting the two positions
    which is the difference vector between them.

    \param p a Vecposition
    \return the distance between the current Vector and the given
    Vector */
float Vector::getDistanceTo(const Vector p) {
    return ( (*this -p).length());
}

/*! This method adjusts the coordinates of the current Vector in
    such a way that the magnitude of the corresponding vector equals
    the float value which is supplied as an argument. It thus scales
    the vector to a given length by multiplying both the x- and
    y-coordinates by the quotient of the argument and the current
    magnitude. This changes the Vector itself.

    \param d a float value representing a new magnitude

    \return the result of scaling the vector corresponding with the
    current Vector to the given magnitude thus yielding a
    different Vector */
Vector Vector::setLength(float d) {
    if (length() > EPSILON)
        (*this) *= (d / length());

    return ( *this);
}

/*! This method determines the magnitude (length) of the vector
    corresponding with the current Vector using the formula of
    Pythagoras.

    \return the length of the vector corresponding with the current
    Vector */
float Vector::length() const {
    return ( sqrt(x * x + y * y));
}

float Vector::crossProduct(const Vector p) {
    return this->x*p.y - this->y*p.x;
}

/*! This method determines the direction of the vector corresponding
    with the current Vector (the phi-coordinate in polar
    representation) using the arc tangent function. Note that the
    signs of x and y have to be taken into account in order to
    determine the correct quadrant.

    \return the direction in degrees of the vector corresponding with
    the current Vector */
AngleDeg Vector::getDirection() const {
    return ( ATan2(y, x));
}

/*! This method determines whether the current Vector is in front
    of a given Vector, i.e. whether the x-coordinate of the
    current Vector is larger than the x-coordinate of the given
    Vector.

    \param p a Vector to which the current Vector must be compared
    \return true when the current Vector is in front of the given
    Vector; false otherwise */
bool Vector::isInFrontOf(const Vector &p) {
    return ( (x > p.getX()) ? true : false);
}

/*! This method determines whether the x-coordinate of the current
    Vector is in front of (i.e. larger than) a given float
    value.

    \param d a float value to which the current x-coordinate must be
    compared

    \return true when the current x-coordinate is in front of the
    given value; false otherwise */
bool Vector::isInFrontOf(const float &d) {
    return ( (x > d) ? true : false);
}

/*! This method determines whether the current Vector is behind a
    given Vector, i.e. whether the x-coordinate of the current
    Vector is smaller than the x-coordinate of the given
    Vector.

    \param p a Vector to which the current Vector must be
    compared

    \return true when the current Vector is behind the given
    Vector; false otherwise */
bool Vector::isBehindOf(const Vector &p) {
    return ( (x < p.getX()) ? true : false);
}

/*! This method determines whether the x-coordinate of the current
    Vector is behind (i.e. smaller than) a given float value.

    \param d a float value to which the current x-coordinate must be
    compared

    \return true when the current x-coordinate is behind the given
    value; false otherwise */
bool Vector::isBehindOf(const float &d) {
    return ( (x < d) ? true : false);
}

/*! This method determines whether the current Vector is to the
    left of a given Vector, i.e. whether the y-coordinate of the
    current Vector is smaller than the y-coordinate of the given
    Vector.

    \param p a Vector to which the current Vector must be
    compared

    \return true when the current Vector is to the left of the
    given Vector; false otherwise */
bool Vector::isLeftOf(const Vector &p) {
    return ( (y < p.getY()) ? true : false);
}

/*! This method determines whether the y-coordinate of the current
    Vector is to the left of (i.e. smaller than) a given float
    value.

    \param d a float value to which the current y-coordinate must be
    compared

    \return true when the current y-coordinate is to the left of the
    given value; false otherwise */
bool Vector::isLeftOf(const float &d) {
    return ( (y < d) ? true : false);
}

/*! This method determines whether the current Vector is to the
    right of a given Vector, i.e. whether the y-coordinate of the
    current Vector is larger than the y-coordinate of the given
    Vector.

    \param p a Vector to which the current Vector must be
    compared

    \return true when the current Vector is to the right of the
    given Vector; false otherwise */
bool Vector::isRightOf(const Vector &p) {
    return ( (y > p.getY()) ? true : false);
}

/*! This method determines whether the y-coordinate of the current
    Vector is to the right of (i.e. larger than) a given float
    value.

    \param d a float value to which the current y-coordinate must be
    compared

    \return true when the current y-coordinate is to the right of the
    given value; false otherwise */
bool Vector::isRightOf(const float &d) {
    return ( (y > d) ? true : false);
}

/*! This method determines whether the current Vector is in
    between two given Vectors when looking in the x-direction,
    i.e. whether the current Vector is in front of the first
    argument and behind the second.

    \param p1 a Vector to which the current Vector must be
    compared

    \param p2 a Vector to which the current Vector must be
    compared

    \return true when the current Vector is in between the two
    given Vectors when looking in the x-direction; false
    otherwise */
bool Vector::isBetweenX(const Vector &p1, const Vector &p2) {
    return ( (isInFrontOf(p1) && isBehindOf(p2)) ? true : false);
}

/*! This method determines whether the x-coordinate of the current
    Vector is in between two given float values, i.e. whether
    the x-coordinate of the current Vector is in front of the
    first argument and behind the second.

    \param d1 a float value to which the current x-coordinate must be
    compared

    \param d2 a float value to which the current x-coordinate must be
    compared

    \return true when the current x-coordinate is in between the two
    given values; false otherwise */
bool Vector::isBetweenX(const float &d1, const float &d2) {
    return ( (isInFrontOf(d1) && isBehindOf(d2)) ? true : false);
}

/*! This method determines whether the current Vector is in
    between two given Vectors when looking in the y-direction,
    i.e. whether the current Vector is to the right of the first
    argument and to the left of the second.

    \param p1 a Vector to which the current Vector must be
    compared

    \param p2 a Vector to which the current Vector must be
    compared

    \return true when the current Vector is in between the two
    given Vectors when looking in the y-direction; false
    otherwise */
bool Vector::isBetweenY(const Vector &p1, const Vector &p2) {
    return ( (isRightOf(p1) && isLeftOf(p2)) ? true : false);
}

/*! This method determines whether the y-coordinate of the current
    Vector is in between two given float values, i.e. whether
    the y-coordinate of the current Vector is to the right of the
    first argument and to the left of the second.

    \param d1 a float value to which the current y-coordinate must be
    compared

    \param d2 a float value to which the current y-coordinate must be
    compared

    \return true when the current y-coordinate is in between the two
    given values; false otherwise */
bool Vector::isBetweenY(const float &d1, const float &d2) {
    return ( (isRightOf(d1) && isLeftOf(d2)) ? true : false);
}

/*! This method normalizes a Vector by setting the magnitude of
    the corresponding vector to 1. This thus changes the Vector
    itself.

    \return the result of normalizing the current Vector thus
    yielding a different Vector */
Vector Vector::normalize() {
    return ( setLength(1.0));
}

/*! This method rotates the vector corresponding to the current
    Vector over a given angle thereby changing the current
    Vector itself. This is done by calculating the polar
    coordinates of the current Vector and adding the given angle
    to the phi-coordinate in the polar representation. The polar
    coordinates are then converted back to Cartesian coordinates to
    obtain the desired result.

    \param angle an angle in degrees over which the vector
    corresponding to the current Vector must be rotated

    \return the result of rotating the vector corresponding to the
    current Vector over the given angle thus yielding a different
    Vector */
Vector Vector::rotate(AngleDeg angle) {
    // determine the polar representation of the current Vector
    float dMag = this->length();
    float dNewDir = this->getDirection() + angle; // add rotation angle to phi
    setVector(dMag, dNewDir, POLAR); // convert back to Cartesian
    return ( *this);
}

/*! This method converts the coordinates of the current Vector
    (which are represented in an global coordinate system with the
    origin at (0,0)) into relative coordinates in a different
    coordinate system (e.g. relative to a player). The new coordinate
    system is defined by the arguments to the method. The relative
    coordinates are now obtained by aligning the relative coordinate
    system with the global coordinate system using a translation to
    make both origins coincide followed by a rotation to align the
    axes.

    \param origin the origin of the relative coordinate frame

    \param ang the angle between the world frame and the relative
    frame (reasoning from the world frame)

    \return the result of converting the current global Vector
    into a relative Vector */
Vector Vector::globalToRelative(Vector origin, AngleDeg ang) {
    // convert global coordinates into relative coordinates by aligning
    // relative frame and world frame. First perform translation to make
    // origins of both frames coincide. Then perform rotation to make
    // axes of both frames coincide (use negative angle since you rotate
    // relative frame to world frame).
    *this -= origin;
    return ( rotate(-ang));
}

/*! This method converts the coordinates of the current Vector
    (which are represented in a relative coordinate system) into
    global coordinates in the world frame (with origin at (0,0)). The
    relative coordinate system is defined by the arguments to the
    method. The global coordinates are now obtained by aligning the
    world frame with the relative frame using a rotation to align the
    axes followed by a translation to make both origins coincide.

    \param origin the origin of the relative coordinate frame

    \param ang the angle between the world frame and the relative
    frame (reasoning from the world frame)

    \return the result of converting the current relative Vector
    into an global Vector */
Vector Vector::relativeToGlobal(Vector origin, AngleDeg ang) {
    // convert relative coordinates into global coordinates by aligning
    // world frame and relative frame. First perform rotation to make
    // axes of both frames coincide (use positive angle since you rotate
    // world frame to relative frame). Then perform translation to make
    // origins of both frames coincide.
    rotate(ang);
    *this += origin;
    return ( *this);
}

/*! This method returns a Vector that lies somewhere on the
    vector between the current Vector and a given
    Vector. The desired position is specified by a given fraction
    of this vector (e.g. 0.5 means exactly in the middle of the
    vector). The current Vector itself is left unchanged.

    \param p a Vector which defines the vector to the current
    Vector

    \param dFrac float representing the fraction of the connecting
    vector at which the desired Vector lies.

    \return the Vector which lies at fraction dFrac on the vector
    connecting p and the current Vector */
Vector Vector::getVectorOnLineFraction(Vector &p, float dFrac) {
    // determine point on line that lies at fraction dFrac of whole line
    // example: this --- 0.25 ---------  p
    // formula: this + dFrac * ( p - this ) = this - dFrac * this + dFrac * p =
    //          ( 1 - dFrac ) * this + dFrac * p
    return ( (*this) * (1.0 - dFrac) + (p * dFrac));
}

/*! This method converts a polar representation of a Vector into
    a Cartesian representation.

    \param dMag a float representing the polar r-coordinate, i.e. the
    distance from the point to the origin

    \param ang the angle that the polar vector makes with the x-axis,
    i.e. the polar phi-coordinate

    \return the result of converting the given polar representation
    into a Cartesian representation thus yielding a Cartesian
    Vector */
Vector Vector::getVectorFromPolar(float dMag, AngleDeg ang) {
    // cos(phi) = x/r <=> x = r*cos(phi); sin(phi) = y/r <=> y = r*sin(phi)
    return ( Vector(dMag * Cos(ang), dMag * Sin(ang)));
}

/**
 * This methods returns the inner product of this vector with another
 *
 * @param other the other vector
 * @return inner product
 */
float Vector::innerProduct(const Vector& other) const {
    return x * other.x + y * other.y;
}


/*****************************************************************************/
/********************** CLASS GEOMETRY ***************************************/
/*****************************************************************************/

/*! A geometric series is one in which there is a constant ratio between each
    element and the one preceding it. This method determines the
    length of a geometric series given its first element, the sum of the
    elements in the series and the constant ratio between the elements.
    Normally: s = a + ar + ar^2 + ...  + ar^n
    Now: dSum = dFirst + dFirst*dRatio + dFirst*dRatio^2 + .. + dFist*dRatio^n
    \param dFirst first term of the series
    \param dRatio ratio with which the the first term is multiplied
    \param dSum the total sum of all the serie
    \return the length(n in above example) of the series */
float Geometry::getLengthGeomSeries(float dFirst, float dRatio, float dSum) {

    // s = a + ar + ar^2 + .. + ar^n-1 and thus sr = ar + ar^2 + .. + ar^n
    // subtract: sr - s = - a + ar^n) =>  s(1-r)/a + 1 = r^n = temp
    // log r^n / n = n log r / log r = n = length
    float temp = (dSum * (dRatio - 1) / dFirst) + 1;
    if (temp <= 0)
        return -1.0;
    return log(temp) / log(dRatio);
}

/*! A geometric series is one in which there is a constant ratio between each
    element and the one preceding it. This method determines the sum of a
    geometric series given its first element, the ratio and the number of steps
    in the series
    Normally: s = a + ar + ar^2 + ...  + ar^n
    Now: dSum = dFirst + dFirst*dRatio + ... + dFirst*dRatio^dSteps
    \param dFirst first term of the series
    \param dRatio ratio with which the the first term is multiplied
    \param dSum the number of steps to be taken into account
    \return the sum of the series */
float Geometry::getSumGeomSeries(float dFirst, float dRatio, float dLength) {
    // s = a + ar + ar^2 + .. + ar^n-1 and thus sr = ar + ar^2 + .. + ar^n
    // subtract: s - sr = a - ar^n) =>  s = a(1-r^n)/(1-r)
    return dFirst * (1 - pow(dRatio, dLength)) / (1 - dRatio);
}

/*! A geometric series is one in which there is a constant ratio between each
    element and the one preceding it. This method determines the sum of an
    infinite geometric series given its first element and the constant ratio
    between the elements. Note that such an infinite series will only converge
    when 0<r<1.
    Normally: s = a + ar + ar^2 + ar^3 + ....
    Now: dSum = dFirst + dFirst*dRatio + dFirst*dRatio^2...
    \param dFirst first term of the series
    \param dRatio ratio with which the the first term is multiplied
    \return the sum of the series */
float Geometry::getSumInfGeomSeries(float dFirst, float dRatio) {

    // s = a(1-r^n)/(1-r) with n->inf and 0<r<1 => r^n = 0
    return dFirst / (1 - dRatio);
}

/*! A geometric series is one in which there is a constant ratio between each
    element and the one preceding it. This method determines the first element
    of a geometric series given its element, the ratio and the number of steps
    in the series
    Normally: s = a + ar + ar^2 + ...  + ar^n
    Now: dSum = dFirst + dFirst*dRatio + ... + dFirst*dRatio^dSteps
    \param dSum sum of the series
    \param dRatio ratio with which the the first term is multiplied
    \param dSum the number of steps to be taken into account
    \return the first element (a) of a serie */
float Geometry::getFirstGeomSeries(float dSum, float dRatio, float dLength) {
    // s = a + ar + ar^2 + .. + ar^n-1 and thus sr = ar + ar^2 + .. + ar^n
    // subtract: s - sr = a - ar^n) =>  s = a(1-r^n)/(1-r) => a = s*(1-r)/(1-r^n)
    return dSum * (1 - dRatio) / (1 - pow(dRatio, dLength));
}

/*! A geometric series is one in which there is a constant ratio between each
    element and the one preceding it. This method determines the first element
    of an infinite geometric series given its first element and the constant
    ratio between the elements. Note that such an infinite series will only
    converge when 0<r<1.
    Normally: s = a + ar + ar^2 + ar^3 + ....
    Now: dSum = dFirst + dFirst*dRatio + dFirst*dRatio^2...
    \param dSum sum of the series
    \param dRatio ratio with which the the first term is multiplied
    \return the first term of the series */
float Geometry::getFirstInfGeomSeries(float dSum, float dRatio) {

    // s = a(1-r^n)/(1-r) with r->inf and 0<r<1 => r^n = 0 => a = s ( 1 - r)
    return dSum * (1 - dRatio);
}

/*! This method performs the abc formula (Pythagoras' Theorem) on the given
    parameters and puts the result in *s1 en *s2. It returns the number of
    found coordinates.
    \param a a parameter in abc formula
    \param b b parameter in abc formula
    \param c c parameter in abc formula
    \param *s1 first result of abc formula
    \param *s2 second result of abc formula
    \return number of found x-coordinates */
int Geometry::abcFormula(float a, float b, float c, float *s1, float *s2) {
    float dDiscr = b * b - 4 * a*c; // discriminant is b^2 - 4*a*c
    if (fabs(dDiscr) < EPSILON) // if discriminant = 0
    {
        *s1 = -b / (2 * a); //  only one solution
        return 1;
    } else if (dDiscr < 0) // if discriminant < 0
        return 0; //  no solutions
    else // if discriminant > 0
    {
        dDiscr = sqrt(dDiscr); //  two solutions
        *s1 = (-b + dDiscr) / (2 * a);
        *s2 = (-b - dDiscr) / (2 * a);
        return 2;
    }
}

/*****************************************************************************/
/********************* CLASS CIRCLE ******************************************/
/*****************************************************************************/

/*! This is the constructor of a circle.
    \param pos first point that defines the center of circle
    \param dR the radius of the circle
    \return circle with pos as center and radius as radius*/
Circle::Circle(Vector pos, float dR) {
    setCircle(pos, dR);
}

/*! This is the constructor of a circle which initializes a circle with a
    radius of zero. */
Circle::Circle() {
    setCircle(Vector(-1000.0, -1000.0), 0);
}


/*! This method sets the values of the circle.
    \param pos new center of the circle
    \param dR new radius of the circle
    ( > 0 )
    \return bool indicating whether radius was set */
bool Circle::setCircle(Vector pos, float dR) {
    setCenter(pos);
    return setRadius(dR);
}

/*! This method sets the radius of the circle.
    \param dR new radius of the circle ( > 0 )
    \return bool indicating whether radius was set */
bool Circle::setRadius(float dR) {
    if (dR > 0) {
        m_dRadius = dR;
        return true;
    } else {
        m_dRadius = 0.0;
        return false;
    }
}

/*! This method returns the radius of the circle.
    \return radius of the circle */
float Circle::getRadius() {
    return m_dRadius;
}

/*! This method sets the center of the circle.
    \param pos new center of the circle
    \return bool indicating whether center was set */
bool Circle::setCenter(Vector pos) {
    m_posCenter = pos;
    return true;
}

/*! This method returns the center of the circle.
    \return center of the circle */
Vector Circle::getCenter() {
    return m_posCenter;
}

/*! This method returns the circumference of the circle.
    \return circumference of the circle */
float Circle::getCircumference() {
    return 2.0 * M_PI * getRadius();
}

/*! This method returns the area inside the circle.
    \return area inside the circle */
float Circle::getArea() {
    return M_PI * getRadius() * getRadius();
}

/*! This method returns a boolean that indicates whether 'pos' is
    located inside the circle.
 
   \param pos position of which should be checked whether it is
   located in the circle

   \return bool indicating whether pos lies inside the circle */
bool Circle::isInside(Vector pos) {
    return m_posCenter.getDistanceTo(pos) < getRadius();
}

/*! This method returns the two possible intersection points between two
    circles. This method returns the number of solutions that were found.
    \param c circle with which intersection should be found
    \param p1 will be filled with first solution
    \param p2 will be filled with second solution
    \return number of solutions. */
int Circle::getIntersectionPoints(Circle c, Vector *p1, Vector *p2) {
    float x0, y0, r0;
    float x1, y1, r1;

    x0 = getCenter().getX();
    y0 = getCenter().getY();
    r0 = getRadius();
    x1 = c.getCenter().getX();
    y1 = c.getCenter().getY();
    r1 = c.getRadius();

    float d, dx, dy, h, a, x, y, p2_x, p2_y;

    // first calculate distance between two centers circles P0 and P1.
    dx = x1 - x0;
    dy = y1 - y0;
    d = sqrt(dx * dx + dy * dy);

    // normalize differences
    dx /= d;
    dy /= d;

    // a is distance between p0 and point that is the intersection point P2
    // that intersects P0-P1 and the line that crosses the two intersection
    // points P3 and P4.
    // Define two triangles: P0,P2,P3 and P1,P2,P3.
    // with distances a, h, r0 and b, h, r1 with d = a + b
    // We know a^2 + h^2 = r0^2 and b^2 + h^2 = r1^2 which then gives
    // a^2 + r1^2 - b^2 = r0^2 with d = a + b ==> a^2 + r1^2 - (d-a)^2 = r0^2
    // ==> r0^2 + d^2 - r1^2 / 2*d
    a = (r0 * r0 + d * d - r1 * r1) / (2.0 * d);

    // h is then a^2 + h^2 = r0^2 ==> h = sqrt( r0^2 - a^2 )
    float arg = r0 * r0 - a*a;
    h = (arg > 0.0) ? sqrt(arg) : 0.0;

    // First calculate P2
    p2_x = x0 + a * dx;
    p2_y = y0 + a * dy;

    // and finally the two intersection points
    x = p2_x - h * dy;
    y = p2_y + h * dx;
    p1->setVector(x, y);
    x = p2_x + h * dy;
    y = p2_y - h * dx;
    p2->setVector(x, y);

    return (arg < 0.0) ? 0 : ((arg == 0.0) ? 1 : 2);
}

/*! This method returns the size of the intersection area of two circles.
    \param c circle with which intersection should be determined
    \return size of the intersection area. */
float Circle::getIntersectionArea(Circle c) {
    Vector pos1, pos2, pos3;
    float d, h, dArea;
    AngleDeg ang;

    d = getCenter().getDistanceTo(c.getCenter()); // dist between two centers
    if (d > c.getRadius() + getRadius()) // larger than sum radii
        return 0.0; // circles do not intersect
    if (d <= fabs(c.getRadius() - getRadius())) // one totally in the other
    {
        float dR = min(c.getRadius(), getRadius()); // return area smallest circ
        return M_PI * dR*dR;
    }

    int iNrSol = getIntersectionPoints(c, &pos1, &pos2);
    if (iNrSol != 2)
        return 0.0;

    // the intersection area of two circles can be divided into two segments:
    // left and right of the line between the two intersection points p1 and p2.
    // The outside area of each segment can be calculated by taking the part
    // of the circle pie excluding the triangle from the center to the
    // two intersection points.
    // The pie equals pi*r^2 * rad(2*ang) / 2*pi = 0.5*rad(2*ang)*r^2 with ang
    // the angle between the center c of the circle and one of the two
    // intersection points. Thus the angle between c and p1 and c and p3 where
    // p3 is the point that lies halfway between p1 and p2.
    // This can be calculated using ang = asin( d / r ) with d the distance
    // between p1 and p3 and r the radius of the circle.
    // The area of the triangle is 2*0.5*h*d.

    pos3 = pos1.getVectorOnLineFraction(pos2, 0.5);
    d = pos1.getDistanceTo(pos3);
    h = pos3.getDistanceTo(getCenter());
    ang = asin(d / getRadius());

    dArea = ang * getRadius() * getRadius();
    dArea = dArea - d*h;

    // and now for the other segment the same story
    h = pos3.getDistanceTo(c.getCenter());
    ang = asin(d / c.getRadius());
    dArea = dArea + ang * c.getRadius() * c.getRadius();
    dArea = dArea - d*h;

    return dArea;
}


/*****************************************************************************/
/***********************  CLASS LINE *****************************************/
/*****************************************************************************/

/*! This constructor creates a line by given the three coefficents of the line.
    A line is specified by the formula ay + bx + c = 0.
    \param dA a coefficients of the line
    \param dB b coefficients of the line
    \param dC c coefficients of the line */
Line::Line(float dA, float dB, float dC) {
    m_a = dA;
    m_b = dB;
    m_c = dC;
}

Line::Line(Vector pos1, Vector pos2) {
    setLineFromTwoPoints(pos1, pos2);
}

Line::Line(Vector vec, AngleDeg angle) {
    // calculate point somewhat further in direction 'angle' and make
    // line from these two points.
    setLineFromTwoPoints(vec, vec + Vector(1, angle, POLAR));
}

void Line::LineFromRay(Ray r) {
    //  if (fabs(r.direction.y) < FLOAT_EPS && fabs(r.direction.x) < FLOAT_EPS)
    //    my_error("LineFromRay: dir can not be zero");
    LineFromTwoPoints(r.origin, r.origin + r.direction);
}



/*! This method returns the intersection point between the current Line and
    the specified line.
    \param line line with which the intersection should be calculated.
    \return Vector position that is the intersection point. */
Vector Line::getIntersection(Line line) {
    Vector pos = Vector(1e6, 1e6);
    float x, y;
    if ((m_a / m_b) == (line.getACoefficient() / line.getBCoefficient()))
        return pos; // lines are parallel, no intersection
    if (m_a == 0) // bx + c = 0 and a2*y + b2*x + c2 = 0 ==> x = -c/b
    { // calculate x using the current line
        x = -m_c / m_b; // and calculate the y using the second line
        y = line.getYGivenX(x);
    } else if (line.getACoefficient() == 0) { // ay + bx + c = 0 and b2*x + c2 = 0 ==> x = -c2/b2
        x = -line.getCCoefficient() / line.getBCoefficient(); // calculate x using
        y = getYGivenX(x); // 2nd line and calculate y using current line
    }        // ay + bx + c = 0 and a2y + b2*x + c2 = 0
        // y = (-b2/a2)x - c2/a2
        // bx = -a*y - c =>  bx = -a*(-b2/a2)x -a*(-c2/a2) - c ==>
        // ==> a2*bx = a*b2*x + a*c2 - a2*c ==> x = (a*c2 - a2*c)/(a2*b - a*b2)
        // calculate x using the above formula and the y using the current line
    else {
        x = (m_a * line.getCCoefficient() - line.getACoefficient() * m_c) /
                (line.getACoefficient() * m_b - m_a * line.getBCoefficient());
        y = getYGivenX(x);
    }

    return Vector(x, y);
}

bool Line::RayIntersection(Ray r, Vector &pInt) {
    Line rLine = Line(r);

    pInt = getIntersection(rLine);

    if (r.InRightDir(pInt)) return true;
    else return false;
}

/*! This method calculates the intersection points between the current line
    and the circle specified with as center 'posCenter' and radius 'dRadius'.
    The number of solutions are returned and the corresponding points are put
    in the third and fourth argument of the method
    \param c circle with which intersection points should be found
    \param posSolution1 first intersection (if any)
    \param posSolution2 second intersection (if any) */
int Line::getCircleIntersectionPoints(Circle circle,
        Vector *posSolution1, Vector *posSolution2) {
    int iSol;
    float dSol1 = 0, dSol2 = 0;
    float h = circle.getCenter().getX();
    float k = circle.getCenter().getY();

    // line:   x = -c/b (if a = 0)
    // circle: (x-h)^2 + (y-k)^2 = r^2, with h = center.x and k = center.y
    // fill in:(-c/b-h)^2 + y^2 -2ky + k^2 - r^2 = 0
    //         y^2 -2ky + (-c/b-h)^2 + k^2 - r^2 = 0
    // and determine solutions for y using abc-formula
    if (fabs(m_a) < EPSILON) {
        iSol = Geometry::abcFormula(1, -2 * k, ((-m_c / m_b) - h)*((-m_c / m_b) - h)
                + k * k - circle.getRadius() * circle.getRadius(), &dSol1, &dSol2);
        posSolution1->setVector((-m_c / m_b), dSol1);
        posSolution2->setVector((-m_c / m_b), dSol2);
        return iSol;
    }

    // ay + bx + c = 0 => y = -b/a x - c/a, with da = -b/a and db = -c/a
    // circle: (x-h)^2 + (y-k)^2 = r^2, with h = center.x and k = center.y
    // fill in:x^2 -2hx + h^2 + (da*x-db)^2 -2k(da*x-db) + k^2 - r^2 = 0
    //         x^2 -2hx + h^2 + da^2*x^2 + 2da*db*x + db^2 -2k*da*x -2k*db
    //                                                         + k^2 - r^2 = 0
    //       (1+da^2)*x^2 + 2(da*db-h-k*da)*x + h2 + db^2  -2k*db + k^2 - r^2 = 0
    // and determine solutions for x using abc-formula
    // fill in x in original line equation to get y coordinate
    float da = -m_b / m_a;
    float db = -m_c / m_a;

    float dA = 1 + da*da;
    float dB = 2 * (da * db - h - k * da);
    float dC = h * h + db * db - 2 * k * db + k * k - circle.getRadius() * circle.getRadius();

    iSol = Geometry::abcFormula(dA, dB, dC, &dSol1, &dSol2);

    posSolution1->setVector(dSol1, da * dSol1 + db);
    posSolution2->setVector(dSol2, da * dSol2 + db);
    return iSol;
}

/*! This method returns the tangent line to a Vector. This is the line
    between the specified position and the closest point on the line to this
    position.
    \param pos Vector point with which tangent line is calculated.
    \return Line line tangent to this position */
Line Line::getTangentLine(Vector pos) {
    // ay + bx + c = 0 -> y = (-b/a)x + (-c/a)
    // tangent: y = (a/b)*x + C1 -> by - ax + C2 = 0 => C2 = ax - by
    // with pos.y = y, pos.x = x
    return Line(m_b, -m_a, m_a * pos.getX() - m_b * pos.getY());
}

/*! This method returns the closest point on a line to a given position.
    \param pos point to which closest point should be determined
    \return Vector closest point on line to 'pos'. */
Vector Line::getPointOnLineClosestTo(Vector pos) {
    //Line l2 = getPerpendicular( pos );  // get tangent line
    Line l2 = getTangentLine(pos); // get tangent line
    return getIntersection(l2); // and intersection between the two lines
}

/*! This method returns the distance between a specified position and the
    closest point on the given line.
    \param pos position to which distance should be calculated
    \return float indicating the distance to the line. */
float Line::getDistanceWithPoint(Vector pos) {
    //float dist1 = pos.getDistanceTo( getPointOnLineClosestTo( pos ) );
    float dist2 = fabs((m_a * pos.y + m_b * pos.x + m_c) / sqrt(m_a * m_a + m_b * m_b));

    return dist2;
    //return pos.getDistanceTo( getPointOnLineClosestTo( pos ) );
}

int Line::getSidePoint(Vector pos) {
    float val = m_a * pos.y + m_b * pos.x + m_c;
    if (val > 0) return 1;
    else if (val < 0) return -1;
    else return 0;
}

/*! This method determines whether the projection of a point on the
    current line lies between two other points ('point1' and 'point2')
    that lie on the same line.

    \param pos point of which projection is checked.
    \param point1 first point on line
    \param point2 second point on line
    \return true when projection of 'pos' lies between 'point1' and 'point2'.*/
bool Line::isInBetween(Vector pos, Vector point1, Vector point2) {
    pos = getPointOnLineClosestTo(pos); // get closest point
    float dDist = point1.getDistanceTo(point2); // get distance between 2 pos

    // if the distance from both points to the projection is smaller than this
    // dist, the pos lies in between.
    return pos.getDistanceTo(point1) <= dDist &&
            pos.getDistanceTo(point2) <= dDist;
}

/**
 * This method gets a perpendicular to this line on point p
 *
 * @param p point
 * @return perpendicular
 */
Line Line::getPerpendicular(Vector pos) const {
    ///sjdsh
    return Line(m_b, -m_a, m_a * pos.getX() - m_b * pos.getY());
    //	if(m_b==0)
    //	{
    //		return Line(0,1,pos.getX());
    //	}
    //	return Line(1,-m_a/m_b,-pos.getY()+(m_a/m_b)*pos.getX());
}

void Line::setLineFromTwoPoints(Vector pos1, Vector pos2) {
    // 1*y + bx + c = 0 => y = -bx - c
    // with -b the direction coefficient (or slope)
    // and c = - y - bx
    float dA, dB, dC;
    float dTemp = pos2.getX() - pos1.getX(); // determine the slope
    if (fabs(dTemp) < EPSILON) {
        // ay + bx + c = 0 with vertical slope=> a = 0, b = 1
        dA = 0.0;
        dB = 1.0;
    } else {
        // y = (-b)x -c with -b the slope of the line
        dA = 1.0;
        dB = -(pos2.getY() - pos1.getY()) / dTemp;
    }
    // ay + bx + c = 0 ==> c = -a*y - b*x
    dC = -dA * pos2.getY() - dB * pos2.getX();

    m_a = dA;
    m_b = dB;
    m_c = dC;
}

/*! This method calculates the y coordinate given the x coordinate
    \param x coordinate
    \return y coordinate on this line */
float Line::getYGivenX(float x) {
    if (m_a == 0) {
        return 0;
    }
    // ay + bx + c = 0 ==> ay = -(b*x + c)/a
    return -(m_b * x + m_c) / m_a;
}

/*! This method calculates the x coordinate given the x coordinate
    \param y coordinate
    \return x coordinate on this line */
float Line::getXGivenY(float y) {
    if (m_b == 0) {
        return 0;
    }
    // ay + bx + c = 0 ==> bx = -(a*y + c)/a
    return -(m_a * y + m_c) / m_b;
}

/*! This method returns the a coefficient from the line ay + bx + c = 0.
    \return a coefficient of the line. */
float Line::getACoefficient() const {
    return m_a;
}

/*! This method returns the b coefficient from the line ay + bx + c = 0.
    \return b coefficient of the line. */
float Line::getBCoefficient() const {
    return m_b;
}

/*! This method returns the c coefficient from the line ay + bx + c = 0.
    \return c coefficient of the line. */
float Line::getCCoefficient() const {
    return m_c;
}

/*****************************************************************************/
/********************* CLASS RECTANGLE ***************************************/
/*****************************************************************************/

/*! This is the constructor of a Rectangle. Two points will be given. The
    order does not matter as long as two opposite points are given (left
    top and right bottom or right top and left bottom).
    \param pos first point that defines corner of rectangle
    \param pos2 second point that defines other corner of rectangle
    \return rectangle with 'pos' and 'pos2' as opposite corners. */
Rect::Rect(Vector pos, Vector pos2) {
    setRectanglePoints(pos, pos2);
}

/*! This method sets the upper left and right bottom point of the current
    rectangle.
    \param pos first point that defines corner of rectangle
    \param pos2 second point that defines other corner of rectangle */
void Rect::setRectanglePoints(Vector pos1, Vector pos2) {
    m_posLeftTop.setX(max(pos1.getX(), pos2.getX()));
    m_posLeftTop.setY(min(pos1.getY(), pos2.getY()));
    m_posRightBottom.setX(min(pos1.getX(), pos2.getX()));
    m_posRightBottom.setY(max(pos1.getY(), pos2.getY()));
}


/*! This method determines whether the given position lies inside the current
    rectangle.
    \param pos position which is checked whether it lies in rectangle
    \return true when 'pos' lies in the rectangle, false otherwise */
bool Rect::isInside(Vector pos) {
    return pos.isBetweenX(m_posRightBottom.getX(), m_posLeftTop.getX()) &&
            pos.isBetweenY(m_posLeftTop.getY(), m_posRightBottom.getY());

}

/*! This method sets the top left position of the rectangle
    \param pos new top left position of the rectangle
    \return true when update was successful */
bool Rect::setPosLeftTop(Vector pos) {
    m_posLeftTop = pos;
    return true;
}

/*! This method returns the top left position of the rectangle
    \return top left position of the rectangle */
Vector Rect::getPosLeftTop() {
    return m_posLeftTop;
}

/*! This method sets the right bottom position of the rectangle
    \param pos new right bottom position of the rectangle
    \return true when update was succesfull */
bool Rect::setPosRightBottom(Vector pos) {
    m_posRightBottom = pos;
    return true;
}

/*! This method returns the right bottom position of the rectangle
    \return top right bottom of the rectangle */
Vector Rect::getPosRightBottom() {
    return m_posRightBottom;
}

Ray::Ray(Vector orig, Vector dir) {
    origin = orig;
    if (fabs(dir.y) < FLOAT_EPS && fabs(dir.x) < FLOAT_EPS) {
        ////my_error("Ray: dir can not be zero");
        direction = Vector(1, 0);
    } else {
        direction = dir;
        direction = direction.normalize();
    }
}

bool Ray::OnRay(Vector pt) {
    Vector v = pt - origin;
    return (fabs(sin((v.getDirection() - direction.getDirection()) * v.length()) / 180.0 * M_PI) < FLOAT_EPS)
            ? true : false;
}

bool Ray::InRightDir(Vector pt) {

    return (fabs(GetNormalizeAngleDeg((pt - origin).getDirection() - direction.getDirection())) < 10)
            ? true : false;
}

bool Ray::InRightSide(Vector pt) {

    return ((GetNormalizeAngleDeg((pt - origin).getDirection() - direction.getDirection())) < 0.0)
            ? true : false; //UNTESTED
}

/**
bool Ray::intersection(Line l, Vector *pPt)
{
 return l.RayIntersection(*this, pPt);
}
 **/

/*

bool Ray::intersection(Ray r, Vector *pPt)
{
  Line thisLine(*this), argLine(r);

  if (thisLine.SameSlope(argLine))
    return false;
 *pPt = thisLine.intersection(argLine);

  // now make sure that the intersection is the correct direction on both lines
  return  (InRightDir(*pPt) && r.InRightDir(*pPt))
       //      fabs(GetNormalizeAngleDeg((*pPt - origin).getDirection() - direction.getDirection())) < 10 &&
       //fabs(GetNormalizeAngleDeg((*pPt - r.origin).getDirection() - r.direction.getDirection())) < 10)
    ? true : false;
}
 */
void Line::LineFromTwoPoints(Vector pt1, Vector pt2) {
    setLineFromTwoPoints(pt1, pt2);
    /**
      float temp = (pt2.x - pt1.x);
      if (fabs(temp) < FLOAT_EPS) {
    //    if (fabs(pt2.y - pt1.y) < FLOAT_EPS)
    //      my_error("LineFromTwoPoints: points can not be the same!");
        m_a = 1;
        m_b = 0;
      } else {
        float m = (pt2.y - pt1.y) / temp;
        m_a = -m;
        m_b = 1;
      }
      m_c = -(m_b * pt2.x + m_b * pt2.y);
     */
}


// intersects a ray and a cricle
// return the number of solutions
// psol1 1 is not as afar along the ray as psol2

int Ray::CircleIntersect(float rad, Vector center, Vector* psol1, Vector* psol2) {

    float a, b, c, disc;
    float t1, t2;
    a = Sqr(direction.x) + Sqr(direction.y);
    b = 2.0 * ((origin.x - center.x) * direction.x + (origin.y - center.y) * direction.y);
    c = Sqr(origin.x - center.x) + Sqr(origin.y - center.y) - Sqr(rad);

    disc = Sqr(b) - 4 * a * c;
    if (disc < 0) {
        return 0;
    }

    disc = sqrt(disc);
    t1 = (-b + disc) / (2.0 * a);
    t2 = (-b - disc) / (2.0 * a);
    //DebugGeom(printf(" RCI: t1: %f\tt2: %f\n", t1, t2));

    if (t1 > t2) {
        //DebugGeom(printf(" RCI: reversing t1, t2\n"));
        float temp = t1;
        t1 = t2;
        t2 = temp;
    }

    if (t1 > 0.0) {
        if (t2 > 0.0) {
            *psol1 = origin + direction * t1;
            *psol2 = origin + direction * t2;
            //DebugGeom(printf(" RCI:two sols\n"));
            return 2;
        } else {
            //my_error("RayCircleIntersect: weird roots");
            return 0;
        }
    } else if (t2 > 0.0) {
        *psol1 = origin + direction * t2;
        //DebugGeom(printf(" RCI:t2 only sol\n"));
        return 1;
    } else
        return 0;

    return 0;
}

/*
Vector Ray::RectangleIntersection(Rectangle R)
{
  return R.RayIntersection(*this);
}
 */

Vector Ray::GetClosestPoint(Vector pt) {
    Line l(*this);
    Vector close_pt = l.ProjectPoint(pt);
    if (InRightDir(close_pt))
        return close_pt;
    else
        return origin;
}




float Ray::dist(Vector pt) {

    return pt.getDistanceTo(GetClosestPoint(pt));
}




#ifdef OLD_CODE
// intersects a ray and a cricle
// return the number of solutions
// psol1 1 is not as afar along the ray as psol2

int RayCircleIntersect(Ray r, float rad, Vector center,
        Vector* psol1, Vector* psol2) {

    float a, b, c, disc;
    float t1, t2;
    a = Sqr(r.direction.x) + Sqr(r.direction.y);
    b = 2.0 * ((r.origin.x - center.x) * r.direction.x + (r.origin.y - center.y) * r.direction.y);
    c = Sqr(r.origin.x - center.x) + Sqr(r.origin.y - center.y) - Sqr(rad);

    disc = Sqr(b) - 4 * a * c;
    if (disc < 0) {
        return 0;
    }

    disc = sqrt(disc);
    t1 = (-b + disc) / (2.0 * a);
    t2 = (-b - disc) / (2.0 * a);
    //DebugGeom(printf(" RCI: t1: %f\tt2: %f\n", t1, t2));

    if (t1 > t2) {
        //DebugGeom(printf(" RCI: reversing t1, t2\n"));
        float temp = t1;
        t1 = t2;
        t2 = temp;
    }

    if (t1 > 0.0) {
        if (t2 > 0.0) {
            *psol1 = r.origin + r.direction * t1;
            *psol2 = r.origin + r.direction * t2;
            //DebugGeom(printf(" RCI:two sols\n"));
            return 2;
        } else {
            //my_error("RayCircleIntersect: weird roots");
            return 0;
        }
    } else if (t2 > 0.0) {
        *psol1 = r.origin + r.direction * t2;
        //DebugGeom(printf(" RCI:t2 only sol\n"));
        return 1;
    } else
        return 0;

    return 0;
}
#endif

