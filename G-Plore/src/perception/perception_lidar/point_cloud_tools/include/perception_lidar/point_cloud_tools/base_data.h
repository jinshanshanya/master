#ifndef _BASE_DATA_
#define _BASE_DATA_

namespace perception
{
    template<typename T>
    class Cube
    {
        public:
            // constructor
            Cube(){ };
            Cube(T xmin, T xmax, T ymin, T ymax, T zmin, T zmax)
            : xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax), zmin_(zmin), zmax_(zmax){  };
            // destructor
            ~Cube() { };
            // function
            inline Cube operator+(Cube& cube);
            inline Cube operator-(Cube& cube);
            inline Cube operator/(T divisor);
            // variable
            T xmin_, xmax_, ymin_, ymax_, zmin_, zmax_;
    };

    template<typename T>
    class Point2D
    {
        public:
            // constructor
            Point2D(){};
            Point2D(T x, T y)
            : x_(x), y_(y){};
            // destructor
            virtual ~Point2D(){};
            // function
            // inline Point2D<T> operator=(Point2D<T>& point2d);
            // variable
            T x_, y_;
    };

    // template<typename T>
    // class Point3D
    // {
    //     public:
    //         // constructor
    //         Point3D(){ };
    //         Point3D(T x, T y, T z)
    //         : x_(x), y_(y), z_(z){ };
    //         // destructor
    //         ~Point3D(){ };
    //         // variable
    //         T x_, y_, z_;
    // };

    template<typename T>
    class Point3D : public Point2D<T>
    {
        public:
            // constructor
            Point3D(){ };
            Point3D(T x, T y, T z)
            : Point2D<T>(x, y), z_(z) { };
            // destructor
            ~Point3D() { };
            // function
            // inline Point3D<T> operator=(Point3D<T>& point3d);
            // variable
            T z_;
    };

}

// template<typename T>
// inline perception::Point2D<T> perception::Point2D<T>::operator=(Point2D<T>& point2d)
// {
//     this->x_ = point2d.x_;
//     this->y_ = point2d.y_;
//     return *this;
// }

// template<typename T>
// inline perception::Point3D<T> perception::Point3D<T>::operator=(Point3D<T>& point3d)
// {
//     this->x_ = point3d.x_;
//     this->y_ = point3d.y_;
//     this->z_ = point3d.z_;
//     return *this;
// }

template<typename T>
inline perception::Cube<T> perception::Cube<T>::operator+(Cube& cube)
{
    Cube<T> cube_sum;
    cube_sum.xmin_ = xmin_ + cube.xmin_;
    cube_sum.xmax_ = xmax_ + cube.xmax_;
    cube_sum.ymin_ = ymin_ + cube.ymin_;
    cube_sum.ymax_ = ymax_ + cube.ymax_;
    cube_sum.zmin_ = zmin_ + cube.zmin_;
    cube_sum.zmax_ = zmax_ + cube.zmax_;
    return cube_sum;
}

template<typename T>
inline perception::Cube<T> perception::Cube<T>::operator-(Cube& cube)
{
    Cube<T> cube_sum;
    cube_sum.xmin_ = xmin_ - cube.xmin_;
    cube_sum.xmax_ = xmax_ - cube.xmax_;
    cube_sum.ymin_ = ymin_ - cube.ymin_;
    cube_sum.ymax_ = ymax_ - cube.ymax_;
    cube_sum.zmin_ = zmin_ - cube.zmin_;
    cube_sum.zmax_ = zmax_ - cube.zmax_;
    return cube_sum;
}

template<typename T>
inline perception::Cube<T> perception::Cube<T>::operator/(T divisor)
{
    Cube<T> cube_sum;
    cube_sum.xmin_ = xmin_ / divisor;
    cube_sum.xmax_ = xmax_ / divisor;
    cube_sum.ymin_ = ymin_  / divisor;
    cube_sum.ymax_ = ymax_ / divisor;
    cube_sum.zmin_ = zmin_ / divisor;
    cube_sum.zmax_ = zmax_ / divisor;
    return cube_sum;
}

#endif