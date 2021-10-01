#ifndef XVIZ_PHYSICS_CONVERT_HPP
#define XVIZ_PHYSICS_CONVERT_HPP

#include <Eigen/Geometry>
#include <LinearMath/btTransform.h>
#include <vector>

namespace xsim {

inline Eigen::Vector3f toEigenVector(const std::vector<float>& vec) {return Eigen::Vector3f(vec[0],vec[1],vec[2]);}
inline Eigen::Vector3f toEigenVector(const btVector3& vec) {return Eigen::Vector3f(vec.x(),vec.y(),vec.z());}
inline btVector3 toBulletVector(const Eigen::Vector3f& vec) {return btVector3(vec[0],vec[1],vec[2]);}

inline Eigen::Isometry3f toEigenTransform(const btTransform& transform) {
  btVector3 transBullet = transform.getOrigin();
  btQuaternion quatBullet = transform.getRotation();
  Eigen::Translation3f transEig;
  transEig = Eigen::Translation3f(toEigenVector(transBullet));
  Eigen::Matrix3f rotEig = Eigen::Quaternionf(quatBullet.w(),quatBullet.x(),quatBullet.y(),quatBullet.z()).toRotationMatrix();
  Eigen::Affine3f out(transEig*rotEig);
  /*Eigen::Isometry3f out ;
  out.translationExt() = transEig.vector() ;
  out.linearExt() = rotEig ;*/
  return Eigen::Isometry3f(out.matrix());
}

inline btTransform toBulletTransform(const Eigen::Affine3f& affine) {
  Eigen::Vector3f transEig = affine.translation();
  Eigen::Matrix3f rotEig = affine.linear();
  Eigen::Quaternionf quatEig = Eigen::Quaternionf(rotEig);
  btVector3 transBullet = toBulletVector(transEig);
  btQuaternion quatBullet = btQuaternion(quatEig.x(), quatEig.y(), quatEig.z(), quatEig.w());
  return btTransform(quatBullet,transBullet);
}


inline btVector3 eigenVectorToBullet(const Eigen::Vector3f &v) {
    return btVector3(btScalar(v.x()), btScalar(v.y()), btScalar(v.z())) ;
}


} // namespace xsim


#endif
