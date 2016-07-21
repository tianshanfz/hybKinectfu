#ifndef _PLANE_H_
#define _PLANE_H_


#include"../../cuda/DataMap.h"
namespace ml {
	template<class FloatType>
	class Plane {
	public:
		Plane() {

		}

		Plane(const Point3f& p0, const Point3f& p1, const Point3f& p2) {
			const Point3f  p[] = { p0, p1, p2 };
			createFromPoints(p);
		}

		Plane(const Point3f *points) {
			createFromPoints(points);
		}

		Plane(const Point3f& normal, FloatType dist) {
			m_Normal = normal;
			m_Distance = dist;
		}

		Plane(const Vector3f &normal, const Point3f& p) {
			m_Normal = normal;
			m_Distance = dot(m_Normal, p);
		}

		inline Point3f getNormal() const {
			return m_Normal;
		}

		inline FloatType getDistance() const {
			return m_Distance;
		}

		inline FloatType distanceToPoint(const Point3f& point) const {
			return dot(m_Normal, point) - m_Distance;
		}

		inline FloatType distanceToPointAbs(const Point3f& point) const {
			return std::abs(distanceToPoint(point));
		}

		inline Point3f projectPointToPlane(const Point3f& point) const {
			return point - distanceToPoint(point) * getNormal();
		}

		inline static Plane<FloatType> xyPlane() {
			return Plane<FloatType>(make_float3(0, 0, 1), 0);
		}
		inline static Plane<FloatType> xzPlane() {
			return Plane<FloatType>(make_float3(0, 1, 0), 0);
		}
		inline static Plane<FloatType> yzPlane() {
			return Plane<FloatType>(make_float3(1, 0, 0), 0);
		}

		inline Plane<FloatType> operator-() const {
			return Plane<FloatType>(make_float3(0, 0, 0) - m_Normal, m_Distance);
		}

	private:
		void createFromPoints(const Point3f* points)
		{
			m_Normal = normalize(cross(points[1] - points[0], points[2] - points[0]));
			m_Distance = dot(m_Normal, points[0]);
			//make sure normal points away from origin (i.e., distance is positive)
			if (m_Distance < (FloatType)0) {
				m_Distance = -m_Distance;
				m_Normal = make_float3(-m_Normal.x,-m_Normal.y,-m_Normal.z);
			}
		}

		Point3f m_Normal;
		FloatType m_Distance;
	};

	typedef Plane<float> Planef;
	typedef Plane<float> Planed;

}

#endif
