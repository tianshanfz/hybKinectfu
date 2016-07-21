/*
 * Mat.h
 *
 *  Created on: May 26, 2016
 *      Author: hanyinbo
 */

#ifndef MAT_H_
#define MAT_H_


#include<stdio.h>
#include"cuda_include.h"
struct Mat33
{
	union {
		struct {
			float m11; float m12; float m13;
			float m21; float m22; float m23;
			float m31; float m32; float m33;
		};
		float entries[9];
		float entries2[3][3];
	};

	inline __device__   __host__
		Mat33()
	{
	}
	inline __device__ __host__ Mat33(const float values[9]) {
		m11 = values[0];	m12 = values[1];	m13 = values[2];
		m21 = values[3];	m22 = values[4];	m23 = values[5];
		m31 = values[6];	m32 = values[7];	m33 = values[8];
	}
	inline __device__ __host__ Mat33(const Mat33& other) {
		m11 = other.m11;	m12 = other.m12;	m13 = other.m13;
		m21 = other.m21;	m22 = other.m22;	m23 = other.m23;
		m31 = other.m31;	m32 = other.m32;	m33 = other.m33;
	}
	inline __device__ __host__ float det() const {
		return
			+m11*m22*m33
			+ m12*m23*m31
			+ m13*m21*m32
			- m31*m22*m13
			- m32*m23*m11
			- m33*m21*m12;
	}
	inline __device__ __host__ Mat33& operator=(const Mat33 &other) {
		m11 = other.m11;	m12 = other.m12;	m13 = other.m13;
		m21 = other.m21;	m22 = other.m22;	m23 = other.m23;
		m31 = other.m31;	m32 = other.m32;	m33 = other.m33;
		return *this;
	}
	inline __device__ __host__ const float* ptr() const {
		return entries;
	}

	static inline __device__ __host__ Mat33 getIdentity() {
		Mat33 res;
		res.setZero();
		res.m11 = res.m22 = res.m33 = 1.0f;
		return res;
	}
	inline __device__ __host__ void setZero(float value = 0.0f) {
		m11 = m12 = m13 = value;
		m21 = m22 = m23 = value;
		m31 = m32 = m33 = value;
	}
	static inline __device__ __host__ Mat33 getZeroMatrix() {
		Mat33 res;
		res.setZero();
		return res;
	}
	inline __device__ __host__
		Mat33 inverse()const
	{
		/*	float3 inv_data[3];
		float det = data[0].x*data[1].y*data[2].z + data[1].x*data[2].y*data[0].z +data[2].x*data[0].y*data[1].z
		- data[0].x*data[2].y*data[1].z -data[1].x*data[0].y*data[2].z - data[2].x*data[1].y*data[0].z;
		inv_data[0].x = (data[1].y*data[2].z - data[2].y*data[1].z) / det;
		inv_data[0].y = (data[2].y*data[0].z - data[0].y*data[2].z) / det;
		inv_data[0].z = (data[0].y*data[1].z - data[1].y*data[0].z) / det;
		inv_data[1].x = (data[2].x*data[1].z - data[1].x*data[2].z) / det;
		inv_data[1].y = (data[0].x*data[2].z - data[2].x*data[0].z) / det;
		inv_data[1].z = (data[1].x*data[0].z - data[0].x*data[1].z) / det;
		inv_data[2].x = (data[1].x*data[2].y - data[2].x*data[1].y) / det;
		inv_data[2].y = (data[2].x*data[0].y - data[0].x*data[2].y) / det;
		inv_data[2].z = (data[0].x*data[1].y - data[1].x*data[0].y) / det;
		return Mat33(inv_data);*/
		Mat33 res;
		res.entries[0] = entries[4] * entries[8] - entries[5] * entries[7];
		res.entries[1] = -entries[1] * entries[8] + entries[2] * entries[7];
		res.entries[2] = entries[1] * entries[5] - entries[2] * entries[4];

		res.entries[3] = -entries[3] * entries[8] + entries[5] * entries[6];
		res.entries[4] = entries[0] * entries[8] - entries[2] * entries[6];
		res.entries[5] = -entries[0] * entries[5] + entries[2] * entries[3];

		res.entries[6] = entries[3] * entries[7] - entries[4] * entries[6];
		res.entries[7] = -entries[0] * entries[7] + entries[1] * entries[6];
		res.entries[8] = entries[0] * entries[4] - entries[1] * entries[3];
		float nom = 1.0f / det();
		return res * nom;
	}
	//! standard matrix matrix multiplication
	inline __device__ __host__  Mat33 operator*(const Mat33 &other) const {
		Mat33 res;
		res.m11 = m11 * other.m11 + m12 * other.m21 + m13 * other.m31;
		res.m12 = m11 * other.m12 + m12 * other.m22 + m13 * other.m32;
		res.m13 = m11 * other.m13 + m12 * other.m23 + m13 * other.m33;

		res.m21 = m21 * other.m11 + m22 * other.m21 + m23 * other.m31;
		res.m22 = m21 * other.m12 + m22 * other.m22 + m23 * other.m32;
		res.m23 = m21 * other.m13 + m22 * other.m23 + m23 * other.m33;

		res.m31 = m31 * other.m11 + m32 * other.m21 + m33 * other.m31;
		res.m32 = m31 * other.m12 + m32 * other.m22 + m33 * other.m32;
		res.m33 = m31 * other.m13 + m32 * other.m23 + m33 * other.m33;
		return res;
	}
	inline __device__ __host__ float3 operator*(const float3 &v) const {
		return make_float3(
			m11*v.x + m12*v.y + m13*v.z,
			m21*v.x + m22*v.y + m23*v.z,
			m31*v.x + m32*v.y + m33*v.z
			);
	}
	inline __device__ __host__ Mat33 operator*(const float t) const {
		Mat33 res;
		res.m11 = m11 * t;		res.m12 = m12 * t;		res.m13 = m13 * t;
		res.m21 = m21 * t;		res.m22 = m22 * t;		res.m23 = m23 * t;
		res.m31 = m31 * t;		res.m32 = m32 * t;		res.m33 = m33 * t;
		return res;
	}
	inline __device__ __host__ Mat33 operator+(const Mat33 &other) const {
		Mat33 res;
		res.m11 = m11 + other.m11;	res.m12 = m12 + other.m12;	res.m13 = m13 + other.m13;
		res.m21 = m21 + other.m21;	res.m22 = m22 + other.m22;	res.m23 = m23 + other.m23;
		res.m31 = m31 + other.m31;	res.m32 = m32 + other.m32;	res.m33 = m33 + other.m33;
		return res;
	}

	inline __device__ __host__ Mat33 operator-(const Mat33 &other) const {
		Mat33 res;
		res.m11 = m11 - other.m11;	res.m12 = m12 - other.m12;	res.m13 = m13 - other.m13;
		res.m21 = m21 - other.m21;	res.m22 = m22 - other.m22;	res.m23 = m23 - other.m23;
		res.m31 = m31 - other.m31;	res.m32 = m32 - other.m32;	res.m33 = m33 - other.m33;
		return res;
	}
	void Print()const
	{
		printf("%.2f  %.2f  %.2f\n%.2f  %.2f  %.2f\n%.2f  %.2f  %.2f", m11, m12, m13, m21, m22, m23, m31, m32, m33);
	}
	friend ostream& operator << (ostream& output,Mat33& c) ;//定义运算符“<<”重载函数
};



struct Mat44
{
	union {
		struct {
			float m11; float m12; float m13; float m14;
				float m21; float m22; float m23; float m24;
				float m31; float m32; float m33; float m34;
				float m41; float m42; float m43; float m44;
		};
		float entries[16];
		float entries2[4][4];
	};

	inline __device__   __host__
		Mat44()
	{
	}
	inline __device__ __host__ Mat44(const float values[16]) {
		m11 = values[0];	m12 = values[1];	m13 = values[2]; m14= values[3];
		m21 = values[4];	m22 = values[5];	m23 = values[6]; m24 = values[7];
		m31 = values[8];	m32 = values[9];	m33 = values[10]; m34 = values[11];
		m41 = values[12];	m42 = values[13];	m43 = values[14]; m44 = values[15];
	}
	inline __device__ __host__ Mat44(const Mat44& other) {
		m11 = other.m11;	m12 = other.m12;	m13 = other.m13; m14 = other.m14;
		m21 = other.m21;	m22 = other.m22;	m23 = other.m23; m24 = other.m24;
		m31 = other.m31;	m32 = other.m32;	m33 = other.m33; m34 = other.m34;
		m41 = other.m41;	m42 = other.m42;	m43 = other.m43; m44 = other.m44;
	}

	inline __device__ __host__ Mat44& operator=(const Mat44 &other) {
		m11 = other.m11;	m12 = other.m12;	m13 = other.m13; m14 = other.m14;
		m21 = other.m21;	m22 = other.m22;	m23 = other.m23; m24 = other.m24;
		m31 = other.m31;	m32 = other.m32;	m33 = other.m33; m34 = other.m34;
		m41 = other.m41;	m42 = other.m42;	m43 = other.m43; m44 = other.m44;
		return *this;
	}
	inline __device__ __host__ const float* ptr() const {
		return entries;
	}

	static inline __device__ __host__ Mat44 getIdentity() {
		Mat44 res;
		res.setZero();
		res.m11 = res.m22 = res.m33 = res.m44=1.0f;
		return res;
	}
	inline __device__ __host__ void setZero(float value = 0.0f) {
		m11 = m12 = m13 =m14= value;
		m21 = m22 = m23 = m24=value;
		m31 = m32 = m33 = m34=value;
		m41 = m42 = m43 = m44 = value;
	}
	static inline __device__ __host__ Mat44 getZeroMatrix() {
		Mat44 res;
		res.setZero();
		return res;
	}

	// untested
	inline __device__ __host__ float4 operator*(const float4& v) const
	{
		return make_float4(
			m11*v.x + m12*v.y + m13*v.z + m14*v.w,
			m21*v.x + m22*v.y + m23*v.z + m24*v.w,
			m31*v.x + m32*v.y + m33*v.z + m34*v.w,
			m41*v.x + m42*v.y + m43*v.z + m44*v.w
			);
	}
	//! not tested
	inline __device__ __host__ Mat44 operator*(const Mat44 &other) const {
		Mat44 res;
		res.m11 = m11*other.m11 + m12*other.m21 + m13*other.m31 + m14*other.m41;
		res.m12 = m11*other.m12 + m12*other.m22 + m13*other.m32 + m14*other.m42;
		res.m13 = m11*other.m13 + m12*other.m23 + m13*other.m33 + m14*other.m43;
		res.m14 = m11*other.m14 + m12*other.m24 + m13*other.m34 + m14*other.m44;

		res.m21 = m21*other.m11 + m22*other.m21 + m23*other.m31 + m24*other.m41;
		res.m22 = m21*other.m12 + m22*other.m22 + m23*other.m32 + m24*other.m42;
		res.m23 = m21*other.m13 + m22*other.m23 + m23*other.m33 + m24*other.m43;
		res.m24 = m21*other.m14 + m22*other.m24 + m23*other.m34 + m24*other.m44;

		res.m31 = m31*other.m11 + m32*other.m21 + m33*other.m31 + m34*other.m41;
		res.m32 = m31*other.m12 + m32*other.m22 + m33*other.m32 + m34*other.m42;
		res.m33 = m31*other.m13 + m32*other.m23 + m33*other.m33 + m34*other.m43;
		res.m34 = m31*other.m14 + m32*other.m24 + m33*other.m34 + m34*other.m44;

		res.m41 = m41*other.m11 + m42*other.m21 + m43*other.m31 + m44*other.m41;
		res.m42 = m41*other.m12 + m42*other.m22 + m43*other.m32 + m44*other.m42;
		res.m43 = m41*other.m13 + m42*other.m23 + m43*other.m33 + m44*other.m43;
		res.m44 = m41*other.m14 + m42*other.m24 + m43*other.m34 + m44*other.m44;

		return res;
	}

	inline __device__ __host__ Mat44 operator*(const float t) const {
		Mat44 res;
		res.m11 = m11 * t;		res.m12 = m12 * t;		res.m13 = m13 * t; res.m14 = m14 * t;
		res.m21 = m21 * t;		res.m22 = m22 * t;		res.m23 = m23 * t; res.m24 = m24 * t;
		res.m31 = m31 * t;		res.m32 = m32 * t;		res.m33 = m33 * t; res.m34 = m34 * t;
		res.m41 = m41 * t;		res.m42 = m42 * t;		res.m43 = m43 * t; res.m44 = m44 * t;
		return res;
	}
	inline __device__ __host__ Mat44 operator+(const Mat44 &other) const {
		Mat44 res;
		res.m11 = m11 + other.m11;	res.m12 = m12 + other.m12;	res.m13 = m13 + other.m13; res.m14 = m14 + other.m14;
		res.m21 = m21 + other.m21;	res.m22 = m22 + other.m22;	res.m23 = m23 + other.m23; res.m24 = m24 + other.m24;
		res.m31 = m31 + other.m31;	res.m32 = m32 + other.m32;	res.m33 = m33 + other.m33; res.m34 = m34 + other.m34;
		res.m41 = m41 + other.m41;	res.m42 = m42 + other.m42;	res.m43 = m43 + other.m43; res.m44 = m44 + other.m44;
		return res;
	}

	inline __device__ __host__ Mat44 operator-(const Mat44 &other) const {
		Mat44 res;
		res.m11 = m11 - other.m11;	res.m12 = m12 - other.m12;	res.m13 = m13 - other.m13; res.m14 = m14 - other.m14;
		res.m21 = m21 - other.m21;	res.m22 = m22 - other.m22;	res.m23 = m23 - other.m23; res.m24 = m24 - other.m24;
		res.m31 = m31 - other.m31;	res.m32 = m32 - other.m32;	res.m33 = m33 - other.m33; res.m34 = m34 - other.m34;
		res.m41 = m41 - other.m41;	res.m42 = m42 - other.m42;	res.m43 = m43 - other.m43; res.m44 = m44 - other.m44;
		return res;
	}
	static inline __device__ __host__  void swap(float& v0, float& v1) {
		float tmp = v0;
		v0 = v1;
		v1 = tmp;
	}
	inline __device__ __host__ void transpose() {
		swap(m12, m21);
		swap(m13, m31);
		swap(m23, m32);
		swap(m41, m14);
		swap(m42, m24);
		swap(m43, m34);
	}
	inline __device__ __host__ float3 getTranslation() const{
		return make_float3(m14,m24,m34);
	}
	inline __device__ __host__ Mat33 getRotation() const{
		float rot[9];
		rot[0]=m11,rot[1]=m12,rot[2]=m13;
		rot[3]=m21,rot[4]=m22,rot[5]=m23;
		rot[6]=m31,rot[7]=m32,rot[8]=m33;
		return Mat33(rot);
	}
	inline __device__ __host__ Mat44 getTranspose() const {
		Mat44 ret = *this;
		ret.transpose();
		return ret;
	}
	inline __device__ __host__ void setRotation(const Mat33& rot) {
			m11=rot.m11,m12=rot.m12,m13=rot.m13;
			m21=rot.m21,m22=rot.m22,m23=rot.m23;
			m31=rot.m31,m32=rot.m32,m33=rot.m33;
		}
	inline __device__ __host__ void setTranslation(const float3& transition)  {
		m14=transition.x;
		m24=transition.y;
		m34=transition.z;
	}
	//! return the inverse matrix; but does not change the current matrix
	inline __device__ __host__ Mat44 getInverse() const {
		float inv[16];

		inv[0] = entries[5] * entries[10] * entries[15] -
			entries[5] * entries[11] * entries[14] -
			entries[9] * entries[6] * entries[15] +
			entries[9] * entries[7] * entries[14] +
			entries[13] * entries[6] * entries[11] -
			entries[13] * entries[7] * entries[10];

		inv[4] = -entries[4] * entries[10] * entries[15] +
			entries[4] * entries[11] * entries[14] +
			entries[8] * entries[6] * entries[15] -
			entries[8] * entries[7] * entries[14] -
			entries[12] * entries[6] * entries[11] +
			entries[12] * entries[7] * entries[10];

		inv[8] = entries[4] * entries[9] * entries[15] -
			entries[4] * entries[11] * entries[13] -
			entries[8] * entries[5] * entries[15] +
			entries[8] * entries[7] * entries[13] +
			entries[12] * entries[5] * entries[11] -
			entries[12] * entries[7] * entries[9];

		inv[12] = -entries[4] * entries[9] * entries[14] +
			entries[4] * entries[10] * entries[13] +
			entries[8] * entries[5] * entries[14] -
			entries[8] * entries[6] * entries[13] -
			entries[12] * entries[5] * entries[10] +
			entries[12] * entries[6] * entries[9];

		inv[1] = -entries[1] * entries[10] * entries[15] +
			entries[1] * entries[11] * entries[14] +
			entries[9] * entries[2] * entries[15] -
			entries[9] * entries[3] * entries[14] -
			entries[13] * entries[2] * entries[11] +
			entries[13] * entries[3] * entries[10];

		inv[5] = entries[0] * entries[10] * entries[15] -
			entries[0] * entries[11] * entries[14] -
			entries[8] * entries[2] * entries[15] +
			entries[8] * entries[3] * entries[14] +
			entries[12] * entries[2] * entries[11] -
			entries[12] * entries[3] * entries[10];

		inv[9] = -entries[0] * entries[9] * entries[15] +
			entries[0] * entries[11] * entries[13] +
			entries[8] * entries[1] * entries[15] -
			entries[8] * entries[3] * entries[13] -
			entries[12] * entries[1] * entries[11] +
			entries[12] * entries[3] * entries[9];

		inv[13] = entries[0] * entries[9] * entries[14] -
			entries[0] * entries[10] * entries[13] -
			entries[8] * entries[1] * entries[14] +
			entries[8] * entries[2] * entries[13] +
			entries[12] * entries[1] * entries[10] -
			entries[12] * entries[2] * entries[9];

		inv[2] = entries[1] * entries[6] * entries[15] -
			entries[1] * entries[7] * entries[14] -
			entries[5] * entries[2] * entries[15] +
			entries[5] * entries[3] * entries[14] +
			entries[13] * entries[2] * entries[7] -
			entries[13] * entries[3] * entries[6];

		inv[6] = -entries[0] * entries[6] * entries[15] +
			entries[0] * entries[7] * entries[14] +
			entries[4] * entries[2] * entries[15] -
			entries[4] * entries[3] * entries[14] -
			entries[12] * entries[2] * entries[7] +
			entries[12] * entries[3] * entries[6];

		inv[10] = entries[0] * entries[5] * entries[15] -
			entries[0] * entries[7] * entries[13] -
			entries[4] * entries[1] * entries[15] +
			entries[4] * entries[3] * entries[13] +
			entries[12] * entries[1] * entries[7] -
			entries[12] * entries[3] * entries[5];

		inv[14] = -entries[0] * entries[5] * entries[14] +
			entries[0] * entries[6] * entries[13] +
			entries[4] * entries[1] * entries[14] -
			entries[4] * entries[2] * entries[13] -
			entries[12] * entries[1] * entries[6] +
			entries[12] * entries[2] * entries[5];

		inv[3] = -entries[1] * entries[6] * entries[11] +
			entries[1] * entries[7] * entries[10] +
			entries[5] * entries[2] * entries[11] -
			entries[5] * entries[3] * entries[10] -
			entries[9] * entries[2] * entries[7] +
			entries[9] * entries[3] * entries[6];

		inv[7] = entries[0] * entries[6] * entries[11] -
			entries[0] * entries[7] * entries[10] -
			entries[4] * entries[2] * entries[11] +
			entries[4] * entries[3] * entries[10] +
			entries[8] * entries[2] * entries[7] -
			entries[8] * entries[3] * entries[6];

		inv[11] = -entries[0] * entries[5] * entries[11] +
			entries[0] * entries[7] * entries[9] +
			entries[4] * entries[1] * entries[11] -
			entries[4] * entries[3] * entries[9] -
			entries[8] * entries[1] * entries[7] +
			entries[8] * entries[3] * entries[5];

		inv[15] = entries[0] * entries[5] * entries[10] -
			entries[0] * entries[6] * entries[9] -
			entries[4] * entries[1] * entries[10] +
			entries[4] * entries[2] * entries[9] +
			entries[8] * entries[1] * entries[6] -
			entries[8] * entries[2] * entries[5];

		float matrixDet = entries[0] * inv[0] + entries[1] * inv[4] + entries[2] * inv[8] + entries[3] * inv[12];

		float matrixDetr = 1.0f / matrixDet;

		Mat44 res;
		for (unsigned int i = 0; i < 16; i++) {
			res.entries[i] = inv[i] * matrixDetr;
		}
		return res;

	}
	friend ostream& operator << (ostream& output,Mat44& c) ;//定义运算符“<<”重载函数
	/*void print()const
	{
		printf("%.4f  %.4f  %.4f %.2f\n%.4f  %.4f  %.4f  %.4f\n%.4f  %.4f  %.4f %.4f\n%.4f  %.4f  %.4f %.4f\n",
			m11, m12, m13,m14,
			m21, m22, m23, m24,
			m31, m32, m33,m34,
			m41,m42,m43,m44);
	}*/
};
__forceinline__ ostream& operator << (ostream& output,Mat33& c) //定义运算符“<<”重载函数
{
   output<<c.m11<<" "<<c.m12<<" "<<c.m13<<" "<<endl;
   output<<c.m21<<" "<<c.m22<<" "<<c.m23<<" "<<endl;
   output<<c.m31<<" "<<c.m32<<" "<<c.m33<<" "<<endl;
   return output;
}
__forceinline__ ostream& operator << (ostream& output,Mat44& c) //定义运算符“<<”重载函数
{
   output<<c.m11<<" "<<c.m12<<" "<<c.m13<<" "<<c.m14<<endl;
   output<<c.m21<<" "<<c.m22<<" "<<c.m23<<" "<<c.m24<<endl;
   output<<c.m31<<" "<<c.m32<<" "<<c.m33<<" "<<c.m34<<endl;
   output<<c.m41<<" "<<c.m42<<" "<<c.m43<<" "<<c.m44<<endl;
   return output;
}

#endif /* MAT_H_ */
