#pragma once
#include "tiny_obj_loader.h"
#include <string>
#include <iostream>
#include <chrono>
#include "opencv2/opencv.hpp"
#include <ctime>
#include <cstdlib>
#include <stdlib.h>
#include <windows.h>

using namespace std;
using namespace chrono;

//#define TINYOBJLOADER_IMPLEMENTATION

const double Pi = 3.14159265359;
const double Eps = 1e-4;
const int RandMod = 10000;
const double Inf = 1e10;
const int maxBBoxFaceNum = 500;

struct Vec { //三维向量数据结构，这一部分主要都来自smallpt: Global Illumination in 99 lines of C++
	double x, y, z;
	Vec(double x, double y, double z) { this->x = x; this->y = y; this->z = z; }
	Vec() : Vec(0, 0, 0) {}
	void toString() { std::cout << "Vec (" << x << ", " << y << ", " << z << ")" << endl; }
	Vec operator+(const Vec &b) const { return Vec(x + b.x, y + b.y, z + b.z); } // 向量相加
	Vec operator-(const Vec &b) const { return Vec(x - b.x, y - b.y, z - b.z); } //向量相减
	Vec operator*(double b) const { return Vec(x*b, y*b, z*b); }//向量乘以常数
	Vec mult(const Vec &b) const { return Vec(x*b.x, y*b.y, z*b.z); }//向量各个坐标相乘（主要用于颜色的叠加）
	Vec& norm() { return *this = *this * (1 / sqrt(x*x + y * y + z * z)); } //向量单位化
	double dot(const Vec &b) const { return x * b.x + y * b.y + z * b.z; }//向量点积
	Vec operator%(Vec&b) { return Vec(y*b.z - z * b.y, z*b.x - x * b.z, x*b.y - y * b.x); } //向量叉积
	double maxCoor() { return x > y ? (x > z ? x : z) : (y > z ? y : z); }//返回最大和最小坐标
	double minCoor() { return x < y ? (x < z ? x : z) : (y < z ? y : z); }
	bool equal(const Vec &b) { return fabs(x - b.x) < Eps * 100 && fabs(y - b.y) < Eps * 100 && fabs(z - b.z) < Eps * 100; }
};

struct Ray { //光线数据结构，由原点o和方向d（单位向量）组成
	Vec o, d;
	Ray(Vec _o, Vec _d) : o(_o), d(_d) { }
};

struct Triangle {//三角面片数据结构
	Vec a, b, c, n; //三个顶点和法向量
	Vec ac, cb, ba; //三条边
	Triangle(Vec _a, Vec _b, Vec _c) : a(_a), b(_b), c(_c) {
		ac = c - a;
		cb = b - c;
		ba = a - b;
		Vec ab = ba * (-1);
		n = (ab%ac).norm();
	}
	void toString() {
		std::cout << "Triangle {" << endl;
		a.toString();
		b.toString();
		c.toString();
		n.toString();
		std::cout << "}" << endl;
	}
	double intersect(const Ray &r) { //运行1亿次耗时4.58秒
		double cosTheta = n.dot(r.d);
		if (fabs(cosTheta) < Eps) { return 0; } //除去与光线平行的面
		double t = (n.dot(a) - n.dot(r.o)) / cosTheta;
		if (t < Eps) { return 0; } //除去离光线原点太近的面和在光线原点后面的面
		Vec ip = r.o + r.d * t;
		Vec aip = ip - a;
		Vec bip = ip - b;
		Vec cip = ip - c;
		if ((aip%ac).dot(n) < 0 || (bip%ba).dot(n) < 0 || (cip%cb).dot(n) < 0) { //判断交点是否在三角形内侧
			//(aip%ac).toString();
			//(bip%ba).toString();
			//(cip%cb).toString();
			//cout << "c" << endl;
			return 0;
		}
		return t; //如果相交，返回距离，否则返回0
	}
	double intersect2(const Ray &r) { //矩阵运算求法，运行1亿次耗时6.28秒, 比向量运算更慢，可能是实现的问题
		double xab = a.x - b.x;
		double xac = a.x - c.x;
		double xae = a.x - r.o.x;
		double yab = a.y - b.y;
		double yac = a.y - c.y;
		double yae = a.y - r.o.y;
		double zab = a.z - b.z;
		double zac = a.z - c.z;
		double zae = a.z - r.o.z;
		double xd = r.d.x;
		double yd = r.d.y;
		double zd = r.d.z;
		double A = determinant(xab, xac, xd, yab, yac, yd, zab, zac, zd);
		if (fabs(A) < Eps) { return 0; }
		double beta = determinant(xae, xac, xd, yae, yac, yd, zae, zac, zd) / A;
		if (beta > 1 || beta < 0) { return 0; }
		double gama = determinant(xab, xae, xd, yab, yae, yd, zab, zae, zd) / A;
		if (gama > 1 || gama < 0) { return 0; }
		double t = determinant(xab, xac, xae, yab, yac, yae, zab, zac, zae) / A;
		return t < Eps ? 0 : t;
	}
	inline double determinant(double a1, double a2, double a3, double b1, double b2, double b3, double c1, double c2, double c3) { //行列式计算
		return a1 * b2 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - a3 * b2 * c1 - a2 * b1 * c3 - a1 * b3 * c2;
	}
};

inline double max3(double x, double y, double z) { return x > y ? (x > z ? x : z) : (y > z ? y : z); }
inline double min3(double x, double y, double z) { return x < y ? (x < z ? x : z) : (y < z ? y : z); }

struct BBox {
	double minX;
	double minY;
	double minZ;
	double maxX;
	double maxY;
	double maxZ;
	BBox(double _minX, double _minY, double _minZ, double _maxX, double _maxY, double _maxZ)
		: minX(_minX), minY(_minY), minZ(_minZ), maxX(_maxX), maxY(_maxY), maxZ(_maxZ) {}
	bool intersect(const Ray &r) {
		double tx1 = (minX - r.o.x) / r.d.x;
		double tx2 = (maxX - r.o.x) / r.d.x;
		double ty1 = (minY - r.o.y) / r.d.y;
		double ty2 = (maxY - r.o.y) / r.d.y;
		double tz1 = (minZ - r.o.z) / r.d.z;
		double tz2 = (maxZ - r.o.z) / r.d.z;
		if (tx1 > tx2) swap(tx1, tx2);
		if (ty1 > ty2) swap(ty1, ty2);
		if (tz1 > tz2) swap(tz1, tz2);
		double tNear = max3(tx1, ty1, tz1);
		double tFar = min3(tx2, ty2, tz2);
		return tFar > 0 && tNear <= tFar + Eps;
	}
	inline bool inside(const Vec &v) {
		return v.x >= minX - Eps && v.x <= maxX  + Eps && v.y >= minY - Eps && v.y <= maxY + Eps && v.z >= minZ - Eps && v.z <= maxZ + Eps;
	}
	inline bool intersect(Triangle &t) {
		Vec A(minX, minY, minZ);
		Vec B(maxX, minY, minZ);
		Vec C(maxX, maxY, minZ);
		Vec D(minX, maxY, minZ);
		Vec E(minX, minY, maxZ);
		Vec F(maxX, minY, maxZ);
		Vec G(maxX, maxY, maxZ);
		Vec H(minX, maxY, maxZ);
		vector<Vec> apexes{ A, B, C, D, E, F, G, H };
		for (int i = 0; i < 8; i++) {
			for (int j = i + 1; j < 8; j++) {
				Ray pos(apexes[i], (apexes[j] - apexes[i]).norm());
				//pos.d.toString();
				Ray neg(apexes[j], (apexes[i] - apexes[j]).norm());
				//neg.d.toString();
				if ((t.intersect(pos) != 0) && (t.intersect(neg) != 0)) {
					return true;
				}
			}
		}
		return inside(t.a) || inside(t.b) || inside(t.c);
	}
	void toString() {
		cout << "(" << minX << ", " << minY << ", " << minZ << ", " << maxX << ", " << maxY << ", " << maxZ << ")" << endl;
	}
};

void saveToTXT(vector<vector<Vec>> &frameBuffer, const string &fileName, int k);

int loadFromTXT(vector<vector<Vec>> &frameBuffer, const string &fileName);

inline bool nearestIntersect(const Ray &r, double &t, int &shapeID, int &faceID, 
	vector<BBox> &bBoxes, vector<vector<Triangle>> &faces);

Vec rayTrace(Ray &r, vector<BBox> &bBoxes, vector<vector<Triangle>> &faces, 
	vector<vector<int>> &materialIDs, std::vector<tinyobj::material_t> &materials, int depth, bool inObject);

bool loadObj(const char* fileDir, const char* baseDir, bool triangulate, tinyobj::attrib_t &attrib,
	std::vector<tinyobj::shape_t> &shapes, std::vector<tinyobj::material_t> &materials);

static void PrintInfo(const tinyobj::attrib_t& attrib,
	const std::vector<tinyobj::shape_t>& shapes,
	const std::vector<tinyobj::material_t>& materials);

static void printInfoTest(const tinyobj::attrib_t& attrib,
	const std::vector<tinyobj::shape_t>& shapes,
	const std::vector<tinyobj::material_t>& materials);

void subBBox(BBox &bBox, vector<Triangle> &fatherFaces, vector<int> &fatherMaterialIDs, 
	vector<BBox> &bBoxes, vector<vector<Triangle>> &faces, vector<vector<int>> &materialIDs);