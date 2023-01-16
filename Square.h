#pragma once

#include <vector>
#include <cassert>

#include "icmMath.h"


class RecSize
{
public:
	int height;
	int width;

	RecSize(int h, int w)
	{
		assert(h > 0 && w > 0);
		height = h;
		width = w;
	}
};


struct OBB
{
	double min;
	double max;

	OBB(std::vector<Point2D> poly, Vector2D<double> dir);
	OBB();
	OBB(const OBB& obb);

	bool intersect_along_axis(const OBB& oth);

};


// 四角形の衝突判定を取り扱うクラス
class Square
{
protected:
	std::vector<Point2D> vertices;

public:
	Square();
	Square(Point2D p1, Point2D p2, Point2D p3, Point2D p4);
	Square(std::vector<Point2D> v);
	Square(const Square& sq);

	std::vector<Point2D> get_vertices() const;
	Point2D get_center() const;
	double get_radius() const;

	bool intersect(Square other);

	std::vector<OBB> makeOBB(std::vector<Vector2D<double>> axis);
	std::vector<Vector2D<double>> get_edge_vector() const;
};


// 複数の四角形の衝突を取り扱うクラス
class MultiSquare
{
private:
	std::vector<Square> msqr;

public:
	MultiSquare();
	MultiSquare(Square sq);
	MultiSquare(std::vector<Square> sqs);

	inline Square element(int i) { return msqr[i]; }
	inline int size() { return (int)msqr.size(); }
	void push(Square sq);
	bool intersect(Square other);
};


class Triangulus
{
private:
	std::vector<Point2D> vertices;

public:
	Triangulus()
		:vertices()
	{
		vertices.resize(3);
	}
	Triangulus(Point2D p1, Point2D p2, Point2D p3)
		:vertices()
	{
		vertices.resize(3);
		vertices[0] = p1;
		vertices[1] = p2;
		vertices[2] = p3;
	}

	Triangulus(std::vector<Point2D> v)
		:vertices(v)
	{
	}

	Triangulus(const Triangulus& tri)
		:vertices()
	{
		this->vertices = tri.vertices;
	}

	std::vector<Point2D> get_vertices() const
	{
		return vertices;
	}

//	Point2D get_center() const;
//	double get_radius() const;

	bool intersect(Square other)
	{
		std::vector<Vector2D<double>> ax1 = get_edge_vector();
		std::vector<Vector2D<double>> ax2 = other.get_edge_vector();
		std::vector<Vector2D<double>> axis = { ax1[0], ax1[1], ax1[2], ax2[0], ax2[1] };
		
		std::vector<OBB> me = makeOBB(axis);
		std::vector<OBB> you = other.makeOBB(axis);

		// In judge phase, Not intersect -> true	intersect -> false
		// but for return value, true -> intersect, false -> NOT intersect
		bool judge = false;
		for (int ind = 0; ind < (int)me.size(); ++ind) {
			judge = me[ind].intersect_along_axis(you[ind]);
			if (judge == true)
				return false;
		}
	
		return true;
	}

	bool intersect(MultiSquare others)
	{
		for(int i=0; i<(int)others.size(); ++i){
			if(intersect(others.element(i)))	return true;
		}
		return false;
	}

	std::vector<OBB> makeOBB(std::vector<Vector2D<double>> axis)
	{
		std::vector<OBB> linkobb;
		for (int ind = 0; ind < (int)axis.size(); ++ind) {
			OBB tmp(vertices, axis[ind]);
			linkobb.push_back(tmp);
		}

		return linkobb;
	}

	std::vector<Vector2D<double>> get_edge_vector() const
	{
		std::vector<Vector2D<double>> axis;
		Vector2D<double> tmp(vertices[1].x - vertices[0].x, vertices[1].y - vertices[0].y);
		tmp.normalize();
		
		Vector2D<double> tmp2(vertices[2].x - vertices[1].x, vertices[2].y - vertices[1].y);
		tmp2.normalize();
	
		Vector2D<double> tmp3(vertices[0].x - vertices[2].x, vertices[0].y - vertices[2].y);
		tmp3.normalize();

		axis.push_back(tmp);	
		axis.push_back(tmp2);
		axis.push_back(tmp3);
	
		return axis;
	}
};
