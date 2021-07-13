#pragma once
#include <iostream>
#include <vector>
#include <string>
#include "Shape.h"
#include "Movable.h"
using namespace std;

class Circle : public Movable, public Shape
{
protected:
	int radius;
	float pi = 3.14159265358979;
public:
	Circle(int X, int Y, int r);
	~Circle() {};
	virtual float calculateArea();
	virtual float calculatePerimeter();
	void calculatePoints();
	void move(int m_x, int m_y);
	void scale(float s_x, float s_y);
	void toString();
};