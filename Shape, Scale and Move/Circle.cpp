#include "Circle.h"
Circle::Circle(int X, int Y, int r) 
{
	radius = r; 
	//Assigns variable values
	leftTop = new Point(X, Y);
	//Assigns the leftTop using the x and y
}
float Circle::calculateArea() 
{
	area = pi * (radius * radius);
	//calculates and returns area of the Circle to area member
	return area;
	//returns value to be outputted
}
float Circle::calculatePerimeter() 
{
	perimeter = 2 * pi * radius;
	//Calculates perimeter of the Circle
	return perimeter;
	//returns value to be outputted
}
void Circle::calculatePoints() 
{
	int new_X = leftTop->get_X();
	int new_Y = leftTop->get_Y();
	//Creates new x and y
	new_X = leftTop->get_X() + (2*radius);
	new_Y = leftTop->get_Y() + (2*radius);
	//Adds new values to new x and y
	if (points.empty())//Checks if the points vector is empty or not 
	{
		//Shape creation
		points.push_back(leftTop);
		points.push_back(new Point(new_X, new_Y));
		//Applies the new x and y with old x and y to get new points
		//Pushes leftTop and rightBottom to be outputted
	}
	else
	{
		//Moving shapes
		points[1]->set_X(new_X);
		points[1]->set_Y(new_Y);
		//Replaces the value for rightBottom with new moved value
	}
}
void Circle::move(int m_x, int m_y) 
{
	leftTop->set_X(m_x);
	leftTop->set_Y(m_y);
}
void Circle::scale(float s_x, float s_y)
{
	if (s_x == s_y)
	{
		radius *= s_x;
	}
	else
	{
		cout << "Error - Shape not manipulated " << endl
			<< "Please input the same x and y scale when scaling a circle" << endl << endl;
	}
}
void Circle::toString()
{ 
	cout << "Area = " << calculateArea() <<
		" Perimeter = " << calculatePerimeter() << endl;
	cout << "Circle [r = " << radius << "]" << endl;
	calculatePoints();
	cout << "Points [";
	for (int i = 0; i < points.size(); i++)
	{
		cout << "(" << points[i]->get_X()
			<< "," << points[i]->get_Y() << ")";
	}
	cout << "]" << endl;
}