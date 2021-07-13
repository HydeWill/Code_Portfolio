#include "Shape.h"
#include "Movable.h"
#include "Circle.h"
#include "Rectangle.h"
#include "Square.h"

#include <iostream>
#include <string>
#include <vector>
using namespace std;
int main()
{
	string userCommand;
	vector <Shape*> shapes;     // vector full of pointers to created shapes
	vector <string> parameters; // vector that holds parameters given from inputted commands


	while (userCommand.compare("exit") != 0)
	{
		cout << "Enter the command: ";
		getline(cin, userCommand);
		//Gets the inputs of the user 

		char* cstr = new char[userCommand.length() + 1];
		strcpy_s(cstr, userCommand.length() + 1, userCommand.c_str());
		//Creates a C-string array of pointers and assigns each character of the userCommand to the array
		string command;
		if (userCommand != "")//Doesn't allow no input, as string conversion causes exception - Exception Handling
		{
			char space[] = " ";
			char* token;
			char* next = NULL;
			int i = 0;
			token = strtok_s(cstr, space, &next);
			string tok = ("%s", token);

			parameters.push_back(tok);
			//Defines parameters and creates the intial token before looping through array to create tokens
			while (token != NULL)//Exception Handeling
			{
				
				token = strtok_s(NULL, space, &next);
				if (token != NULL)
				{
					string numbers = "0123456789";
					string Token = token;
					if (Token.find_first_not_of(numbers))
					{
						parameters.push_back(token);// populates parameters vector with
					}
					else
					{
						cout << "Error - Parameter inputs need to be a valid number" << endl;
					}
					 
				}
			}
			//Loop seperates charaters about the spaces into tokens,
			//these tokens are then put together to populatethe parameters vector 

			command = parameters[0];
			//Takes the first token put into the pararmeters vector. 
			//This token decides which functionality will be executed
		}
		if (command.compare("addR") == 0) 
		{
			if (parameters.size() == 5)//Exception handling
			{
				int x = stoi(parameters[1].c_str());
				int y = stoi(parameters[2].c_str());
				int h = stoi(parameters[3].c_str());
				int w = stoi(parameters[4].c_str());
				//Converts strings to integers

				Rectangle* r = new Rectangle(x, y, h, w);
				shapes.push_back(r);
				r->toString();
				//Object is created with parameters, object is then pushed into the vector shapes.
				//Object is then outputted using toString function
			}
			else
			{
				cout << "Error - Amount of valid parameters for addR is wrong" << endl
					<< "addR requires 4 valid parameters";
			}
		}
		else if (command.compare("addS") == 0) 
		{
			if (parameters.size() == 4)//Exception handling
			{
				int x = stoi(parameters[1].c_str());
				int y = stoi(parameters[2].c_str());
				int e = stoi(parameters[3].c_str());
				//Converts strings to integers

				Square* s = new Square(x, y, e);
				shapes.push_back(s);
				s->toString();
				//Object is created with parameters, object is then pushed into the vector shapes.
				//Object is then outputted using toString function
			}
			else
			{
				cout << "Error - Amount of valid parameters for addS is wrong" << endl
					<< "addS requires 3 valid parameters";
			}
		}
		else if (command.compare("addC") == 0) 
		{
			if (parameters.size() == 4)//Exception handling
			{
				int x = stoi(parameters[1].c_str());
				int y = stoi(parameters[2].c_str());
				int r = stoi(parameters[3].c_str());
				//Converts strings to integers

				Circle* c = new Circle(x, y, r);
				shapes.push_back(c);
				c->toString();
				//Object is created with parameters, object is then pushed into the vector shapes.
				//Object is then outputted using toString function
			}
			else
			{
				cout << "Error - Amount of valid parameters for addC is wrong" << endl
					<< "addC requires 3 valid parameters";
			}
		}
		else if (command.compare("scale") == 0) 
		{
			if (parameters.size() == 4 && shapes.size() != 0)//Exception Handeling
			{
				int shapeNo = stoi(parameters[1].c_str());
				//Converts string into integer
				if (shapeNo <= shapes.size())//Exception Handeling
				{
					float x = stof(parameters[2].c_str());
					float y = stof(parameters[3].c_str());
					//Converts strings to floats

					Movable* m = dynamic_cast<Movable*>(shapes[shapeNo - 1]);//Creates object using the shape index
					m->scale(x, y);//Calls scales with new object and scales to calculate new points
					shapes[shapeNo - 1]->toString();
					//New points are outputted using toString
				}
				else
				{
					cout << "Error - Shape does not exsist " << endl
						<< "Make sure call a shape that has already been created";
				}
				
			}
			else
			{
				cout << "Error - Invalid amount of arguements or no shapes have been created" << endl
					<< "Make sure to create a shape before using the scale command" << endl
					<< "The scale command requires 3 valid parameters";
			}
		}
		else if (command.compare("move") == 0) 
		{
			if (parameters.size() == 4 && shapes.size() != 0)//Exception Handeling
			{
				int shapeNo = stoi(parameters[1].c_str()); // read from parameters
				if (shapeNo <= shapes.size())//Exception Handeling
				{
					int x = stoi(parameters[2].c_str());
					int y = stoi(parameters[3].c_str());
					//New points parameters are converted from string

					Movable* m = dynamic_cast<Movable*>(shapes[shapeNo - 1]);//Creates object using the shape index
					m->move(x, y);//Calls move with new object and new postions to calculate new points
					shapes[shapeNo - 1]->toString();
					//New points are outputted using toString
				}
				else
				{
					cout << "Error - Shape does not exsist " << endl
						<< "Make sure call a shape that has already been created";
				}
			}
			else
			{
				cout << "Error - Invalid amount of arguements or no shapes have been created" << endl
					<< "Make sure to create a shape before using the move command" << endl
					<< "The scale command requires 3 valid parameters";
			}
		}
		else if (command.compare("exit") != 0)//<--Doesn't show error upon exiting
		{
			cout << "Error - Incorrect command input. " << endl
				<< "Please make sure your intial command is one of these:" << endl
				<< "addR" << endl << "addC" << endl << "addS" << endl << "move" << endl << "scale";
		}
		cout << endl << endl;
		while (!parameters.empty())
		{
			parameters.pop_back();
		}
		//Depopulates parameters vector to be used again
	}


	cout << "Press any key to continue...";
	getchar();
	return 0;
}
