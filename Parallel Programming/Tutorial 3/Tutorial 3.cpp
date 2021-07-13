#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <numeric>
#include "Utils.h"


void print_help() {
	std::cerr << "Application usage:" << std::endl;

	std::cerr << "  -p : select platform " << std::endl;
	std::cerr << "  -d : select device" << std::endl;
	std::cerr << "  -l : list all platforms and devices" << std::endl;
	std::cerr << "  -h : print this message" << std::endl;
}

/*Implmentation summary

Implemented kernels: 
	-local mean reduce add kernel
	-local Standard Deviation reduce add kernel
		-squared difference function
	-local Min reduce compare exchange kernel
	-local Max reduce compare exchange kernel

The kernels are edited versions of the reduce_add_3 from tutorial 3. 
These have been changed to allow for floats and to change functionality dependant
on the stat. The Mean and Standard Deviation are both local memory reduce adds, with the 
Standard Deviation using a helper function to map the input array with the squared difference
before summation. The local workgroups are sequentially totalled on the host before final 
operations.

The Min and Max kernels are local memory compare exchange reduce kernels, which work similarly 
to the reduce adds but compare and exchange the minimum or maximum value instead of performing
addition. Similarly to the Mean and Standard Deviation, the local workgroups are sequentially 
compared and exchanged at the end to get the final result.

The file read in loops through the document getting everything in the temperature column
as a collection of chars, collates them into a string and converts the string to a float.
The float is put into a predefined vector ready for use.

Commented below the file read is a sequential stats implementation for testing purposes.
The Min and Max are the same as the parallel implementation, but the total of the short 
file reduce add is off by one and the larger file off by 69, even though the size of the array is the same. This means 
the Mean and Standard Deviation results are slightly different.
*/

int main(int argc, char **argv) {
	//Part 1 - handle command line options such as device selection, verbosity, etc.
	int platform_id = 0;
	int device_id = 0;

	for (int i = 1; i < argc; i++)	{
		if ((strcmp(argv[i], "-p") == 0) && (i < (argc - 1))) { platform_id = atoi(argv[++i]); }
		else if ((strcmp(argv[i], "-d") == 0) && (i < (argc - 1))) { device_id = atoi(argv[++i]); }
		else if (strcmp(argv[i], "-l") == 0) { std::cout << ListPlatformsDevices() << std::endl; }
		else if (strcmp(argv[i], "-h") == 0) { print_help(); return 0;}
	}

	//detect any potential exceptions
	try {
		//Part 2 - host operations
		//2.1 Select computing devices
		cl::Context context = GetContext(platform_id, device_id);

		//display the selected device
		std::cout << "Runinng on " << GetPlatformName(platform_id) << ", " << GetDeviceName(platform_id, device_id) << std::endl;

		//create a queue to which we will push commands for the device
		cl::CommandQueue queue(context,CL_QUEUE_PROFILING_ENABLE);

		//2.2 Load & build the device code
		cl::Program::Sources sources;

		AddSources(sources, "kernels/my_kernels.cl");

		cl::Program program(context, sources);

		//build and debug the kernel code
		try {
			program.build();
		}
		catch (const cl::Error& err) {
			std::cout << "Build Status: " << program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(context.getInfo<CL_CONTEXT_DEVICES>()[0]) << std::endl;
			std::cout << "Build Options:\t" << program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(context.getInfo<CL_CONTEXT_DEVICES>()[0]) << std::endl;
			std::cout << "Build Log:\t " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(context.getInfo<CL_CONTEXT_DEVICES>()[0]) << std::endl;
			throw err;
		}
		//datatype declaration used to switch between int and float
		typedef float mytype;

		//Variable declarations
		string temp;
		string line;
		vector<mytype> input;

		//Opens file ready for manipulation
		ifstream file;
		file.open("temp_lincolnshire.txt");

		//Loops through each line of file and gets number from 5th column
		while (getline(file, line))
		{
			int i = 0;
			string str;
			for (char x : line)// Loops through each character of the line
			{
				if (i == 5)
				{
					str = str + x;//Creates a string from chars
					//cout << str; 
				}
				else if (x == ' ')
				{
					i += 1; //Counts column position
				}

			}
			//Converts to float and pushes to vector
			mytype num = stof(str);
			input.push_back(num);
		}

		////Sequential testing
		//float test_t = 0;
		//float test_min = 9999;
		//float test_max = -9999;
		//for (int i = 0; i < size(input);i++)
		//{
		//	if (input[i] < test_min)
		//	{
		//		test_min = input[i];
		//	}
		//	if (input[i] > test_max)
		//	{
		//		test_max = input[i];
		//	}
		//	test_t += input[i];
		//	//std::cout << typeid(tem[0]).name() << std::endl;
		//}
		//std::cout << "Min = " << test_min << std::endl;
		//std::cout << "Max = " << test_max << std::endl;
		//float test_mean = test_t / size(input);
		//std::cout << "Mean = " << test_mean << std::endl;

		//float test_total = 0;
		//for (int i = 0; i < size(input); i++)
		//{
		//	float diff = input[i] - test_mean;
		//	test_total += pow(diff, 2);
		//}
		//float test_var = test_total / size(input);
		//float test_SD = sqrt(test_var);

		//std::cout << "Total = " << test_t << std::endl;
		//std::cout << "Var = " << test_var << std::endl;
		//std::cout << "Standard Deviation = " << test_SD << std::endl;
		//std::cout << "Size = " << size(input) << std::endl << std::endl;

		//Part 3 - memory allocation

		//Workgroup sizing 
		size_t local_size = 32; //min recommended 32, max recommended 256. Same range for all kernels
		size_t padding_size = input.size() % local_size; 

		//if the input vector is not a multiple of the local_size
		//insert additional neutral elements (0 for addition) so that the total will not be affected
		if (padding_size) {
			//create an extra vector with neutral values
			std::vector<int> input_ext(local_size-padding_size, 0);
			//append that extra vector to our input
			input.insert(input.end(), input_ext.begin(), input_ext.end());
		}

		//initialisation of vector A size variables for buffer input
		size_t input_elements = input.size();//number of input elements
		size_t input_size = input.size()*sizeof(mytype);//size in bytes
		size_t nr_groups = input_elements / local_size;

		//host - output
		//Mean

		//output vector initialisation
		std::vector<mytype> Mean(input_elements);
		size_t output_size = Mean.size()*sizeof(mytype);//size in bytes

		//device - buffers
		cl::Buffer buffer_input(context, CL_MEM_READ_ONLY, input_size);
		cl::Buffer buff_Mean(context, CL_MEM_READ_WRITE, output_size);

		cl::Event prof_event; // Profiling event
		cl::Event buff_input_event;
		//Part 4 - device operations

		//4.1 copy array A to and initialise other arrays on device memory
		queue.enqueueWriteBuffer(buffer_input, CL_TRUE, 0, input_size, &input[0], NULL, &buff_input_event);
		queue.enqueueFillBuffer(buff_Mean, 0, 0, output_size);//zero Mean buffer on device memory

		//4.2 Setup and execute all kernels (i.e. device code)
		cl::Kernel kernel_Mean = cl::Kernel(program, "kernel_mean");
		kernel_Mean.setArg(0, buffer_input);
		kernel_Mean.setArg(1, buff_Mean);
		kernel_Mean.setArg(2, cl::Local(local_size*sizeof(mytype)));//local memory size

		//call all kernels in a sequence
		queue.enqueueNDRangeKernel(kernel_Mean, cl::NullRange, cl::NDRange(input_elements), cl::NDRange(local_size),NULL,&prof_event);
		
		//4.3 Copy the result from device to host
		queue.enqueueReadBuffer(buff_Mean, CL_TRUE, 0, output_size, &Mean[0]);

		//Input array print
		//std::cout << "input = " << input << std::endl;

		//input buffer transfer time time
		auto buff_input_prof = buff_input_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - buff_input_event.getProfilingInfo<CL_PROFILING_COMMAND_START>();
		std::cout << "Input buffer execution time [ns]:" << buff_input_prof << std::endl <<std::endl;

		//summation of the first element in each work group
		mytype sum = 0;
		for (int i = 0; i < size(Mean); i += local_size)
		{
			sum += Mean[i];
			//std::cout << Mean[i] << std::endl;
		}
		//total output
		std::cout << "Total reduce add = " << sum << endl; //total one less than sequential test
		
		//mean calculation and output
		mytype mean = sum / size(input);
		cout.precision(4);
		std::cout << "Mean = " << mean << std::endl;
		
		//profiling of kernel execution time in ns
		auto buff_prof_Mean = prof_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - prof_event.getProfilingInfo<CL_PROFILING_COMMAND_START>();
		std::cout << "Mean kernel execution time [ns]:" << buff_prof_Mean << std::endl << std::endl;

		//--------
		//Standard deviation

		//output vector initialisation
		std::vector<mytype> SD(input_elements);
		output_size = SD.size() * sizeof(mytype);//size in bytes

		//output buffer
		cl::Buffer buf_SD(context, CL_MEM_READ_WRITE, output_size);

		queue.enqueueFillBuffer(buf_SD, 0, 0, output_size);//zero B buffer on device memory
		
		//creates object for kernel_SD and sets its' arguements
		cl::Kernel kernel_SD = cl::Kernel(program, "kernel_SD");
		kernel_SD.setArg(0, buffer_input); //input arr
		kernel_SD.setArg(1, buf_SD); //output size
		kernel_SD.setArg(2, cl::Local(local_size * sizeof(mytype)));//local memory size
		kernel_SD.setArg(3, mean); //float mean  

		//call all kernels in a sequence
		queue.enqueueNDRangeKernel(kernel_SD, cl::NullRange, cl::NDRange(input_elements), cl::NDRange(local_size), NULL, &prof_event);

		//copies size of buf_SD to host
		queue.enqueueReadBuffer(buf_SD, CL_TRUE, 0, output_size, &SD[0]);

		//Sums local groups 
		mytype total = 0;
		for (int i = 0; i < size(SD); i += local_size)
		{
			total += SD[i];
			//std::cout << SD[i] << std::endl;
		}

		//Prints total
		//cout.precision(4);
		//printf("Total = %f\n", total);

		//calculates var and outputs
		mytype var = total / size(input);
		std::cout << "Var = " << var << std::endl;
		
		//square roots var and outputs Standard Deviation
		mytype StanDev = sqrt(var);
		cout.precision(4);
		std::cout << "SD = " << StanDev << std::endl;

		//profiling of kernel execution time in ns
		auto buff_prof_SD = prof_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - prof_event.getProfilingInfo<CL_PROFILING_COMMAND_START>();
		std::cout << "SD kernel execution time [ns]:" << buff_prof_SD << std::endl << std::endl;

		
		//------
		//Minimum

		//output vector initialisation
		std::vector<mytype> Min(input_elements);
		output_size = Min.size() * sizeof(mytype);//size in bytes

		//output buffer
		cl::Buffer buf_Min(context, CL_MEM_READ_WRITE, output_size);

		//copy size of buf_min to host
		queue.enqueueFillBuffer(buf_Min, 0, 0, output_size);

		//creates object for kernel_SD and sets its' arguements
		cl::Kernel kernel_Min = cl::Kernel(program, "kernel_min");
		kernel_Min.setArg(0, buffer_input); //input arr
		kernel_Min.setArg(1, buf_Min); //output size
		kernel_Min.setArg(2, cl::Local(local_size * sizeof(mytype)));//local memory size

		//call all kernels in a sequence
		queue.enqueueNDRangeKernel(kernel_Min, cl::NullRange, cl::NDRange(input_elements), cl::NDRange(local_size), NULL, &prof_event);

		//copies size of buf_min to host
		queue.enqueueReadBuffer(buf_Min, CL_TRUE, 0, output_size, &Min[0]);

		//finds minimum of the first element in each kernel_min workgroups
		float min = 999999;
		for (int i = 0; i < size(Min); i += local_size)
		{
			if (Min[i] < min)
			{
				min = Min[i];
			}
			//std::cout << Min[i] << std::endl;
		}

		//minimum number from input array output
		std::cout << "Min = " << min << std::endl;

		//profiling of kernel execution time in ns
		auto buff_prof_Min = prof_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - prof_event.getProfilingInfo<CL_PROFILING_COMMAND_START>();
		std::cout << "Min kernel execution time [ns]:" << buff_prof_Min << std::endl << std::endl;

		//-------
		//Maximum

		//output vector initialisation
		std::vector<mytype> Max(input_elements);
		output_size = Max.size() * sizeof(mytype);//size in bytes

		//output buffer
		cl::Buffer buf_Max(context, CL_MEM_READ_WRITE, output_size);

		//copy size of buf_max to host
		queue.enqueueFillBuffer(buf_Max, 0, 0, output_size);//zero B buffer on device memory

		//creates object for kernel_SD and sets its' arguements
		cl::Kernel kernel_Max = cl::Kernel(program, "kernel_max");
		kernel_Max.setArg(0, buffer_input); //input arr
		kernel_Max.setArg(1, buf_Max); //output size
		kernel_Max.setArg(2, cl::Local(local_size * sizeof(mytype)));//local memory size

		//call all kernels in a sequence
		queue.enqueueNDRangeKernel(kernel_Max, cl::NullRange, cl::NDRange(input_elements), cl::NDRange(local_size), NULL, &prof_event);

		//copies size of buf_max to host
		queue.enqueueReadBuffer(buf_Max, CL_TRUE, 0, output_size, &Max[0]);

		//finds maximum of the first element in each kernel_max workgroups
		float max = -999999;
		for (int i = 0; i < size(Max); i += local_size)
		{
			if (Max[i] > max)
			{
				max = Max[i];
			}
			//std::cout << Min[i] << std::endl;
		}

		//maximum number from input array output
		std::cout << "Max = " << max << std::endl;

		//profiling of kernel execution time in ns
		auto buff_prof_Max = prof_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - prof_event.getProfilingInfo<CL_PROFILING_COMMAND_START>();
		std::cout << "Max kernel execution time [ns]:" << buff_prof_Max << std::endl << std::endl;

		//Total kernel execution time
		auto total_buff_prof = buff_prof_Max + buff_prof_Min + buff_prof_SD + buff_prof_Mean;
		std::cout << "Total kernel execution time [ns]: " << total_buff_prof << endl; ;

		//outputs  max and min split of work items into workgroups
		cl::Device device = context.getInfo<CL_CONTEXT_DEVICES>()[0]; // get device
		cerr << kernel_SD.getWorkGroupInfo<CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(device) << endl; // get info
		cerr << kernel_SD.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(device) << endl;
	}
	catch (cl::Error err) {
		std::cerr << "ERROR: " << err.what() << ", " << getErrorString(err.err()) << std::endl;
	}

	return 0;
}	// Local reduce add can be made more efficient by using atmoic add rather than adding reduce add results on the host 