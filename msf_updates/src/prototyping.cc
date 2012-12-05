
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <assert.h>
#include <cstdlib>

int main(int argc, char** argv)
{

	Eigen::Matrix<double, 10, 1> m;

	m.setRandom();

	std::cout<<"vec "<<m<<std::endl;

	//	EigenMatIterator<double> begin(m.data());
	//	EigenMatIterator<double> end(m.data() + m.SizeAtCompileTime);
	//
	//	EigenMatIterator<double> middle = begin + std::floor((end - begin) / 2);
	//	std::nth_element(begin, middle, end);
	//	double median = *middle;

	double * begin = m.data();
	double * end   = m.data() + m.SizeAtCompileTime;
	double * middle = begin + (int)std::floor((end - begin) / 2);
	std::nth_element(begin, middle, end);
	double median = *middle;


	std::cout<<"median "<<median<<std::endl;


	//test against a std::vector implementation
	const int ntest = 2000;
	std::cout<<std::endl<<"testing against std::vector implementation...";std::cout.flush();
	for(size_t i = 0;i<1000;++i){

		Eigen::Matrix<double, ntest, 1> m;
		std::vector<double> v;
		v.resize(ntest);
		for(int j = 0;j<ntest;++j){
			v[j] = m(j) = (double)rand() / RAND_MAX;
		}

		double * begin = m.data();
		double * end   = m.data() + m.SizeAtCompileTime;
		double * middle = begin + (int)std::floor((end - begin) / 2);
		std::nth_element(begin, middle, end);
		double median = *middle;

		//compare to vector median
		std::vector<double>::iterator v_begin = v.begin();
		std::vector<double>::iterator v_end = v.end();
		std::vector<double>::iterator v_middle = v_begin + std::floor((v_end - v_begin) / 2);
		std::nth_element(v_begin, v_middle, v_end);
		double v_median = *v_middle;

		assert(abs(median - v_median)<std::numeric_limits<double>::epsilon());
	}
	std::cout<<"ok"<<std::endl;


}
