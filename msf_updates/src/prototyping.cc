
#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>


class A{
public:
	virtual ~A(){};
	virtual void echo(){
		std::cout<<"in A"<<std::endl;
	}
};

template<typename T>
class AA{
public:
	virtual ~AA(){};
	virtual void echoTemplate(){
		std::cout<<"in AA"<<std::endl;
	}
};

class B:public A, public AA<int>{
public:
	virtual ~B(){};
	virtual void echo(){
		std::cout<<"in B"<<std::endl;
	}
};

class C:public A, public AA<long>{
public:
	virtual ~C(){};
	virtual void echo(){
		std::cout<<"in C"<<std::endl;
	}
};

int main(int argc, char** argv)
{
std::vector<boost::shared_ptr<A> > v;

v.push_back(boost::shared_ptr<A>(new B()));
v.push_back(boost::shared_ptr<A>(new C()));


for(size_t i = 0;i<v.size();++i){
	v.at(i)->echo();
}

dynamic_cast<B*>(v.at(0).get())->echoTemplate();

}
