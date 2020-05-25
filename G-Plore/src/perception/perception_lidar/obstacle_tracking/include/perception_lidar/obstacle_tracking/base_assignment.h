#ifndef _BASE_ASSIGNMENT_
#define _BASE_ASSIGNMENT_

#include <vector>

namespace perception
{

    template<typename T1, typename T2>
    class Base_Assignment
    {
        public:
        // constructor
        Base_Assignment(T1 cost_threshold)
        :
        cost_threshold_(cost_threshold)
        {

        };
        // destructor
        virtual ~Base_Assignment(){ };
        // function
        virtual T1 Solve(std::vector<std::vector<T1> >& DistMatrix, std::vector<T2>& Assignment) { };
        protected:
        T1 cost_threshold_;
    };

}

#endif