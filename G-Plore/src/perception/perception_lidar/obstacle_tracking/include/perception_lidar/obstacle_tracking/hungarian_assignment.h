#ifndef _HUNGARIAN_ASSIGNMENT_
#define _HUNGARIAN_ASSIGNMENT_

// c++
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <vector>
#include <iostream>

// base_assignment
#include "perception_lidar/obstacle_tracking/base_assignment.h"

namespace perception
{

    class Hungarian_Assignment : public Base_Assignment<double, int>
    {
        public:
        // constructor
        Hungarian_Assignment(double cost_threshold)
        :
        Base_Assignment<double, int>(cost_threshold)
        {

        };
        // destructor
        ~Hungarian_Assignment(){ };
        // function
        double Solve(std::vector<std::vector<double> >& DistMatrix, std::vector<int>& Assignment);
        protected:
        void assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns);
        void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
        void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
        void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
        void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
        void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
        void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
        void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    };

}



#endif