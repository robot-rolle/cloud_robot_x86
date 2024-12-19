#ifndef _YD_GLOBAL_BASE_H_
#define _YD_GLOBAL_BASE_H_

#include <cmath>
#include <vector>
#include <algorithm>
#include <exception>
#include <iostream>
/**
** @name:              
** @brief:             designer logic
** @param:             
** @date:              2024-09-09
** @version:           V0.0
---------------------------------------*/

namespace  yd_global_planner
{
    class BaseTree
    {
    private:
        /* data */
    public:
        BaseTree(/* args */);
        ~BaseTree();
    };
    
    BaseTree::BaseTree(/* args */)
    {
    }
    
    BaseTree::~BaseTree()
    {
    }
    
    class ydplan_exception: public std::exception
    {
        private:
            std::string message;
        public:
            ydplan_exception(const std::string& msg);
            virtual const char* what() const noexcept override;
    };
        
}



#endif
