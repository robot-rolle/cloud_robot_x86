#include <Grid_PathMap.h>
#include <vector>
/**
** @name:          grid_pathmap    
** @brief:          栅格地图 获取路径 栅格下降   
** @param:             
** @date:              2024-09-06
** @version:           V0.0
---------------------------------------*/

namespace yd_global_planner
{
    /**
    ** @name:              GetPath
    ** @brief:             
    ** @param:             float* potential,  // 代价值 value = potential【index】
                            double start_x,    // 
                            double start_y,
                            double end_x,     
                            double end_y,
    ** @date:              2024-09-06
    ** @version:           V0.0
    ---------------------------------------*/
   bool  grid_pathmap::GetPath(  float* potential,  // 代价值 value = potential【index】
                            double start_x,    // 
                            double start_y,
                            double end_x,     
                            double end_y,
                            std::vector<std::pair<float, float> > &path)
    {
        std::pair<float, float> BufferP_;
        BufferP_.first  =  end_x;
        BufferP_.second =  end_y;
        int Index_Start= GetIndex(start_x, start_y);        //

        path.push_back(BufferP_);
        int FailedCount = xs_ * ys_ *4;
        //循环找到每个点最低点代价存入进去依次往后推得到误差值
        while(GetIndex(BufferP_.first,BufferP_.second)!=Index_Start)
        {
            int  index_x=0, index_y=0;
            int  min_x=0,min_y=0;
            float InitCost = 1e10;
            int  IndexB_;
            for(int i = -1; i<=1 ; i++) //行列  x 行  
            {
                //不能动i 容易死循环
                for(int j = -1; j<=1; j++)//        //数据存在index 溢出情况未进行数组下标溢出
                {
                    if((i==0)&&(j==0))
                        continue;
                    // index_x = BufferP_.first + i;
                    // index_y = BufferP_.second + j;
                    //八领域 判断index只需要-1 0 +1
                    index_x = BufferP_.first+i>= xs_? BufferP_.first+i-1:\  
                              BufferP_.first+i<0    ? BufferP_.first+i+1:\
                              BufferP_.first+ i ;
                    index_y = BufferP_.second+j>= ys_? BufferP_.second+j-1:\
                              BufferP_.second+j<0    ? BufferP_.second+j+1:\
                              BufferP_.second+j;

                    IndexB_ = GetIndex(index_x,index_y);
                    if(potential[IndexB_] < InitCost)           //存入最低数据
                    {
                        InitCost = potential[IndexB_];
                        min_x = index_x;
                        min_y = index_y;   
                    }
                }
            }
            if(((min_x==0) &&( min_y==0))||((min_x==end_x)&&(min_y==end_y)))//返回原点这个点不应该为0 或者 又回到了 
            {
                return false;
            }
            BufferP_.first  = min_x;
            BufferP_.second = min_y;
            path.push_back(BufferP_);

            if(FailedCount-- < 1)             //遍历四次 找不到目标点直接判定失败
            {
                return false;  //can not find the path
            }

        }
        return true;
    }
}