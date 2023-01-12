#ifndef NRI_MISSION_UTIL_HPP_
#define NRI_MISSION_UTIL_HPP_

#include <mutex>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace nri_mission_runner
{ 

bool PruneGoalPath(int map_width, int map_height, double cur_x, double cur_y, double goal_x, double goal_y, double& prunned_goal_x, double& prunned_goal_y)
{
   bool is_final_local_goal = true;
   double del_x = goal_x - cur_x;
   double del_y = goal_y - cur_y;
   double abs_del_x = std::abs(del_x); 
   double abs_del_y = std::abs(del_y);
   
   double safety_margin = 1; 
   
   if((abs_del_x<(map_width/2-safety_margin))&&(abs_del_y<(map_height/2-safety_margin)))
   {
      prunned_goal_x = goal_x;
      prunned_goal_y = goal_y;
      return is_final_local_goal;
   }
   else is_final_local_goal = false;

   if(abs_del_x>=abs_del_y)
   {
      int sign = 0;
      if (del_x >= 0) sign = 1;
      else sign = -1;
       
      prunned_goal_x = sign*map_width/2-safety_margin;
      prunned_goal_y = prunned_goal_x*(del_y/del_x);
   }
   else
   {
      int sign = 0;
      if (del_y >= 0) sign = 1;
      else sign = -1;
      
      prunned_goal_y = map_height/2-safety_margin;
      prunned_goal_x = prunned_goal_y*(del_x/del_y);
   }

   return is_final_local_goal;
}

}// namespace nri_mission_runnner

#endif  // NRI_MISSION_UTIL_HPP_
