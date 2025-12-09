/*
 * @Author: rogue-wave zhangjingjie@zju.edu.cn
 * @Date: 2025-11-23 22:33:37
 * @LastEditors: rogue-wave zhangjingjie@zju.edu.cn
 * @LastEditTime: 2025-11-23 22:47:25
 * @FilePath: \dipan_quanxiang\Tasks\pid.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef PID_HPP
#define PID_HPP
namespace pid
{
  class Pid{
  public:
  Pid(float kp,float ki,float kd,float max,float min)
  {
    this->kp=kp;
    this->ki=ki;
    this->kd=kd;
    this->max_=max;
    this->min_=min;
  }
  void set_error(float error){
    this->error_=error;
  }
  float calc(void);
  private:
  float kp = 0,ki = 0,kd = 0,max_ = 0,min_ = 0;
  float error_ = 0;
  float prev_error_ = 0;
  float integral_ = 0;
  float derivative_ = 0;

  };
}









#endif // PID_HPP