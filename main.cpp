#include "processing.h"
#include "robot.h"
#include "server.h"

int sc_main(int argc, char* argv[]) 
{
  sc_clock t_clock("clock", 10, SC_MS);
  sc_signal<bool> r_to_p_r1_flag, r_to_p_r2_flag, r_to_p_r3_flag, r_to_p_r4_flag;
  sc_signal<sc_uint<16> > r_to_p_data_r1, r_to_p_data_r2;
  sc_signal<sc_uint<16> > r_to_p_data_r3, r_to_p_data_r4;
  sc_signal<bool> r_to_p_ack_r1, r_to_p_ack_r2, r_to_p_ack_r3, r_to_p_ack_r4;
  
  sc_signal<bool> p_to_r_flag1, p_to_r_flag2;
  sc_signal<bool> p_to_r_flag3, p_to_r_flag4; 
  sc_signal<sc_uint<16> > p_to_r_data1, p_to_r_data2;
  sc_signal<sc_uint<16> > p_to_r_data3, p_to_r_data4;
  sc_signal<bool> p_to_r_ack1, p_to_r_ack2;
  sc_signal<bool> p_to_r_ack3, p_to_r_ack4;
  
  sc_signal<bool> r_to_s_flag1, r_to_s_flag2;
  sc_signal<bool> r_to_s_flag3, r_to_s_flag4;
  sc_signal<bool> r_to_s_ack1, r_to_s_ack2, r_to_s_ack3, r_to_s_ack4;
  sc_signal<sc_uint<16> > r_to_s_data1, r_to_s_data2, r_to_s_data3, r_to_s_data4;
  
  sc_signal<bool> s_to_r_flag_to_r1, s_to_r_flag_to_r2;
  sc_signal<bool> s_to_r_flag_to_r3, s_to_r_flag_to_r4;
  sc_signal<bool> s_to_r_ack_to_r1, s_to_r_ack_to_r2;
  sc_signal<bool> s_to_r_ack_to_r3, s_to_r_ack_to_r4;
  sc_signal<sc_uint<16> > s_to_r_data_to_r1, s_to_r_data_to_r2;
  sc_signal<sc_uint<16> > s_to_r_data_to_r3, s_to_r_data_to_r4;
  sc_signal<bool> r1_sent, r2_sent, r3_sent, r4_sent;
  
  sc_signal<int> fifo_r1[23], fifo_r2[23], fifo_r3[23], fifo_r4[23];
  
  sc_signal<double> robot1_x, robot2_x, robot3_x, robot4_x;
  sc_signal<double> robot1_y, robot2_y, robot3_y, robot4_y;
  
  sc_signal<bool> r1_speed, r2_speed, r3_speed, r4_speed;
  
  sc_trace_file* speed = sc_create_vcd_trace_file("speed_file");
  
  /**Connect Modules**/
  processing p1("p1", speed);
  p1.clock(t_clock); 
  p1.r1_flag(r_to_p_r1_flag);
  p1.r2_flag(r_to_p_r2_flag);
  p1.r3_flag(r_to_p_r3_flag);
  p1.r4_flag(r_to_p_r4_flag);
  p1.data_r1(r_to_p_data_r1);
  p1.data_r2(r_to_p_data_r2);
  p1.data_r3(r_to_p_data_r3);
  p1.data_r4(r_to_p_data_r4);
  p1.ack_r1(r_to_p_ack_r1);
  p1.ack_r2(r_to_p_ack_r2);
  p1.ack_r3(r_to_p_ack_r3);
  p1.ack_r4(r_to_p_ack_r4);
  p1.output_flag1(p_to_r_flag1);
  p1.output_flag2(p_to_r_flag2);
  p1.output_flag3(p_to_r_flag3);
  p1.output_flag4(p_to_r_flag4); 
  p1.output_data1(p_to_r_data1);
  p1.output_data2(p_to_r_data2);
  p1.output_data3(p_to_r_data3); 
  p1.output_data4(p_to_r_data4);
  p1.output_ack1(p_to_r_ack1);
  p1.output_ack2(p_to_r_ack2);
  p1.output_ack3(p_to_r_ack3);
  p1.output_ack4(p_to_r_ack4);
  for (int i = 0; i < 23; i++)
  {
    p1.fifo_r1[i](fifo_r1[i]);
    p1.fifo_r2[i](fifo_r2[i]);
    p1.fifo_r3[i](fifo_r3[i]);
    p1.fifo_r4[i](fifo_r4[i]);
  }
  p1.r1_sent(r1_sent);
  p1.r2_sent(r2_sent);
  p1.r3_sent(r3_sent);
  p1.r4_sent(r4_sent);
  p1.robot1_x(robot1_x);
  p1.robot2_x(robot2_x);
  p1.robot3_x(robot3_x);
  p1.robot4_x(robot4_x);
  p1.robot1_y(robot1_y);
  p1.robot2_y(robot2_y);
  p1.robot3_y(robot3_y);
  p1.robot4_y(robot4_y);
  p1.r1_speed(r1_speed);
  p1.r2_speed(r2_speed);
  p1.r3_speed(r3_speed);
  p1.r4_speed(r4_speed);
  
  robot r1("r1", 1);
  r1.clock(t_clock); 
  r1.p_flag(p_to_r_flag1);
  r1.p_ack(p_to_r_ack1);
  r1.p_data(p_to_r_data1);
  r1.flag_to_p(r_to_p_r1_flag);
  r1.ack_to_p(r_to_p_ack_r1);
  r1.data_to_p(r_to_p_data_r1);
  r1.flag_to_s(r_to_s_flag1);
  r1.ack_to_s(r_to_s_ack1);
  r1.data_to_s(r_to_s_data1);
  r1.s_flag(s_to_r_flag_to_r1);
  r1.s_ack(s_to_r_ack_to_r1);
  r1.s_data(s_to_r_data_to_r1);
    
  
  robot r2("r2", 2);
  r2.clock(t_clock); 
  r2.p_flag(p_to_r_flag2);
  r2.p_ack(p_to_r_ack2);
  r2.p_data(p_to_r_data2);
  r2.flag_to_p(r_to_p_r2_flag);
  r2.ack_to_p(r_to_p_ack_r2);
  r2.data_to_p(r_to_p_data_r2);
  r2.flag_to_s(r_to_s_flag2);
  r2.ack_to_s(r_to_s_ack2);
  r2.data_to_s(r_to_s_data2);
  r2.s_flag(s_to_r_flag_to_r2);
  r2.s_ack(s_to_r_ack_to_r2);
  r2.s_data(s_to_r_data_to_r2);
  
  robot r3("r3", 3);
  r3.clock(t_clock); 
  r3.p_flag(p_to_r_flag3);
  r3.p_ack(p_to_r_ack3);
  r3.p_data(p_to_r_data3);
  r3.flag_to_p(r_to_p_r3_flag);
  r3.ack_to_p(r_to_p_ack_r3);
  r3.data_to_p(r_to_p_data_r3);
  r3.flag_to_s(r_to_s_flag3);
  r3.ack_to_s(r_to_s_ack3);
  r3.data_to_s(r_to_s_data3);
  r3.s_flag(s_to_r_flag_to_r3);
  r3.s_ack(s_to_r_ack_to_r3);
  r3.s_data(s_to_r_data_to_r3);
  
  robot r4("r4", 4);
  r4.clock(t_clock); 
  r4.p_flag(p_to_r_flag4);
  r4.p_ack(p_to_r_ack4);
  r4.p_data(p_to_r_data4);
  r4.flag_to_p(r_to_p_r4_flag);
  r4.ack_to_p(r_to_p_ack_r4);
  r4.data_to_p(r_to_p_data_r4);
  r4.flag_to_s(r_to_s_flag4);
  r4.ack_to_s(r_to_s_ack4);
  r4.data_to_s(r_to_s_data4);
  r4.s_flag(s_to_r_flag_to_r4);
  r4.s_ack(s_to_r_ack_to_r4);
  r4.s_data(s_to_r_data_to_r4);
  
  server s1("s1");
  s1.clock(t_clock); 
  s1.r_flag1(r_to_s_flag1);
  s1.r_flag2(r_to_s_flag2);
  s1.r_flag3(r_to_s_flag3);
  s1.r_flag4(r_to_s_flag4);
  s1.r_ack1(r_to_s_ack1);
  s1.r_ack2(r_to_s_ack2);
  s1.r_ack3(r_to_s_ack3);
  s1.r_ack4(r_to_s_ack4);
  s1.r_data1(r_to_s_data1);
  s1.r_data2(r_to_s_data2);
  s1.r_data3(r_to_s_data3);
  s1.r_data4(r_to_s_data4);
  s1.flag_to_r1(s_to_r_flag_to_r1);
  s1.flag_to_r2(s_to_r_flag_to_r2);
  s1.flag_to_r3(s_to_r_flag_to_r3);
  s1.flag_to_r4(s_to_r_flag_to_r4);
  s1.ack_to_r1(s_to_r_ack_to_r1);
  s1.ack_to_r2(s_to_r_ack_to_r2);
  s1.ack_to_r3(s_to_r_ack_to_r3);
  s1.ack_to_r4(s_to_r_ack_to_r4);
  s1.data_to_r1(s_to_r_data_to_r1);
  s1.data_to_r2(s_to_r_data_to_r2);
  s1.data_to_r3(s_to_r_data_to_r3);
  s1.data_to_r4(s_to_r_data_to_r4);
  for (int i = 0; i < 23; i++)
  {
    s1.fifo_r1[i](fifo_r1[i]);
    s1.fifo_r2[i](fifo_r2[i]);
    s1.fifo_r3[i](fifo_r3[i]);
    s1.fifo_r4[i](fifo_r4[i]);
  }
  s1.r1_sent(r1_sent);
  s1.r2_sent(r2_sent);
  s1.r3_sent(r3_sent);
  s1.r4_sent(r4_sent);
  s1.robot1_x(robot1_x);
  s1.robot2_x(robot2_x);
  s1.robot3_x(robot3_x);
  s1.robot4_x(robot4_x);
  s1.robot1_y(robot1_y);
  s1.robot2_y(robot2_y);
  s1.robot3_y(robot3_y);
  s1.robot4_y(robot4_y);
  s1.r1_speed(r1_speed);
  s1.r2_speed(r2_speed);
  s1.r3_speed(r3_speed);
  s1.r4_speed(r4_speed);
  
  sc_start(25000, SC_MS);
  
  sc_close_vcd_trace_file(speed);
    
  return 0;
}