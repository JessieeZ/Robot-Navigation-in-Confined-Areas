#include "systemc.h"

class server : public sc_module
{
  public:
    /**********INPUTS**********/
    sc_in<bool> clock; 
    sc_in<bool> r_flag1, r_flag2, r_flag3, r_flag4;
    sc_in<bool> r_ack1, r_ack2, r_ack3, r_ack4;
    sc_in<sc_uint<16> > r_data1, r_data2, r_data3, r_data4;
    
    sc_in<double> robot1_x, robot2_x, robot3_x, robot4_x;
    sc_in<double> robot1_y, robot2_y, robot3_y, robot4_y;
    
    /**********OUTPUTS**********/
    sc_out<bool> flag_to_r1, flag_to_r2, flag_to_r3, flag_to_r4;
    sc_out<bool> ack_to_r1, ack_to_r2, ack_to_r3, ack_to_r4;
    sc_out<sc_uint<16> > data_to_r1, data_to_r2, data_to_r3, data_to_r4;
    sc_out<int> fifo_r1[23], fifo_r2[23], fifo_r3[23], fifo_r4[23];
    sc_out<bool> r1_sent, r2_sent, r3_sent, r4_sent;
    sc_out<bool> r1_speed, r2_speed, r3_speed, r4_speed;
    
    /**********CONSTRUCTOR**********/
    SC_HAS_PROCESS(server);
    
    server(sc_module_name name) : sc_module(name)
    {
      // Internal Process
      SC_METHOD(prc_update);
      sensitive << clock.pos();
      
      // Processes: Connects to Robots
      // Receive
      SC_THREAD(prc_rx);
      //sensitive << rx_signal; // not necessary
      
      // Transmit
      SC_THREAD(prc_tx);
      sensitive << tx_signal;
      
    }
    
    private:
      
      /*int map_data[10][9];
      const int* map_ptr;
      int robot_path[6][30];
      const int* path_ptr;*/
      
      int path[4][23] = {  {1, 11, 13, 14, 15, 16, 17, 18, 24, 31, 30, 29, 28, 27, 26, 36, 39, 49, 51, 52, 53, -1},
                           {10, 12, 22, 21, 20, 19, 18, 24, 31, 32, 33, 34, 35, 25, -1},
		                       {51, 49, 39, 36, 26, 27, 28, 29, 30, 31, 32, 37, 45, 46, 47, 48, 38, -1},
                      		 {60, 50, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 36, 26, 23, -1}  
                        };
    
      sc_event tx_signal;
      
      /*** Counters for Requests ***/
      // incremented by prc_rx(), decremented by prc_update();
      int rx_counter = 0;
      // incremented by prc_update(), decremented by prc_tx();
      int tx_counter = 0;
      
      /*** Local Table for Communications ***/
      
      // (Robot Index, Status, Modified)
      // Column 1: Robot Index – all robots are listed
      // Column 2: New Status (Revised by Server)
      // Column 3: Modified (set to 1 is revised by prc_rx(), 
      //           after prc_update(), reset to 0)
      int rx_table[4][3] = {  {1, 5, 0},
                              {2, 5, 0},
                              {3, 5, 0},
                              {4, 5, 0}  };
                              
      // (Robot Index, Status, Modified)   
      // Column 1: Robot Index – all robots are listed
      // Column 2: New Status (Revised by Processing)
      // Column 3: Modified (set to 1 is revised by prc_update(), 
      //           after prc_tx(), reset to 0)
      int tx_table[4][3] = {  {1, 5, 0},
                              {2, 5, 0},
                              {3, 5, 0},
                              {4, 5, 0}  };
      
      // Main_Table for Processing
      // (Robot Index, Status, Modified)
      int main_table[4][3] = {  {1, 5, 0},
                                {2, 5, 0},
                                {3, 5, 0},
                                {4, 5, 0}  };
      
      // Intersection Grid, Order of Robots, Distance of Robot to Intersection, Expected Time needed
      int node_table[6][13] = {  {18, /**/1, 0, -1, -1/**/, /**/6, 7, -1, -1 /**/, 6, 7, -1, -1},
                                 {26, /**/2, 3, 0, -1/**/, /**/2, 2, 5, -1 /**/, 2, 2, 5, -1},
                                 {31, /**/1, 2, 0, -1/**/, /**/2, 5, 2, -1 /**/, 2, 5, 2, -1},
                                 {39, /**/2, 3, 0, -1/**/, /**/2, 6, 2, -1 /**/, 2, 6, 2, -1},
                                 {45, /**/3, 2, -1, -1/**/, /**/3, 3, -1, -1 /**/, 3, 3, -1, -1},
                                 {48, /**/3, 2, -1, -1/**/, /**/2, 3, -1, -1 /**/, 2, 3, -1, -1}
                              };
                              
      int intersection_path[4][6] = { {18, 31, 26, 39, -1},
                                			{18, 31, -1},
                                			{39, 26, 31, 45, 48, -1},
                                			{48, 45, 39, 26, -1}
                                    };
                                    
      int grid[61][9] = {   {1, 0, 2, 16, 18, -1, 11, -1, 2},
                            {2, 2, 4, 16, 18, -1, -1, 1, 3},
                            {3, 4, 6, 16, 18, -1,	-1,	2, 4},
                            {4, 6, 8, 16, 18, -1, -1, 3, 5},
                            {5, 8, 10, 16, 18, -1, -1, 4, 6},
                            {6, 10, 12, 16, 18, -1,	-1,	5, 7},
                            {7, 12, 14, 16, 18, -1,	-1,	6, 8},
                            {8, 14, 16, 16, 18, -1,	-1,	7, 9},
                            {9, 16, 18, 16, 18, -1,	-1,	8, 10},
                            {10, 18, 20, 16, 18, -1, 12, 9,	-1},
                            //
                            {11, 0, 2, 14, 16, 1, 13, -1, -1},
                            {12, 18, 20, 14, 16, 10, 22, -1, -1},
                            //
                            {13, 0, 2, 12, 14, 11, 23, -1, 14},
                            {14, 2, 4, 12, 14, -1, -1, 13, 15},
                            {15, 4, 6, 12, 14, -1, -1, 14, 16},
                            {16, 6, 8, 12, 14, -1, -1, 15, 17},
                            {17, 8, 10, 12, 14, -1, -1, 16, 18},
                            {18, 10, 12, 12, 14, -1, 24, 17, 19},
                            {19, 12, 14, 12, 14, -1, -1, 18, 20},
                            {20, 14, 16, 12, 14, -1, -1, 19, 21},
                            {21, 16, 18, 12, 14, -1, -1, 20, 22},
                            {22, 18, 20, 12, 14, 12, 25, 21, -1},
                            //
                            {23, 0, 2, 10, 12, 13, 26, -1, -1},
                            {24, 10, 12, 10, 12, 18, 31, -1, -1},
                            {25, 18, 20, 10, 12, 22, 35, -1, -1},
                            //
                            {26, 0, 2, 8, 10, 23, 36, -1, 27},
                            {27, 2, 4, 8, 10, -1, -1, 26, 28},
                            {28, 4, 6, 8, 10, -1, -1, 27, 29},
                            {29, 6, 8, 8, 10, -1, -1, 28, 30},
                            {30, 8, 10, 8, 10, -1, -1, 29, 31},
                            {31, 10, 12, 8, 10, 24, -1, 30, 32},
                            {32, 12, 14, 8, 10, -1, 37, 31, 33},
                            {33, 14, 16, 8, 10, -1, -1, 32, 34},
                            {34, 16, 18, 8, 10, -1, -1, 33, 35},
                            {35, 18, 20, 8, 10, 25, 38, 34, -1},
                            //
                            {36, 0, 2, 6, 8, 26, 39, -1, -1},
                            {37, 12, 14, 6, 8, 32, 45, -1, -1},
                            {38, 18, 20, 6, 8, 35, 48, -1, -1},
                            //
                            {39, 0, 2, 4, 6, 36, 49, -1, 40},
                            {40, 2, 4, 4, 6, -1, -1, 39, 41},
                            {41, 4, 6, 4, 6, -1, -1, 40, 42},
                            {42, 6, 8, 4, 6, -1, -1, 41, 43},
                            {43, 8, 10, 4, 6, -1, -1, 42, 44},
                            {44, 10, 12, 4, 6, -1, -1, 43, 45},
                            {45, 12, 14, 4, 6, 37, -1, 44, 46},
                            {46, 14, 16, 4, 6, -1, -1, 45, 47},
                            {47, 16, 18, 4, 6, -1, -1, 46, 48},
                            {48, 18, 20, 4, 6, 38, 50, 47, -1},  
                            //
                            {49, 0, 2, 2, 4, 39, 51, -1, -1},
                            {50, 18, 20, 2, 4, 48, 60, -1, -1},
                            //
                            {51, 0, 2, 0, 2, 49, -1, -1, 52},
                            {52, 2, 4, 0, 2, -1, -1, 51, 53},
                            {53, 4, 6, 0, 2, -1, -1, 52, 54},
                            {54, 6, 8, 0, 2, -1, -1, 53, 55},
                            {55, 8, 10, 0, 2, -1, -1, 54, 56},
                            {56, 10, 12, 0, 2, -1, -1, 55, 57},
                            {57, 12, 14, 0, 2, -1, -1, 56, 58},
                            {58, 14, 16, 0, 2, -1, -1, 57, 29},
                            {59, 16, 18, 0, 2, -1, -1, 58, 60},
                            {60, 18, 20, 0, 2, 50, -1, 59, -1},    
                            {-1, 90, 100, 90, 100, -1, -1, -1, -1}  };
     
      // 18, 26, 31, 39, 45, 48
      int r1_crossed[6] = {0, 0, 0, 0, 0, 0};
      int r2_crossed[6] = {0, 0, 0, 0, 0, 0};
      int r3_crossed[6] = {0, 0, 0, 0, 0, 0};
      int r4_crossed[6] = {0, 0, 0, 0, 0, 0};
      
      sc_uint<16> status;
      int count_clock = -1;
      
      void prc_rx()
      {
        ack_to_r1.write(false);
        ack_to_r2.write(false);
        ack_to_r3.write(false);
        ack_to_r4.write(false);
        wait(r_flag1.posedge_event() | r_flag2.posedge_event() |
             r_flag2.posedge_event() | r_flag4.posedge_event() );
        if (r_flag1 == true)
        {
          status = r_data1;
          ack_to_r1.write(true);
          rx_table[0][1] = status;
          rx_table[0][2] = 1;
          wait(SC_ZERO_TIME);
          ack_to_r1.write(false);
          rx_counter++;
        }
        if (r_flag2 == true)
        {
          status = r_data2;
          ack_to_r2.write(true);
          rx_table[1][1] = status;
          rx_table[1][2] = 1;
          wait(SC_ZERO_TIME);
          ack_to_r2.write(false);
          rx_counter++;
        }
        if (r_flag3 == true)
        {
          status = r_data3;
          ack_to_r3.write(true);
          rx_table[2][1] = status;
          rx_table[2][2] = 1;
          wait(SC_ZERO_TIME);
          ack_to_r3.write(false);
          rx_counter++;
        }
        if (r_flag4 == true)
        {
          status = r_data4;
          ack_to_r4.write(true);
          rx_table[3][1] = status;
          rx_table[3][2] = 1;
          wait(SC_ZERO_TIME);
          ack_to_r4.write(false);
          rx_counter++;
        }
      }
      
      void prc_update()
      {
        int update_status;
        while (rx_counter > 0)
        {
          for (int i = 0; i < 4; i++)
          {
            if (rx_table[i][2] == 1)
            {
              if (main_table[i][1] != 5)
              {
                //Status 0 -> STOPPED1 (stopped due to obstacles)
                if (rx_table[i][1] == 0)
                {
                  break;
                }
                //Status 1 -> RESTART (ask server to started after obstacle clearance)
                if (rx_table[i][1] == 1)
                {
                  tx_table[i][1] = 9;
                  tx_table[i][2] = 1;
                  tx_counter++;
                }
                //Status 2 -> CROSSING (just before the boundary crossing)
                if (rx_table[i][1] == 2)
                {
                  tx_table[i][1] = 5;
                  tx_table[i][2] = 1;
                  tx_counter++;
                }
                //Status 3 -> STOPPED2 (stopped at the boundary due to no Ack from Server)
                if (rx_table[i][1] == 3)
                {
                  main_table[i][1] = 7;
                }
                //Status 4 -> CROSSED (this will tell server to update server’s table)
                if (rx_table[i][1] == 4)
                {
                  break;
                }
              }
              rx_counter--;
              rx_table[i][2] = 0;
            }
          }
        }
        for (int i = 0; i < 4; i++)
        {
          if (i == 0 && robot1_x != 0 && robot1_y != 0)
          {
            for (int j = 0; j < 60; j++)
            {
              if ((robot1_x > grid[i][1]) && (robot1_x < grid[i][2]) && 
                  (robot1_y > grid[i][3]) && (robot1_y < grid[i][4]))
              { 
                if (grid[j][0] == 24)
                {
                  r1_crossed[0] = 1;
                }
                if (grid[j][0] == 30)
                {
                  r1_crossed[2] = 1;
                }
                if (grid[j][0] == 36)
                {
                  r1_crossed[1] = 1;
                }
                if (grid[j][0] == 49)
                {
                  r1_crossed[3] = 1;
                }
              }
            }
          }
          if (i == 1)
          {
            for (int j = 0; j < 60; j++)
            {
              if ((robot2_x > grid[i][1]) && (robot2_x < grid[i][2]) && 
                  (robot2_y > grid[i][3]) && (robot2_y < grid[i][4]))
              {
                if (grid[j][0] == 24)
                {
                  r2_crossed[0] = 1;
                }
                if (grid[j][0] == 32)
                {
                  r2_crossed[2] = 1;
                }
              }
            }
          }
          if (i == 2)
          {
            for (int j = 0; j < 60; j++)
            {
              if ((robot3_x > grid[i][1]) && (robot3_x < grid[i][2]) && 
                  (robot3_y > grid[i][3]) && (robot3_y < grid[i][4]))
              {
                if (grid[j][0] == 36)
                {
                  r3_crossed[3] = 1;
                }
                if (grid[j][0] == 27)
                {
                  r3_crossed[1] = 1;
                }
                if (grid[j][0] == 32)
                {
                  r3_crossed[2] = 1;
                }
                if (grid[j][0] == 46)
                {
                  r3_crossed[4] = 1;
                }
                if (grid[j][0] == 38)
                {
                  r3_crossed[5] = 1;
                }
              }
            }
          }
          if (i == 3)
          {
            for (int j = 0; j < 60; j++)
            {
              if ((robot4_x > grid[i][1]) && (robot4_x < grid[i][2]) && 
                  (robot4_y > grid[i][3]) && (robot4_y < grid[i][4]))
              {
                if (grid[j][0] == 47)
                {
                  r4_crossed[5] = 1;
                }
                if (grid[j][0] == 44)
                {
                  r4_crossed[4] = 1;
                }
                if (grid[j][0] == 36)
                {
                  r4_crossed[3] = 1;
                }
                if (grid[j][0] == 23)
                {
                  r4_crossed[1] = 1;
                }
              }
            }
          }
        }
        for (int i = 0; i < 4; i++)
        {
          if (i == 0)
          {
            r1_speed = 1;
            for (int j = 0; j < 60; j++)
            {
              if ((robot1_x > grid[i][1]) && (robot1_x < grid[i][2]) && 
                  (robot1_y > grid[i][3]) && (robot1_y < grid[i][4]))
              {
                  if (grid[j][0] == 17 && r2_crossed[0] == 0)
                  {
                    r1_speed = 0;
                  }
                  else if (grid[j][0] == 24 && (r2_crossed[2] == 0) && (r3_crossed[2] == 0))
                  {
                    r1_speed = 0;
                  }
                  else if (grid[j][0] == 27 && (r3_crossed[1] == 0) && (r4_crossed[1] == 0))
                  {
                    r1_speed = 0;
                  }
                  else if (grid[j][0] == 36 && (r3_crossed[3] == 0) && r4_crossed[3] == 0)
                  {
                    r1_speed = 0;
                  }
              }
            }
          }
      
          if (i == 1)
          {
            r2_speed = 1;
          }
          if (i == 2)
          {
            r3_speed = 1;
            for (int j = 0; j < 60; j++)
            {
              if ((robot3_x > grid[i][1]) && (robot3_x < grid[i][2]) && 
                  (robot3_y > grid[i][3]) && (robot3_y < grid[i][4]))
              {
                for (int a = 0; a < 23; a++)
                {
                  if (grid[j][0] == path[2][a])
                  {
                    if (a == 8 && (r2_crossed[2] == 0))
                    {
                      r3_speed = 0;
                    }
                    else if (a == 11 && r4_crossed[4] == 0)
                    {
                      r3_speed = 0;
                    }
                    else if (a = 14 && r4_crossed[5] == 0)
                    {
                      r3_speed = 0;
                    }
                  }
                }
              }
            }
          }
          
          if (i == 3)
          {
            r4_speed = 1;
            for (int j = 0; j < 60; j++)
            {
              if ((robot4_x > grid[i][1]) && (robot4_x < grid[i][2]) && 
                  (robot4_y > grid[i][3]) && (robot4_y < grid[i][4]))
              {
                for (int a = 0; a < 23; a++)
                {
                  if (grid[j][0] == path[3][a])
                  {
                    if (a == 10 && r3_crossed[3] == 0)
                    {
                      r4_speed = 0;
                    }
                    else if (a = 12 && r3_crossed[1] == 0)
                    {
                      r4_speed = 0;
                    }
                  }
                }
              }
            }
          }
        }
        count_clock++;
        if (count_clock == 101)
        {
          cout << "Time: " << 
          sc_time_stamp() << endl;
          cout << "Robot #1 received from server: PATH" << endl; 
          cout << "Robot #1 received from server: SPEED" << endl; 
          cout << endl;          
          send_path(0);
          main_table[0][1] = 6;
          r1_sent.write(true);
        }
        else if (count_clock == 501)
        {
          cout << "Time: " << 
          sc_time_stamp() << endl;
          cout << "Robot #2 received from server: PATH" << endl; 
          cout << "Robot #2 received from server: SPEED" << endl; 
          cout << endl;
          send_path(1);
          main_table[1][1] = 6;
          r2_sent.write(true);
        }
        else if (count_clock == 701)
        {
          cout << "Time: " << 
          sc_time_stamp() << endl;
          cout << "Robot #3 received from server: PATH" << endl; 
          cout << "Robot #3 received from server: SPEED" << endl; 
          cout << endl;
          send_path(2);
          main_table[2][1] = 6;
          r3_sent.write(true);
        }
        else if (count_clock == 201)
        {
          cout << "Time: " << 
          sc_time_stamp() << endl;
          cout << "Robot #4 received from server: PATH" << endl; 
          cout << "Robot #4 received from server: SPEED" << endl;           
          cout << endl;
          send_path(3);
          main_table[3][1] = 6;
          r4_sent.write(true);
        }
        
        if (tx_counter > 0)
        {
          tx_signal.notify();
        }
      }
      
      void prc_tx()
      {
        sc_uint<16> tx_status;
        while (true)
        {
          wait(tx_signal);  // wait for tx_signal.pos()
          while (tx_counter != 0)
          {
            if (main_table[0][2] == 1)
            {
              tx_status = main_table[0][1];
              tx_table[0][1] = tx_status;
              main_table[0][2] = 0;
              flag_to_r1.write(true);  // creating positive edge
              data_to_r1.write(status);
              wait(r_ack1.posedge_event()); // wait for ack from the receiving process
              flag_to_r1.write(false);
              wait(SC_ZERO_TIME);
            }
            if (main_table[1][2] == 1)
            {
              tx_status = main_table[1][1];
              tx_table[1][1] = tx_status;
              main_table[1][2] = 0;
              flag_to_r2.write(true);  // creating positive edge
              data_to_r2.write(status);
              wait(r_ack2.posedge_event()); // wait for ack from the receiving process
              flag_to_r2.write(false);
              wait(SC_ZERO_TIME);
            }
            if (main_table[2][2] == 1)
            {
              tx_status = main_table[2][1];
              tx_table[2][1] = tx_status;
              main_table[2][2] = 0;
              flag_to_r3.write(true);  // creating positive edge
              data_to_r3.write(status);
              wait(r_ack3.posedge_event()); // wait for ack from the receiving process
              flag_to_r3.write(false);
              wait(SC_ZERO_TIME);
            }
            if (main_table[3][2] == 1)
            {
              tx_status = main_table[3][1];
              tx_table[3][1] = tx_status;
              main_table[3][2] = 0;
              flag_to_r4.write(true);  // creating positive edge
              data_to_r4.write(status);
              wait(r_ack4.posedge_event()); // wait for ack from the receiving process
              flag_to_r4.write(false);
              wait(SC_ZERO_TIME);
            }
            tx_counter--;
          }
        }
      }  
      
      void send_path(int robot_index)
      {
        tx_table[robot_index][1] = 11;
        tx_table[robot_index][2] = 1;
        tx_counter++;
        for (int i = 0; i < 23; i++)
        {
          if (robot_index == 0)
          {
            fifo_r1[i].write(path[robot_index][i]);
            if (path[robot_index][i] == -1)
            {
              break;
            }
          }
          if (robot_index == 1)
          {
            fifo_r2[i].write(path[robot_index][i]);
            if (path[robot_index][i] == -1)
            {
              break;
            }
          }
          if (robot_index == 2)
          {
            fifo_r3[i].write(path[robot_index][i]);
            if (path[robot_index][i] == -1)
            {
              break;
            }
          }
          if (robot_index == 3)
          {
            fifo_r4[i].write(path[robot_index][i]);
            if (path[robot_index][i] == -1)
            {
              break;
            }
          }
        }
      }
       
};