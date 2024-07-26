#include "systemc.h"
#include <math.h>
#include <cmath>

class processing : public sc_module
{
  public:
    /**********INPUTS**********/
    sc_in<bool> clock; 
    sc_in<int> fifo_r1[23], fifo_r2[23], fifo_r3[23], fifo_r4[23];
    /*** rx_in ***/
    sc_in<bool> r1_flag, r2_flag, r3_flag, r4_flag;
    sc_in<sc_uint<16> > data_r1, data_r2, data_r3, data_r4;
    /*** tx_out ***/
    sc_in<bool> ack_r1, ack_r2, ack_r3, ack_r4;
    sc_in<bool> r1_sent, r2_sent, r3_sent, r4_sent;
    sc_in<bool> r1_speed, r2_speed, r3_speed, r4_speed;
    
    /**********OUTPUTS**********/
    /*** tx_out ***/
    sc_out<bool> output_flag1, output_flag2;
    sc_out<bool> output_flag3, output_flag4; 
    sc_out<sc_uint<16> > output_data1, output_data2;
    sc_out<sc_uint<16> > output_data3, output_data4;
    /*** rx_in ***/
    sc_out<bool> output_ack1, output_ack2;
    sc_out<bool> output_ack3, output_ack4;
    
    sc_out<double> robot1_x, robot2_x, robot3_x, robot4_x;
    sc_out<double> robot1_y, robot2_y, robot3_y, robot4_y;
    
    /**********CONSTRUCTOR**********/
    SC_HAS_PROCESS(processing);
    
    processing(sc_module_name name, sc_trace_file* trace_file) : sc_module(name), tf (trace_file)
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
      
      sc_trace(tf, speed_r1, "Robot1_Speed");
      sc_trace(tf, speed_r2, "Robot2_Speed");
      sc_trace(tf, speed_r3, "Robot3_Speed");
      sc_trace(tf, speed_r4, "Robot4_Speed");
    }
  
  private:
    
    sc_trace_file *tf;
    
    double speed_r1 = 0;
    double speed_r2 = 0;
    double speed_r3 = 0;
    double speed_r4 = 0;
    
    /*** Counters for Requests ***/
    // incremented by prc_rx(), decremented by prc_update();
    int rx_counter = 0;
    // incremented by prc_update(), decremented by prc_tx();
    int tx_counter = 0;
    
    //Local events for triggering Interprocess Communications
    //sc_event rx_signal;
    sc_event tx_signal;
    
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
                         
    // **************************************
    // Status Value from Robot (Processing) to Server
    // Status 0 -> STOPPED1 (stopped due to obstacles)
    // Status 1 -> RESTART (ask server to started after obstacle clearance)
    // Status 2 -> CROSSING (just before the boundary crossing)
    // Status 3 -> STOPPED2 (stopped at the boundary due to no Ack from Server)
    // Status 4 -> CROSSED (this will tell server to update server’s table)
    // **************************************
    // Status Value from Server to Robot (Processing)
    // Status 5 -> OK1 (ok to cross)
    // Status 6 -> OK2 (ok to move)
    // Status 7 -> STOP1 (stop, do not cross, do not move)
    // Status 8 -> STOP2 (do not move)
    // Status 9 -> RESUME (resume from previous stop)
    // Status 10 -> 10 SPEED (sending speed value via FIFO) – Phase 2
    // Status 11 -> PATH (sending path sequence via FIFO) – Phase 2
    sc_uint<16> status;
    
    // Direction at which the robot moves
    // 0 -> North
    // 1 -> West
    // 2 -> East
    // 3 -> South
    //(Robot Index, Direction)
    int direction[4][2] = {  {0, -1},
                             {1, -1},
                             {2, -1},
                             {3, -1}  };
                             
    int obstacle_direction[6][2] = {  {0, -1},
                                      {1, -1},
                                      {2, -1},
                                      {3, -1},
                                      {4, -1},
                                      {5, -1}  };
    
    double robot_speed = 0.02;
    double obstacle_speed = 0.04;
    
    // x, y
    double robot1[2] = {0, 0};
    double robot2[2] = {0, 0};
    double robot3[2] = {0, 0};
    double robot4[2] = {0, 0};
    
    // x, y
    double obstacle1[2] = {1, 17};
    double obstacle2[2] = {1, 13};
    double obstacle3[2] = {11, 13};
    double obstacle4[2] = {1, 9};
    double obstacle5[2] = {13, 9};
    double obstacle6[2] = {1, 5};
    
    int r1_init = 0;
    int r2_init = 0;
    int r3_init = 0;
    int r4_init = 0;
    
    int r1_stop = 0;
    int r2_stop = 0;
    int r3_stop = 0;
    int r4_stop = 0;
    
    int obstacle_path[6][30] = { {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 11, 1},
                               {13, 14, 15, 16, 17, 18, 24, 31, 30, 29, 28, 27, 26,23, 13},
                               {18, 19, 29, 21, 22, 25, 35, 34, 33, 32, 31, 24, 18},
                               {26, 27, 28, 29, 30, 31, 32, 37, 45, 44, 43, 42, 41, 40, 39, 36, 26},
                               {32, 33, 34, 35, 38, 48, 47, 46, 45, 37, 32},
                               {39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 50, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 49, 39},
                             };
      
    // (Grid #, x_start, x_end, y_start, y_end, N, S, W, E)
    int grid[61][9] = {     {1, 0, 2, 16, 18, -1, 11, -1, 2},
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
                            
    int count_clock = -1;
    float sum;
    float mod;  
    float temp;
    float mod2;
    int increase_speed;
                            
    void prc_rx()
    {
      output_ack1.write(false);
      output_ack2.write(false);
      output_ack3.write(false);
      output_ack4.write(false);
      // Wait for request (flag.pos()) from sending process
      wait(r1_flag.posedge_event() | r2_flag.posedge_event() |
           r3_flag.posedge_event() | r4_flag.posedge_event());
      // Robot #1
      if (r1_flag.read() == true)
      {
        status = data_r1;
        // Status 5 -> OK1 (ok to cross)
        if (status ==  5)  
        {
          // Status 2 -> CROSSING (just before the boundary crossing)
          status = 2;  
          rx_table[0][2] = 1;  // Modified
        }
        // Status 7 -> STOP1 (stop, do not cross, do not move)  
        else if (status == 7) 
        {
          // Status 3 -> STOPPED2 (stopped at the boundary due to no Ack from Server)
          status = 3;  
          rx_table[0][2] = 1;  // Modified
        }
        rx_counter++;
        rx_table[0][1] = status;
        output_ack1.write(true);  // creating positive edge
      }
      
      // Robot #2
      if (r2_flag.read() == true)
      {
        status = data_r2;
        // Status 5 -> OK1 (ok to cross)
        if (status ==  5)  
        {
          // Status 2 -> CROSSING (just before the boundary crossing)
          status = 2;  
          rx_table[1][2] = 1;  // Modified
        }
        // Status 7 -> STOP1 (stop, do not cross, do not move)  
        else if (status == 7) 
        {
          // Status 3 -> STOPPED2 (stopped at the boundary due to no Ack from Server)
          status = 3;  
          rx_table[1][2] = 1;  // Modified
        }
        rx_counter++;
        rx_table[1][1] = status;
        output_ack2.write(true);  // creating positive edge
      }
      
      // Robot #3
      if (r3_flag.read() == true)
      {
        status = data_r3;
        // Status 5 -> OK1 (ok to cross)
        if (status ==  5)  
        {
          // Status 2 -> CROSSING (just before the boundary crossing)
          status = 2;  
          rx_table[2][2] = 1;  // Modified
        }
        // Status 7 -> STOP1 (stop, do not cross, do not move)  
        else if (status == 7) 
        {
          // Status 3 -> STOPPED2 (stopped at the boundary due to no Ack from Server)
          status = 3;  
          rx_table[2][2] = 1;  // Modified
        }
        rx_counter++;
        rx_table[2][1] = status;
        output_ack3.write(true);  // creating positive edge
      }
      
      // Robot #4
      status = data_r4;
      // Status 5 -> OK1 (ok to cross)
      if (r4_flag.read() == true)
      {
        status = data_r4;
        if (status ==  5)  
        {
          // Status 2 -> CROSSING (just before the boundary crossing)
          // Status 5 -> OK1 (ok to cross)
          status = 2;  
          rx_table[3][2] = 1;  // Modified
        }
        // Status 7 -> STOP1 (stop, do not cross, do not move)  
        else if (status == 7) 
        {
          // Status 3 -> STOPPED2 (stopped at the boundary due to no Ack from Server)
          status = 3; 
          rx_table[3][2] = 1;  // Modified 
        }
        rx_counter++;
        rx_table[0][3] = status;
        output_ack4.write(true);  // creating positive edge
      }
      
      //cout << endl << "Processing prc_rx()" << endl;
    }
    
    void prc_update()
    {
      // If rx_counter is not zero, 
      // then scan RX Table and update Main Table.
      // Resets modified to 0
      for (int i = 0; i < rx_counter; i++)
      {
        if (rx_table[i][2] == 1)
        {
          main_table[i][2] = rx_table[i][2];
          rx_table[i][2] = 0;
        }
        if (i == (rx_counter - 1))
        {
          rx_counter = 0;
        }
      }
      
      int r1_next;
      int r2_next;
      int r3_next;
      int r4_next;
      int r1_dest = 0;
      int r2_dest = 0;
      int r3_dest = 0;
      int r4_dest = 0;
      
      if (r1_sent.read() == true)
      {
        if (r1_init == 0)
        {
          for (int i = 0; i < 60; i++)
          {
            if (grid[i][0] == fifo_r1[0])
            {
              robot1[0] = grid[i][2] - 1;
              robot1[1] = grid[i][4] - 1;
              r1_init = 1;
            }
          }
        }
        for (int i = 0; i < 60; i++)
        {
          if ((robot1[0] >= grid[i][1]) && (robot1[0] <= grid[i][2]) && 
             (robot1[1] >= grid[i][3]) && (robot1[1] <= grid[i][4]))
          {
            for (int a = 0; a < 23; a++)
            {
              if (fifo_r1[a] == grid[i][0])
              {
                r1_next = fifo_r1[a+1];
              }
            }
            for (int b = 4; b < 9; b++)
            {                        
              if (grid[i][b] == r1_next)
              {                         
                if (b == 5)
                {
                  direction[0][1] = 0;
                }
                else if (b == 6)
                {
                  direction[0][1] = 3;
                }
                else if (b == 7)
                {
                  direction[0][1] = 1;
                }
                else if (b == 8)
                {
                  direction[0][1] = 2;
                }
              }
            }
          }
        }
      }
      
      if (r2_sent.read() == true)
      {
        if (r2_init == 0)
        {
          for (int i = 0; i < 60; i++)
          {
            if (grid[i][0] == fifo_r2[0])
            {
              robot2[0] = grid[i][2] - 1;
              robot2[1] = grid[i][4] - 1;
              r2_init = 1;
            }
          }
        }
        for (int i = 0; i < 60; i++)
        {
          if ((robot2[0] >= grid[i][1]) && (robot2[0] <= grid[i][2]) && 
              (robot2[1] >= grid[i][3]) && (robot2[1] <= grid[i][4]))
          {
            for (int a = 0; a < 23; a++)
            {
              if (fifo_r2[a] == grid[i][0])
              {
                r2_next = fifo_r2[a+1];
              }
            }
            for (int b = 4; b < 9; b++)
            {                         
              if (grid[i][b] == r2_next)
              {                           
                if (b == 5)
                {
                  direction[1][1] = 0;
                }
                else if (b == 6)
                {
                  direction[1][1] = 3;
                }
                else if (b == 7)
                {
                  direction[1][1] = 1;
                }
                else if (b == 8)
                {
                  direction[1][1] = 2;
                }
              }
            }
          }
        }
      }
      
      if (r3_sent.read() == true)
      {
        if (r3_init == 0)
        {
          for (int i = 0; i < 60; i++)
          {
            if (grid[i][0] == fifo_r3[0])
            {
              robot3[0] = grid[i][2] - 1;
              robot3[1] = grid[i][4] - 1;
              r3_init = 1;
            }
          }
        }
        for (int i = 0; i < 60; i++)
        {
          if ((robot3[0] >= grid[i][1]) && (robot3[0] <= grid[i][2]) && 
              (robot3[1] >= grid[i][3]) && (robot3[1] <= grid[i][4]))
          {
            for (int a = 0; a < 23; a++)
            {
              if (fifo_r3[a] == grid[i][0])
              {
                r3_next = fifo_r3[a+1];
              }
            }
            for (int b = 4; b < 9; b++)
            {                          
              if (grid[i][b] == r3_next)
              {                              
                if (b == 5)
                {
                  direction[2][1] = 0;
                }
                else if (b == 6)
                {
                  direction[2][1] = 3;
                }
                else if (b == 7)
                {
                  direction[2][1] = 1;
                }
                else if (b == 8)
                {
                  direction[2][1] = 2;
                }
              }
            }
          }
        }
      }
      
      if (r4_sent.read() == true)
      {
        if (r4_init == 0)
        {
          for (int i = 0; i < 60; i++)
          {
            if (grid[i][0] == fifo_r4[0])
            {
              robot4[0] = grid[i][2] - 1;
              robot4[1] = grid[i][4] - 1;
              r4_init = 1;
            }
          }
        }
        for (int i = 0; i < 60; i++)
        {
          if ((robot4[0] >= grid[i][1]) && (robot4[0] <= grid[i][2]) && 
              (robot4[1] >= grid[i][3]) && (robot4[1] <= grid[i][4]))
          {
            for (int a = 0; a < 23; a++)
            {
              if (fifo_r4[a] == grid[i][0])
              {
                r4_next = fifo_r4[a+1];
              }
            }
            for (int b = 4; b < 9; b++)
            {                         
              if (grid[i][b] == r4_next)
              {                             
                if (b == 5)
                {
                  direction[3][1] = 0;
                }
                else if (b == 6)
                {
                  direction[3][1] = 3;
                }
                else if (b == 7)
                {
                  direction[3][1] = 1;
                }
                else if (b == 8)
                {
                  direction[3][1] = 2;
                }
              }
            }
          }
        }
      }
      
      //cout << "r1_speed: " << r1_speed << endl;
      //cout << "speed_r1: " << speed_r1 << endl;
      //cout << "r2_speed: " << speed_r2 << endl;
      //cout << "r3_speed: " << speed_r3 << endl;
      //cout << "r4_speed: " << r4_speed << endl;
      if (r1_speed == 1 && speed_r1 <= 0.02)
      {
        speed_r1 += 0.001;
      }
      else if (r1_speed == 0)
      {
        speed_r1 = 0;
      }
      if (r2_speed == 1 && speed_r2 <= 0.02)
      {
        speed_r2 += 0.001;
      }
      else if (r2_speed == 0)
      {
        speed_r2 = 0;
      }
      if (r3_speed == 1 && speed_r3 <= 0.02)
      {
        speed_r3 += 0.001;
      }
      else if (r3_speed == 0)
      {
        speed_r3 = 0;
      }
      if (r4_speed == 1 && speed_r4 <= 0.02)
      {
        speed_r4 += 0.001;
      }
      else if (r4_speed == 0)
      {
        speed_r4 = 0;
      }
      
      // Loop 1: each cycle, each robot updates the position.
      for (int i = 0; i < 4; i++)
      {
        // Status 2 -> CROSSING (just before the boundary crossing)
        // Status 5 -> OK1 (ok to cross)
        if (main_table[i][1] == 2 || main_table[i][1] == 4 || 
            main_table[i][1] == 5 || main_table[i][1] == 9)
        {
          for (int j = 0; j < 23; j++)
          {
            if (i == 0)
            {
              for (int k = 0; k < 60; k++)
              {
                if ((robot1[0] >= grid[k][1]) && (robot1[0] <= grid[k][2]) && 
                    (robot1[1] >= grid[k][3]) && (robot1[1] <= grid[k][4]))
                {
                  if (fifo_r1[j] == -1)
                  { 
                    if (fifo_r1[j-1] == grid[k][0])
                    {
                      r1_dest = 1;     
                      speed_r1 = 0;                 
                    }
                  } 
                }
              }
            }
            if (i == 1)
            {
              for (int k = 0; k < 60; k++)
              {
                if ((robot2[0] >= grid[k][1]) && (robot2[0] <= grid[k][2]) && 
                    (robot2[1] >= grid[k][3]) && (robot2[1] <= grid[k][4]))
                {
                  if (fifo_r2[j] == -1)
                  { 
                    if (fifo_r2[j-1] == grid[k][0])
                    {
                      r2_dest = 1;
                      speed_r2 = 0;  
                    }
                  } 
                }
              }
            }
            if (i == 2)
            {
              for (int k = 0; k < 60; k++)
              {
                if ((robot3[0] >= grid[k][1]) && (robot3[0] <= grid[k][2]) && 
                    (robot3[1] >= grid[k][3]) && (robot3[1] <= grid[k][4]))
                {
                  if (fifo_r3[j] == -1)
                  { 
                    if (fifo_r3[j-1] == grid[k][0])
                    {
                      r3_dest = 1;
                      speed_r3 = 0;  
                    }
                  } 
                }
              }
            }
            if (i == 3)
            {
              for (int k = 0; k < 60; k++)
              {
                if ((robot4[0] >= grid[k][1]) && (robot4[0] <= grid[k][2]) && 
                    (robot4[1] >= grid[k][3]) && (robot4[1] <= grid[k][4]))
                {
                  if (fifo_r4[j] == -1)
                  { 
                    if (fifo_r4[j-1] == grid[k][0])
                    {
                      r4_dest = 1;
                      speed_r4 = 0;  
                    }
                  } 
                }
              }
            }
          }
          if (r1_dest == 1)
          {
            direction[0][1] = -1;
          }
          if (r2_dest == 1)
          {
            direction[1][1] = -1;
          }
          if (r3_dest == 1)
          {
            direction[2][1] = -1;
          }
          if (r4_dest == 1)
          {
            direction[3][1] = -1;
          }
          if (direction[i][1] == 0)
          {
            // Moves North
            if (i == 0)
            { 
              //speed_r1 += 0.001;           
              robot1[1] += robot_speed;
              speed_r1 = 2;
              //robot1[1] += speed_r1;
              
            }
            else if (i == 1)
            {
              //speed_r2 += 0.001;   
              robot2[1] += robot_speed;
              speed_r2 = 2;
              //robot2[1] += speed_r2;
            }
            else if (i == 2)
            {
              robot3[1] += robot_speed;
              speed_r3 = 2;
              //robot3[1] += speed_r3;
            }
            else if (i == 3)
            {
              robot4[1] += robot_speed;
              speed_r4 = 2;
              //robot4[1] += speed_r4;
            }
          }
          // Moves West
          else if (direction[i][1] == 1)
          {
            if (i == 0)
            {
              robot1[0] -= robot_speed;
              speed_r1 = 2;
              //robot1[0] -= speed_r1;
            }
            else if (i == 1)
            {
              robot2[0] -= robot_speed;
              speed_r2 = 2;
              //robot2[0] -= speed_r2;
            }
            else if (i == 2)
            {
              robot3[0] -= robot_speed;
              speed_r3 = 2;
              //robot3[0] -= speed_r3;
              
            }
            else if (i == 3)
            {
              robot4[0] -= robot_speed;
              speed_r4 = 2;
              //robot4[0] -= speed_r4;
            }
          }
          // Moves East
          else if (direction[i][1] == 2)
          {
            if (i == 0)
            {
              robot1[0] += robot_speed;
              speed_r1 = 2;
              //robot1[0] += speed_r1;
            }
            else if (i == 1)
            {
              robot2[0] += robot_speed;
              speed_r2 = 2;
              //robot2[0] += speed_r2;
            }
            else if (i == 2)
            {
              robot3[0] += robot_speed;
              speed_r3 = 2;
              //robot3[0] += speed_r3;
            }
            else if (i == 3)
            {
              robot4[0] += robot_speed;
              speed_r4 = 2;
              //robot4[0] += speed_r4;
            }        
          }
          // Moves South
          else if (direction[i][1] == 3)
          {
            if (i == 0)
            {
              robot1[1] -= robot_speed;
              speed_r1 = 2;
              //robot1[1] -= speed_r1;
              
            }
            else if (i == 1)
            {
              robot2[1] -= robot_speed;
              speed_r2 = 2;
              //robot2[1] -= speed_r2;
            }
            else if (i == 2)
            {
              robot3[1] -= robot_speed;
              speed_r3 = 2;
              //robot3[1] -= speed_r3;
            }
            else if (i == 3)
            {
              robot4[1] -= robot_speed;
              speed_r4 = 2;
              //robot4[1] -= speed_r4;
            }
          }
        }
      }  

      // Loop 2: each cycle, each obstacle updates the position
      for (int i = 0; i < 6; i++)
      {
        if (i == 0)
        {
          if ((obstacle1[0] > 0 && obstacle1[0] < 19) &&
              (obstacle1[1] > 16  && obstacle1[1] < 18) )
          {
            obstacle1[0] += obstacle_speed;
          }
          else if ((obstacle1[0] > 18 && obstacle1[0] < 20) &&
              (obstacle1[1] > 13  && obstacle1[1] < 18) )
          {
            obstacle1[1] -= obstacle_speed;
          }
          else if ((obstacle1[0] > 1 && obstacle1[0] < 20) &&
              (obstacle1[1] > 12  && obstacle1[1] < 14) )
          {
            obstacle1[0] -= obstacle_speed;
          }
          else if ((obstacle1[0] > 0 && obstacle1[0] < 2) &&
              (obstacle1[1] > 12  && obstacle1[1] < 17) )
          {
            obstacle1[1] += obstacle_speed;
          }
        }
        
        if (i == 1)
        {
          if ((obstacle2[0] > 0 && obstacle2[0] < 11) &&
              (obstacle2[1] > 12  && obstacle2[1] < 14) )
          {
            obstacle2[0] += obstacle_speed;
          }
          else if ((obstacle2[0] < 12 && obstacle2[0] > 10) &&
                   (obstacle2[1] < 14  && obstacle2[1] > 9) )
          {
            obstacle2[1] -= obstacle_speed;
          }
          else if ((obstacle2[0] < 12 && obstacle2[0] > 1) &&
                   (obstacle2[1] < 10  && obstacle2[1] > 8) )
          {
            obstacle2[0] -= obstacle_speed;
          }
          else if ((obstacle2[0] < 2 && obstacle2[0] > 0) &&
                   (obstacle2[1] < 13 && obstacle2[1] > 8) )
          {
            obstacle2[1] += obstacle_speed;
          } 
        }
        
        if (i == 2)
        {
          if ((obstacle3[0] > 10 && obstacle3[0] < 19) &&
              (obstacle3[1] > 12  && obstacle3[1] < 14) )
          {
            obstacle3[0] += obstacle_speed;
          }
          else if ((obstacle3[0] < 20 && obstacle3[0] > 18) &&
                   (obstacle3[1] < 14  && obstacle3[1] > 9) )
          {
            obstacle3[1] -= obstacle_speed;
          }
          else if ((obstacle3[0] < 20 && obstacle3[0] > 11) &&
                   (obstacle3[1] < 10  && obstacle3[1] > 8) )
          {
            obstacle3[0] -= obstacle_speed;
          }
          else if ((obstacle3[0] < 12 && obstacle3[0] > 10) &&
                   (obstacle3[1] < 13 && obstacle3[1] > 8) )
          {
            obstacle3[1] += obstacle_speed;
          }
        }
        
        if (i == 3)
        {
        	if ((obstacle4[0] > 0 && obstacle4[0] < 13) &&
          	  (obstacle4[1] < 10 && obstacle4[1] > 8) )
        	{
        		obstacle4[0] += obstacle_speed;
        	}
        	else if ((obstacle4[0] < 14 && obstacle4[0] > 12) &&
        	(obstacle4[1] < 10 && obstacle4[1] > 5) )
        	{
        		obstacle4[1] -= obstacle_speed;
        	}
        	else if ((obstacle4[0] < 14 && obstacle4[0] > 1) &&
        	(obstacle4[1] < 6  && obstacle4[1] > 4) )
        	{
        		obstacle4[0] -= obstacle_speed;
        	}
        	else if ((obstacle4[0] < 2 && obstacle4[0] > 0) &&
        	(obstacle4[1] < 9 && obstacle4[1] > 4) )
        	{
        		obstacle4[1] += obstacle_speed;
        	}
        }
        
        if (i == 4)
        {
        	if ((obstacle5[0] > 12 && obstacle5[0] < 19) &&
        	    (obstacle5[1] < 10  && obstacle5[1] > 8) )
        	{
        		obstacle5[0] += obstacle_speed;
        	}
        	else if ((obstacle5[0] < 20 && obstacle5[0] > 18) &&
        	(obstacle5[1] < 10  && obstacle5[1] > 5) )
        	{
        		obstacle5[1] -= obstacle_speed;
        	}
        	else if ((obstacle5[0] < 20 && obstacle5[0] > 13) &&
        	         (obstacle5[1] < 6  && obstacle5[1] > 4) )
      	  {
        		obstacle5[0] -= obstacle_speed;
        	}
        	else if ((obstacle5[0] < 14 && obstacle5[0] > 12) &&
        	(obstacle5[1] < 9 && obstacle5[1] > 4) )
        	{
        		obstacle5[1] += obstacle_speed;
        	}
        }
        
        if (i == 5)
        {
        	if ((obstacle6[0] > 0 && obstacle6[0] < 19) &&
            	(obstacle6[1] < 6  && obstacle6[1] > 4) )
        	{
        		obstacle6[0] += obstacle_speed;
        	}
        	else if ((obstacle6[0] < 20 && obstacle6[0] > 18) &&
        	(obstacle6[1] < 6  && obstacle6[1] > 1) )
        	{
        		obstacle6[1] -= obstacle_speed;
        	}
        	else if ((obstacle6[0] < 20 && obstacle6[0] > 1) &&
        	(obstacle6[1] < 2  && obstacle6[1] > 0) )
        	{
        		obstacle6[0] -= obstacle_speed;
        	}
        	else if ((obstacle6[0] < 2 && obstacle6[0] > 0) &&
        	(obstacle6[1] < 5 && obstacle6[1] > 0) )
        	{
        		obstacle6[1] += obstacle_speed;
        	}
        }         
      }
      
      // Loop 3: for each robot, compare the distance from the boundary. 
      // If close to the boundary (10% of the grid size), 
      // sends the signal to the robot (CROSSING). If the robot crosses 
      // the boundary, sends the signal to the robot (CROSSED).
      for (int i = 0; i < 4; i++)
      {
        if (direction[i][1] == 0)
        {
          if (i == 0)
          {
            sum = robot1[1] + 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot1[1];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[0][1] = 2; 
              main_table[0][2] = 1;  // Status modified
              tx_counter++;
              if (r1_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #1 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[0][1] = 4; 
              main_table[0][2] = 1;  // Status modified
              tx_counter++;
              if (r1_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #1 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          if (i == 1)
          {
            sum = robot2[1] + 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot2[1];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[1][1] = 2; 
              main_table[1][2] = 1;  // Status modified
              tx_counter++;
              if (r2_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #2 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[1][1] = 4; 
              main_table[1][2] = 1;  // Status modified
              tx_counter++;
              if (r2_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #2 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          if (i == 2)
          {
            sum = robot3[1] + 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot3[1];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[2][1] = 2; 
              main_table[2][2] = 1;  // Status modified
              tx_counter++;
              if (r3_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #3 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[2][1] = 4; 
              main_table[2][2] = 1;  // Status modified
              tx_counter++;
              if (r3_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #3 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          if (i == 3)
          {
            sum = robot4[1] + 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot4[1];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[3][1] = 2; 
              main_table[3][2] = 1;  // Status modified
              tx_counter++;
              if (r4_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #4 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[3][1] = 4; 
              main_table[3][2] = 1;  // Status modified
              tx_counter++;
              if (r1_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #4 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
        }
        if (direction[i][1] == 1)
        {
          if (i == 0)
          {
            sum = robot1[0] - 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot1[0];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[0][1] = 2; 
              main_table[0][2] = 1;  // Status modified
              tx_counter++;
              if (r1_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #1 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[0][1] = 4; 
              main_table[0][2] = 1;  // Status modified
              tx_counter++;
              if (r1_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #1 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          if (i == 1)
          {
            sum = robot2[0] - 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot2[0];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[1][1] = 2; 
              main_table[1][2] = 1;  // Status modified
              tx_counter++;
              if (r2_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #2 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[1][1] = 4; 
              main_table[1][2] = 1;  // Status modified
              tx_counter++;
              if (r2_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #2 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          if (i == 2)
          {
            if ((robot3[0] < 2) && (robot3[0] < 2) && 
                (robot3[1] > 4) && (robot3[1] > 18))
            {
              sum = robot3[1] + 0.2;       
              mod = fmod(sum, 2);
              //
              temp = robot3[1];
              mod2 = fmod(temp, 2);
                                             
              if (mod == 0)
              {          
                // Status 2 -> CROSSING (just before the boundary crossing)
                main_table[2][1] = 2; 
                main_table[2][2] = 1;  // Status modified
                tx_counter++;
                if (r3_dest != 1)
                {
                  cout << "Time: " << sc_time_stamp() << " | Robot #3 is CROSSING" << endl;
                }
              }
              else if (mod2 == 0)
              {
                // Status 4 -> CROSSED
                main_table[2][1] = 4; 
                main_table[2][2] = 1;  // Status modified
                tx_counter++;
                if (r3_dest != 1)
                {
                  cout << "Time: " << sc_time_stamp() << " | Robot #3 CROSSED" << endl;
                }
              }
              else
              {
              }
            }
            else
            {
              sum = robot3[0] - 0.2;       
              mod = fmod(sum, 2);
              //
              temp = robot3[0];
              mod2 = fmod(temp, 2);
                                             
              if (mod == 0)
              {          
                // Status 2 -> CROSSING (just before the boundary crossing)
                main_table[2][1] = 2; 
                main_table[2][2] = 1;  // Status modified
                tx_counter++;
                if (r3_dest != 1)
                {
                  cout << "Time: " << sc_time_stamp() << " | Robot #3 is CROSSING" << endl;
                }
              }
              else if (mod2 == 0)
              {
                // Status 4 -> CROSSED
                main_table[2][1] = 4; 
                main_table[2][2] = 1;  // Status modified
                tx_counter++;
                if (r3_dest != 1)
                {
                  cout << "Time: " << sc_time_stamp() << " | Robot #3 CROSSED" << endl;
                }
              }
              else
              {
              }
            }
            
          }
          if (i == 3)
          {
            sum = robot4[0] - 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot4[0];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[3][1] = 2; 
              main_table[3][2] = 1;  // Status modified
              tx_counter++;
              if (r4_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #4 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[3][1] = 4; 
              main_table[3][2] = 1;  // Status modified
              tx_counter++;
              if (r4_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #4 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          
        }
        if (direction[i][1] == 2)
        {
          if (i == 0)
          {
            sum = robot1[0] + 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot1[0];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[0][1] = 2; 
              main_table[0][2] = 1;  // Status modified
              tx_counter++;
              if (r1_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #1 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[0][1] = 4; 
              main_table[0][2] = 1;  // Status modified
              tx_counter++;
              if (r1_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #1 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          if (i == 1)
          {
            sum = robot2[0] + 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot2[0];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[1][1] = 2; 
              main_table[1][2] = 1;  // Status modified
              tx_counter++;
              if (r2_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #2 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[1][1] = 4; 
              main_table[1][2] = 1;  // Status modified
              tx_counter++;
              if (r2_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #2 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          if (i == 2)
          {
            sum = robot3[0] + 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot3[0];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[2][1] = 2; 
              main_table[2][2] = 1;  // Status modified
              tx_counter++;
              if (r3_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #3 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[2][1] = 4; 
              main_table[2][2] = 1;  // Status modified
              tx_counter++;
              if (r3_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #3 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          if (i == 3)
          {
            sum = robot4[0] + 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot4[0];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[3][1] = 2; 
              main_table[3][2] = 1;  // Status modified
              tx_counter++;
              if (r4_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #4 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[3][1] = 4; 
              main_table[3][2] = 1;  // Status modified
              tx_counter++;
              if (r4_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #4 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
        }
        if (direction[i][1] == 3)
        {
          if (i == 0)
          {
            sum = robot1[1] - 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot1[1];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[0][1] = 2; 
              main_table[0][2] = 1;  // Status modified
              tx_counter++;
              if (r1_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #1 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[0][1] = 4; 
              main_table[0][2] = 1;  // Status modified
              tx_counter++;
              if (r1_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #1 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          if (i == 1)
          {
            sum = robot2[1] - 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot2[1];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[1][1] = 2; 
              main_table[1][2] = 1;  // Status modified
              tx_counter++;
              if (r2_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #2 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[1][1] = 4; 
              main_table[1][2] = 1;  // Status modified
              tx_counter++;
              if (r2_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #2 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          if (i == 2)
          {
            sum = robot3[1] - 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot3[1];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[2][1] = 2; 
              main_table[2][2] = 1;  // Status modified
              tx_counter++;
              if (r3_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #3 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[2][1] = 4; 
              main_table[2][2] = 1;  // Status modified
              tx_counter++;
              if (r3_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #3 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
          if (i == 3)
          {
            sum = robot4[1] - 0.2;       
            mod = fmod(sum, 2);
            //
            temp = robot4[1];
            mod2 = fmod(temp, 2);
                                           
            if (mod == 0)
            {          
              // Status 2 -> CROSSING (just before the boundary crossing)
              main_table[3][1] = 2; 
              main_table[3][2] = 1;  // Status modified
              tx_counter++;
              if (r4_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #4 is CROSSING" << endl;
              }
            }
            else if (mod2 == 0)
            {
              // Status 4 -> CROSSED
              main_table[3][1] = 4; 
              main_table[3][2] = 1;  // Status modified
              tx_counter++;
              if (r4_dest != 1)
              {
                cout << "Time: " << sc_time_stamp() << " | Robot #4 CROSSED" << endl;
              }
            }
            else
            {
            }
          }
        }
      }
      
      // Loop 4: for each robot, compare the distance from the obstacles. 
      // If the robot is close to the obstacle, stops the robot, sends 
      // robot the status. (STOPPED). If the obstacles are cleared, sends 
      // the signal to the robot (RESUMED). 
      if (abs(robot1[0] - obstacle1[0]) < 2.1 && abs(robot1[1] - obstacle1[1]) < 2.1)
      {
        //Status 0 -> STOPPED1 (stopped due to obstacles)
        main_table[0][1] = 0; 
        main_table[0][2] = 1;  // Status modified
        tx_counter++;
        if (r1_dest != 1 && r1_stop == 0)
        {
          cout << "Time: " << sc_time_stamp() << " | Robot #1 STOPPED" << endl;
          r1_stop = 1;
        }
        speed_r1 = 0;
      }
      else if (main_table[0][1] == 0)
      {
        // Status 9 -> RESUME
        main_table[0][1] = 9; 
        main_table[0][2] = 1;  // Status modified
        tx_counter++;
        if (r1_dest != 1)
        {
          cout << "Time: " << sc_time_stamp() << " | Robot #1 RESUMED" << endl;
          r1_stop = 0;
        }
        speed_r1 = 2;
      }
      if (abs(robot2[0] - obstacle1[0]) < 2.1 && abs(robot2[1] - obstacle1[1]) < 2.1 &&
          abs(robot2[0] - obstacle3[0]) < 2.1 && abs(robot2[1] - obstacle3[1]) < 3 &&
          abs(robot2[0] - obstacle5[0]) < 2.1 && abs(robot2[1] - obstacle5[1]) < 2.1 &&
          abs(robot2[0] - obstacle6[0]) < 2.1 && abs(robot2[1] - obstacle6[1]) < 2.1)
      {
        //Status 0 -> STOPPED1 (stopped due to obstacles)
        main_table[1][1] = 0; 
        main_table[1][2] = 1;  // Status modified
        tx_counter++;
        if (r2_dest != 1 && r2_stop == 0)         
        {
          cout << "Time: " << sc_time_stamp() << " | Robot #2 STOPPED" << endl;
          r2_stop = 1;
        }
        speed_r2 = 0;
      }
      else if (main_table[1][1] == 0)
      {
        // Status 9 -> RESUME
        main_table[1][1] = 9; 
        main_table[1][2] = 1;  // Status modified
        tx_counter++;
        if (r2_dest != 1 )
        {
          cout << "Time: " << sc_time_stamp() << " | Robot #2 RESUMED" << endl;
          r2_stop = 0;
        }
        speed_r2 = 2;
      }
      if (abs(robot3[0] - obstacle1[0]) < 2.1 && abs(robot3[1] - obstacle1[1]) < 2.1 &&
          abs(robot3[0] - obstacle2[0]) < 2.1 && abs(robot3[1] - obstacle2[1]) < 2.1 &&
          abs(robot3[0] - obstacle4[0]) < 2.1 && abs(robot3[1] - obstacle4[1]) < 2.1 &&
          abs(robot3[0] - obstacle5[0]) < 2.1 && abs(robot3[1] - obstacle5[1]) < 2.1 &&
          abs(robot3[0] - obstacle6[0]) < 2.1 && abs(robot3[1] - obstacle6[1]) < 2.1)
      {
        //Status 0 -> STOPPED1 (stopped due to obstacles)
        main_table[2][1] = 0; 
        main_table[2][2] = 1;  // Status modified
        tx_counter++;
        if (r3_dest != 1 && r3_stop == 0)
        {
          cout << "Time: " << sc_time_stamp() << " | Robot #3 STOPPED" << endl;
          r3_stop = 1;
        }
        speed_r3 = 0;
      }
      else if (main_table[2][1] == 0)
      {
        // Status 9 -> RESUME
        main_table[2][1] = 9; 
        main_table[2][2] = 1;  // Status modified
        tx_counter++;
        if (r3_dest != 1)
        {
          cout << "Time: " << sc_time_stamp() << " | Robot #3 RESUMED" << endl;
          r3_stop = 0;
        }
        speed_r3 = 2;
      }
      if (abs(robot4[0] - obstacle6[0]) < 2.1 && abs(robot4[1] - obstacle6[1]) < 2.1)
      {
        //Status 0 -> STOPPED1 (stopped due to obstacles)
        main_table[3][1] = 0; 
        main_table[3][2] = 1;  // Status modified
        tx_counter++;
        if (r4_dest != 1 && r4_stop == 0)
        {
          cout << "Time: " << sc_time_stamp() << " | Robot #4 STOPPED" << endl;
          r4_stop = 1;
        }
        speed_r4 = 0;
      }
      else if (main_table[3][1] == 0)
      {
        // Status 9 -> RESUME
        main_table[3][1] = 9; 
        main_table[3][2] = 1;  // Status modified
        tx_counter++;
        if (r4_dest != 1)
        {
          cout << "Time: " << sc_time_stamp() << " | Robot #4 RESUMED" << endl;
          r4_stop = 0;
        }
        speed_r4 = 2;
      }  
      
      robot1_x = robot1[0];
      robot1_y = robot1[1];
      robot2_x = robot2[0];
      robot2_y = robot2[1];
      robot3_x = robot3[0];
      robot3_y = robot3[1];
      robot4_x = robot4[0];
      robot4_y = robot4[1];
       
      if (tx_counter > 0)
      {
        tx_signal.notify();
      }
      
      if (count_clock++ == 100)
      {
        count_clock = 1;
        cout << endl;
        cout << "Time: " << 
        sc_time_stamp() << endl;
        cout << "~~~~ Robot Location ~~~~" << endl;
        for (int i = 0; i < 4; i++)
        {
          if (i == 0)
          {
            if (r1_sent != 1)
            {
              cout << "Robot #" << i+1 << " is at Grid #" << -1 << endl;       
            }
            else
            {
              update_position_of_robots(i, robot1[0], robot1[1]);
            }
          }
          
          if (i == 1)
          {
            if (r2_sent != 1)
            {
              cout << "Robot #" << i+1 << " is at Grid #" << -1 << endl;       
            }
            else
            {
              update_position_of_robots(i, robot2[0], robot2[1]);
            }
          }
          
          if (i == 2)
          {
            if (r3_sent != 1)
            {
              cout << "Robot #" << i+1 << " is at Grid #" << -1 << endl;       
            }
            else
            {
              update_position_of_robots(i, robot3[0], robot3[1]);
            }
          }
          
          if (i == 3)
          {
            if (r4_sent != 1)
            {
              cout << "Robot #" << i+1 << " is at Grid #" << -1 << endl;       
            }
            else
            {
              update_position_of_robots(i, robot4[0], robot4[1]);
            }
          }
        }
        cout << "~~ Obstacle Location ~~" << endl;
        for (int i = 0; i < 6; i++)
        {
          if (i == 0)
          {
            update_position_of_obstacles(i, obstacle1[0], obstacle1[1]);
          }
          
          if (i == 1)
          {
            update_position_of_obstacles(i, obstacle2[0], obstacle2[1]);
          }
          
          if (i == 2)
          {
            update_position_of_obstacles(i, obstacle3[0], obstacle3[1]);
          }
          
          if (i == 3)
          {
            update_position_of_obstacles(i, obstacle4[0], obstacle4[1]);
          }
          
          if (i == 4)
          {
            update_position_of_obstacles(i, obstacle5[0], obstacle5[1]);
          }
          
          if (i == 5)
          {
            update_position_of_obstacles(i, obstacle6[0], obstacle6[1]);
          }
        }
        cout << endl;
      }
      
      //cout << "Processing prc_update()" << endl;
    }
    
    /**** Method for finding the Grid # based on the x and y coordinates ****/
    void update_position_of_robots(int j, double x, double y)
    {
      for (int i = 0; i < 60; i++)
      {
        if ((x >= grid[i][1]) && (x <= grid[i][2]) && 
            (y >= grid[i][3]) && (y <= grid[i][4]))
        {
          cout << "Robot #" << j+1 << " is at Grid #" << grid[i][0] << " | ";
          for (int a = 0; a < 23; a++)
          {
            if (j == 0)
            {
              if (fifo_r1[a] == grid[i][0])
              {
                cout << "Next Grid: " <<  fifo_r1[a+1] << endl;;
              }
            }
            if (j == 1)
            {
              if (fifo_r2[a] == grid[i][0])
              {
                cout << "Next Grid: " <<  fifo_r2[a+1] << endl;;
              }
            }
            if (j == 2)
            {
              if (fifo_r3[a] == grid[i][0])
              {
                cout << "Next Grid: " <<  fifo_r3[a+1] << endl;;
              }
            }
            if (j == 3)
            {
              if (fifo_r4[a] == grid[i][0])
              {
                cout << "Next Grid: " <<  fifo_r4[a+1] << endl;;
              }
            }
          }
        }
      }
    }
    
    /**** Method for finding the Grid # based on the x and y coordinates ****/
    void update_position_of_obstacles(int j, double x, double y)
    {
      for (int i = 0; i < 60; i++)
      {
        if ((x > grid[i][1]) && (x < grid[i][2]) && 
            (y > grid[i][3]) && (y < grid[i][4]))
        {
          cout << "Obstacle #" << j+1 << " is at Grid #" << grid[i][0] << endl;
        }
      }
    }
    
    void prc_tx()
    {
      sc_uint<16> tx_status;
      while (true)
      {
        wait(tx_signal);  // wait for tx_signal.pos()
        output_flag1.write(false);
        output_flag2.write(false);
        output_flag3.write(false);
        output_flag4.write(false);
        while (tx_counter != 0)
        {
          if (main_table[0][2] == 1)
          {
            tx_status = main_table[0][1];
            tx_table[0][1] = tx_status;
            output_flag1.write(true);  // creating positive edge
            output_data1.write(status);
            wait(ack_r1.posedge_event()); // wait for ack from the receiving process
            output_flag1.write(false);
            wait(SC_ZERO_TIME);
          }
          if (main_table[1][2] == 1)
          {
            tx_status = main_table[1][1];
            tx_table[1][1] = tx_status;
            output_flag2.write(true);  // creating positive edge
            output_data2.write(status);
            wait(ack_r2.posedge_event()); // wait for ack from the receiving process
            output_flag2.write(false);
            wait(SC_ZERO_TIME);
          }
          if (main_table[2][2] == 1)
          {
            tx_status = main_table[2][1];
            tx_table[2][1] = tx_status;
            output_flag3.write(true);  // creating positive edge
            output_data3.write(status);
            wait(ack_r3.posedge_event()); // wait for ack from the receiving process
            output_flag3.write(false);
            wait(SC_ZERO_TIME);
          }
          if (main_table[3][2] == 1)
          {
            tx_status = main_table[3][1];
            tx_table[3][1] = tx_status;
            output_flag4.write(true);  // creating positive edge
            output_data4.write(status);
            wait(ack_r4.posedge_event()); // wait for ack from the receiving process
            output_flag4.write(false);
            wait(SC_ZERO_TIME);
          }
          tx_counter--;
        }
      }
    }
};