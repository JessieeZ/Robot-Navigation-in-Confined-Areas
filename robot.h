#include "systemc.h"

class robot : public sc_module
{
  public:
    /**********INPUTS**********/
    sc_in<bool> clock; 
    sc_in<bool> p_flag;
    sc_in<bool> p_ack;
    sc_in<sc_uint<16> > p_data;
    sc_in<bool> s_flag;
    sc_in<bool> s_ack;
    sc_in<sc_uint<16> > s_data;
    
    /**********OUTPUTS**********/
    sc_out<bool> flag_to_p;
    sc_out<bool> ack_to_p;
    sc_out<sc_uint<16> > data_to_p;
    sc_out<bool> flag_to_s;
    sc_out<bool> ack_to_s;
    sc_out<sc_uint<16> > data_to_s;
    
    /**********CONSTRUCTOR**********/
    SC_HAS_PROCESS(robot);
    
    robot(sc_module_name name, int rob_index) : sc_module(name), robot_index(rob_index)
    {
      // Internal Process
      SC_METHOD(prc_update);
      sensitive << clock.pos();
      
      // Processes: Connects to Robots
      // Receive
      SC_THREAD(prc_rx_p);
      //sensitive << rx_signal; // not necessary
      
      // Transmit
      SC_THREAD(prc_tx_p);
      sensitive << tx_p_signal;
      
      // Processes: Connects to Robots
      // Receive
      SC_THREAD(prc_rx_s);
      //sensitive << rx_signal; // not necessary
      
      // Transmit
      SC_THREAD(prc_tx_s);
      sensitive << tx_s_signal;
      
      robot_index = rob_index;
    }
    
    private:
    
      int robot_index;
      //Local events for triggering Interprocess Communications
      //sc_event rx_signal;
      
      sc_event tx_p_signal;
      sc_event tx_s_signal;
      
      /*** Counters for Requests ***/
      // incremented by prc_rx(), decremented by prc_update();
      int rx_p_counter = 0;
      // incremented by prc_update(), decremented by prc_tx();
      int tx_p_counter = 0;
      
      // incremented by prc_rx(), decremented by prc_update();
      int rx_s_counter = 0;
      // incremented by prc_update(), decremented by prc_tx();
      int tx_s_counter = 0;
      
      // (Robot Index, Status, Modified)
      int rx_p_table[3];
      int tx_p_table[3];
      int rx_s_table[3];
      int tx_s_table[3];
      int main_table[3];
      
      sc_uint<16> status;
      
      void prc_rx_p()
      {
        ack_to_p.write(false);
        wait(p_flag.posedge_event());
        rx_p_table[1] = p_data.read();
        rx_p_table[2] = 1;  // Modified
        ack_to_p.write(true);  // Send ack signal to process
        rx_p_counter++;
        
        //cout << "Robot prc_rx()" << endl;
      }
      
      void prc_rx_s()
      {
        ack_to_s.write(false);
        wait(s_flag.posedge_event());
        rx_s_table[1] = p_data.read();
        rx_s_table[2] = 1;  // Modified
        ack_to_s.write(true);  // Send ack signal to process
        rx_s_counter++;
        
        //cout << "Robot prc_rx()" << endl;
      }
      
      void prc_update()
      {
        for (int i = 0; i < rx_p_counter; i++)
        {
          if (rx_p_table[2] == 1)
          {
            status = rx_p_table[1];
            main_table[1] = status;
            main_table[2] = 1;  // Modified
            rx_p_table[2] = 0;
          }
          if (i == (rx_p_counter - 1))
          {
            rx_p_counter = 0;
          }
          tx_p_counter++;
        }
        
        for (int i = 0; i < rx_s_counter; i++)
        {
          if (rx_s_table[2] == 1)
          {
            status = rx_s_table[1];
            main_table[1] = status;
            main_table[2] = 1;  // Modified
            rx_s_table[2] = 0;
          }
          if (i == (rx_s_counter - 1))
          {
            rx_s_counter = 0;
          }
          tx_s_counter++;
        }
        
        if (tx_p_counter > 0)
        {
          tx_p_signal.notify();
        }
        
        if (tx_s_counter > 0)
        {
          tx_s_signal.notify();
        }
        
        //cout << "Robot prc_update()" << endl;
      }
      
      void prc_tx_p()
      {
        sc_uint<16> tx_status;
        wait(tx_p_signal);  // wait for tx_signal.pos()
        tx_status = main_table[1]; 
        tx_p_table[1] = tx_status;
        tx_p_table[2] = 1;  // Modified
        data_to_p.write(tx_status);
        flag_to_p.write(true);
        wait(p_ack.posedge_event()); // wait for ack from processing
        flag_to_p.write(false);
        //cout << "Robot prc_tx()" << endl;
      }
      
      void prc_tx_s()
      {
        sc_uint<16> tx_s_status;
        wait(tx_s_signal);  // wait for tx_signal.pos()
        tx_s_status = main_table[1]; 
        tx_s_table[1] = tx_s_status;
        tx_s_table[2] = 1;  // Modified
        data_to_s.write(tx_s_status);
        flag_to_s.write(true);
        wait(s_ack.posedge_event()); // wait for ack from processing
        flag_to_s.write(false);
        //cout << "Robot prc_tx()" << endl;
      }
};