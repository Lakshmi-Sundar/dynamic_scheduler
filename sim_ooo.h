#ifndef SIM_OO_H_
#define SIM_OO_H_

#include <stdio.h>
#include <stdbool.h>
#include <sstream>
#include <iostream>
#include <inttypes.h>
#include <cassert>
#include <cstdlib>
#include <stdlib.h>
#include <fstream>
#include <cstring>
#include <string>
#include <iomanip>
#include <map>

using namespace std;

template <class T> class Fifo;
#define UNDEFINED 0xFFFFFFFF //constant used for initialization
#define NUM_GP_REGISTERS 32
#define NUM_OPCODES 28
#define NUM_STAGES 4

typedef enum {LW, SW, ADD, ADDI, SUB, SUBI, XOR, XORI, OR, ORI, AND, ANDI, MULT, DIV, BEQZ, BNEZ, BLTZ, BGTZ, BLEZ, BGEZ, JUMP, EOP, LWS, SWS, ADDS, SUBS, MULTS, DIVS} opcode_t;

typedef enum {INTEGER_RS, ADD_RS, MULT_RS, LOAD_B, RS_TOTAL} res_station_t;

typedef enum {INTEGER, ADDER, MULTIPLIER, DIVIDER, MEMORY, EX_TOTAL} exe_unit_t;

typedef enum{ISSUE, EXECUTE, WRITE_RESULT, COMMIT} stage_t;

const string opcode_str[] = {"ADD", "SUB", "XOR", "OR", "AND", "MULT", "DIV", "BEQZ", "BNEZ", "BLTZ", "BGTZ", "BLEZ", "BGEZ", "ADDI", "SUBI", "XORI", "ORI", "ANDI", "JUMP", "EOP", "NOP", "LW", "SW", "LWS", "SWS", "ADDS", "SUBS", "MULTS", "DIVS"};


class sim_ooo{

   typedef struct dynInstructT* dynInstructPT;
   typedef struct instructT* instructPT;

   //Data structure for ROB
   struct robT{
      dynInstructPT   dInstP;
      bool            ready;
      bool            busy;
      unsigned        dest;
      unsigned        value;

      robT(instructPT instructP){
         dInstP  = new dynInstructT(instructP);
         ready   = false;
      }
   }

   //Data structure for Reservation Station
   struct resStationT{
      dynInstructPT   dInstP;
      bool            busy;
      unsigned        pcRs;
      unsigned        vj;
      bool            vjR;
      unsigned        vk;
      bool            vkR;
      int             qj;
      int             qk;
      int             tagD;
      int             addr;
      bool            inExec;

      resStationT(dynInstructPT dynInstP){
         dInstP     = dynInstP;
         busy       = false;
         vjR        = true;
         vkR        = true;
         inExec     = false;
         pcRs       = UNDEFINED;
         vj         = UNDEFINED;
         vk         = UNDEFINED;
         qj         = UNDEFINED;
         qk         = UNDEFINED; 
         tagD       = UNDEFINED; 
         addr       = UNDEFINED;
      }

   }

   //-------------------------------------------------------------------------------//
   struct dynInstructT : public instructT{
      int                t_issue;
      int                t_execute;
      int                t_wr;
      int                t_commit;
      stage_t            state;

      dynInstructT( instructPT input ){
         t_issue          = UNDEFINED;
         t_execute        = UNDEFINED;
         t_wr             = UNDEFINED;
         t_commit         = UNDEFINED;
         state            = ISSUE;
         instructT(input);
      }
   };

   struct instructT{
      opcode_t           opcode;
      uint32_t           dst;
      uint32_t           src1;
      uint32_t           src2;
      uint32_t           imm;
      bool               dstValid;
      bool               src1Valid;
      bool               src2Valid;
      bool               dstF;
      bool               src1F;
      bool               src2F;
      bool               is_stall;
      bool               is_branch;
      bool               is_taken;

      instructT(){
         nop();
      }

      instructT( instructPT input ){
         opcode     = input->opcode; 
         dst        = input->dst; 
         src1       = input->src1; 
         src2       = input->src2; 
         imm        = input->imm; 
         dstValid   = input->dstValid; 
         src1Valid  = input->src1Valid;
         src2Valid  = input->src2Valid;
         is_stall   = input->is_stall; 
         is_branch  = input->is_branch;
         is_taken   = input->is_taken;
         dstF       = input->dstF;
         src1F      = input->src1F;    
         src2F      = input->src2F;    
      }

      void print(){
         cout << "Opcode: " << opcode_str[opcode] << ", dst: " << dst << ", src1: " << src1 << ", src2: " << src2 << ", imm: " << imm << ", dstValid: " << dstValid << ", src1Valid: " << src1Valid << ", src2Valid: " << src2Valid << ", dstF: " << dstF << ", src1F: " << src1F << ", src2F: " << src2F << ", is_stall: " << is_stall << ", is_branch: " << is_branch << endl;
      }

      void nop(){
         opcode     = NOP;
         dst        = UNDEFINED;
         src1       = UNDEFINED;
         src2       = UNDEFINED;
         imm        = UNDEFINED;
         dstValid   = false;
         src1Valid  = false;
         src2Valid  = false;
         is_stall   = false;
         is_branch  = false;
         is_taken   = false;
         dstF       = false;
         src1F      = false;
         src2F      = false;
      }

      void stall(){
         nop();
         is_stall   = true;
      }
   };

   struct gprFileT{
      int            value;
      int            busy;
      int            tag;
   };

   struct fpFileT{
      float          value;
      int            busy;
      int            tag;
   };

   struct execLaneT{
      resStationT*   payloadP;
      int            ttl;

      execLaneT(){
         ttl            = 0;
      }

   };

   struct execUnitT{
      execLaneT      *lanes;
      int            numLanes;
      int            latency;

      execUnitT(){
         lanes          = NULL;
         numLanes       = 0;
         latency        = 0;
      }

      void init(int numLanes, int latency){
         ASSERT( latency > 0, "Impractical latency found (=%d)", latency );
         ASSERT( numLanes > 0, "Unsupported number of lanes (=%d)", numLanes );
         this->numLanes += numLanes;
         this->latency   = latency;
         lanes           = (execLaneT*)realloc(lanes, this->numLanes * sizeof(execLaneT));
      }
   };


   int            cycleCount;
   unsigned       PC;

   instructPT     *instMemory;
   int            instCount;

   gprFileT       gprFile[NUM_GP_REGISTERS];
   fpFileT        fpFile[NUM_FP_REGISTERS];
   execUnitT      execFp[EXEC_UNIT_TOTAL];

   unsigned       data_memory_size;
   unsigned       dataMemSize;

   unsigned       memLatency;
   unsigned       memFlag;
   unsigned       baseAddress;

   vector <resStationT*>  resStation[RS_TOTAL];
   unsigned       resStSize[];

   unsigned       robSize;
   unsigned       issueWidth;

   //----------------------------------------------------------------------------//

   Fifo <robT> *robP;
   public:

   /* Instantiates the simulator
      Note: registers must be initialized to UNDEFINED value, and data memory to all 0xFF values
   */
   sim_ooo(unsigned mem_size, 		// size of data memory (in byte)
         unsigned rob_size, 		// number of ROB entries
         unsigned num_int_res_stations,	// number of integer reservation stations 
         unsigned num_add_res_stations,	// number of ADD reservation stations
         unsigned num_mul_res_stations, 	// number of MULT/DIV reservation stations
         unsigned num_load_buffers,	// number of LOAD buffers
         unsigned issue_width=1		// issue width
         );	

   //de-allocates the simulator
   ~sim_ooo();

   // adds one or more execution units of a given type to the processor
   // - exec_unit: type of execution unit to be added
   // - latency: latency of the execution unit (in clock cycles)
   // - instances: number of execution units of this type to be added
   void init_exec_unit(exe_unit_t exec_unit, unsigned latency, unsigned instances=1);

   //loads the assembly program in file "filename" in instruction memory at the specified address
   void load_program(const char *filename, unsigned base_address=0x0);

   //runs the simulator for "cycles" clock cycles (run the program to completion if cycles=0) 
   void run(unsigned cycles=0);

   //resets the state of the simulator
   /* Note: 
      - registers should be reset to UNDEFINED value 
      - data memory should be reset to all 0xFF values
      - instruction window, reservation stations and rob should be cleaned
      */
   void reset();

   //returns value of the specified integer general purpose register
   int get_int_register(unsigned reg);

   //set the value of the given integer general purpose register to "value"
   void sim_ooo::set_int_register(unsigned reg, int value, int tag, bool busy);

   //returns value of the specified floating point general purpose register
   float get_fp_register(unsigned reg);

   //set the value of the given floating point general purpose register to "value"
   void sim_ooo::set_fp_register(unsigned reg, float value, int tag, bool busy);

   // returns the index of the ROB entry that will write this integer register (UNDEFINED if the value of the register is not pending
   unsigned get_pending_int_register(unsigned reg);

   // returns the index of the ROB entry that will write this floating point register (UNDEFINED if the value of the register is not pending
   unsigned get_pending_fp_register(unsigned reg);

   //returns the IPC
   float get_IPC();

   //returns the number of instructions fully executed
   unsigned get_instructions_executed();

   //returns the number of clock cycles 
   unsigned get_clock_cycles();

   //prints the content of the data memory within the specified address range
   void print_memory(unsigned start_address, unsigned end_address);

   // writes an integer value to data memory at the specified address (use little-endian format: https://en.wikipedia.org/wiki/Endianness)
   void write_memory(unsigned address, unsigned value);

   //prints the values of the registers 
   void print_registers();

   //prints the status of processor excluding memory
   void print_status();

   // prints the content of the ROB
   void print_rob();

   //prints the content of the reservation stations
   void print_reservation_stations();

   //print the content of the instruction window
   void print_pending_instructions();

   //print the whole execution history 
   void print_log();
};

template <class T> 
class Fifo {
   private:
      T*        array;
      uint32_t  head;
      uint32_t  tail;
      uint32_t  count;
      uint32_t  size;

   public:
      Fifo( uint32_t size );
      ~Fifo();

      uint32_t push( T entry );
      T pop( bool &underflow );
      T* peekHead();
      // n is assumed to be indexed from 0
      // 0 means head, count - 1 means tail
      T* peekNth( uint32_t n );
      // Peek a custom physical index
      // NOTE: should be between head and tail
      T* peekIndex( uint32_t index );
      uint32_t getHeadIndex();
      uint32_t getTailIndex();
      uint32_t getCount();
      void popAll();
      // Phase: true if head == tail is to be considered full (head moving against tail)
      //        false if head == tail is empty (head moving towards tail)
      uint32_t getCountHeadTail( uint32_t head, uint32_t tail, bool phase=false );
      void moveHead( uint32_t head, bool phase=false, uint32_t count=-1 );
      void moveTail( uint32_t tail, bool phase=false, uint32_t count=-1 );
      bool isFull();
      bool isEmpty();
      uint32_t getSize();
      uint32_t genIndex( uint32_t ith );
};
#endif /*SIM_OOO_H_*/
