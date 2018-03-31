#include "sim_ooo.h"

using namespace std;

//used for debugging purposes
static const char *stage_names[NUM_STAGES] = {"ISSUE", "EXE", "WR", "COMMIT"};
static const char *instr_names[NUM_OPCODES] = {"LW", "SW", "ADD", "ADDI", "SUB", "SUBI", "XOR", "XORI", "OR", "ORI", "AND", "ANDI", "MULT", "DIV", "BEQZ", "BNEZ", "BLTZ", "BGTZ", "BLEZ", "BGEZ", "JUMP", "EOP", "LWS", "SWS", "ADDS", "SUBS", "MULTS", "DIVS"};
static const char *res_station_names[5]={"Int", "Add", "Mult", "Load"};

map <string, opcode_t> opcode_2str = { {"LW", LW}, {"SW", SW}, {"ADD", ADD}, {"ADDI", ADDI}, {"SUB", SUB}, {"SUBI", SUBI}, {"XOR", XOR}, {"XORI", XORI}, {"OR", OR}, {"ORI", ORI}, {"AND", AND}, {"ANDI", ANDI}, {"MULT", MULT}, {"DIV", DIV}, {"BEQZ", BEQZ}, {"BNEZ", BNEZ}, {"BLTZ", BLTZ}, {"BGTZ", BGTZ}, {"BLEZ", BLEZ}, {"BGEZ", BGEZ}, {"JUMP", JUMP}, {"EOP", EOP}, {"LWS", LWS}, {"SWS", SWS}, {"ADDS", ADDS}, {"SUBS", SUBS}, {"MULTS", MULTS}, {"DIVS", DIVS}};

//------------------------------------convert functions begin--------------------------------------------------------------//
/* convert a float into an unsigned */
inline unsigned float2unsigned(float value){
        unsigned result;
        memcpy(&result, &value, sizeof value);
        return result;
}

/* convert an unsigned into a float */
inline float unsigned2float(unsigned value){
        float result;
        memcpy(&result, &value, sizeof value);
        return result;
}

/* convert integer into array of unsigned char - little indian */
inline void unsigned2char(unsigned value, unsigned char *buffer){
        buffer[0] = value & 0xFF;
        buffer[1] = (value >> 8) & 0xFF;
        buffer[2] = (value >> 16) & 0xFF;
        buffer[3] = (value >> 24) & 0xFF;
}

/* convert array of char into integer - little indian */
inline unsigned char2unsigned(unsigned char *buffer){
       return buffer[0] + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24);
}
//-------------------------------------convert functions end-------------------------------------------------------------//

sim_ooo::sim_ooo(unsigned mem_size,
                unsigned rob_size,
                unsigned num_int_res_stations,
                unsigned num_add_res_stations,
                unsigned num_mul_res_stations,
                unsigned num_load_res_stations,
                unsigned max_issue){

	data_memory_size       = mem_size;
   robSize                = rob_size;
   issueWidth             = max_issue;

   resStSize              = new unsigned[RS_TOTAL];
   resStSize[INTEGER_RS]  = num_int_res_stations;
   resStSize[ADD_RS]      = num_add_res_stations;
   resStSize[MULT_RS]     = num_mul_res_stations;
   resStSize[LOAD_B]      = num_load_res_stations;

   //Allocating issue queue, ROB, reservation stations
	data_memory            = new unsigned char[data_memory_size];
   rob                    = Fifo<robT*>( rob_size );

   reset();
}
	
sim_ooo::~sim_ooo(){
}

void sim_ooo::init_exec_unit(exe_unit_t exec_unit, unsigned latency, unsigned instances){
   execFp[exec_unit].init(instances, latency);
}

void sim_ooo::load_program(const char *filename, unsigned base_address){
   instMemSize               = parse(string(filename), base_address);
   this->PC                  = base_address;
}

instructT sim_ooo::fetchInstruction ( unsigned pc ) {
   int      index     = (pc - this->baseAddress)/4;
   ASSERT((index >= 0) && (index < instMemSize), "out of bound access of instruction memory %d", index);
   instructT instruct = *(instMemory[index]);
   if(instruct.opcode != EOP)
      instCount++;
   return instruct;
}

// The following function is for IF + ID + RR
void sim_ooo::fetch(){
    for (int j = 0; j < issueWidth && rob.isFull(); j++){
      //fetching instruction according to PC
      instructT instruct     = fetchInstruction ( PC );
      //finding the execution unit of the opcode
      exe_unit_t unit        = opcodeToExUnit(instruct.opcode);

      // Assert for overflown reservation station
      ASSERT(resStation[unit].size() <= resStSize[unit] , "Illegal resStation size found (%lu > %u)", resStation[unit].size(), resStSize[unit]);

      //Checking if reservation station is not full 
      if (resStation[unit].size() != resStSize[unit]) {
         robT* robEntryP        = new robT(&instruct);
         robEntryP->busy        = true;
         if(instruct.is_store)
            robEntryP->memLatency = execFp[MEMORY].latency;
         uint32_t robIndex      = rob.push(robEntryP);

         resStationT* resP      = new resStationT(robEntryP->dInstP);
         resP->pcRs             = PC;
         resP->vj               = (instruct.src1Valid) ? regRead(instruct.src1, instruct.src1F) : UNDEFINED;
         resP->vk               = (instruct.src2Valid) ? regRead(instruct.src2, instruct.src2F) : UNDEFINED;
         resP->qj               = (instruct.src1Valid) ? regTag(instruct.src1, instruct.src1F)  : UNDEFINED;
         resP->qk               = (instruct.src2Valid) ? regTag(instruct.src2, instruct.src2F)  : UNDEFINED;
         resP->tagD             = robIndex;

         //setting bits for values ready/not ready
         if(instruct.src1Valid && regBusy(instruct.src1, instruct.src1F)) 
            resP->vjR  = false; 
         if(instruct.src2Valid && regBusy(instruct.src2, instruct.src2F))
            resP->vkR  = false; 

         //Updating address field of reservation station entry according to memory unit
         if( unit == MEMORY )
            resP->addr          = instruct.imm;

         // Add an entry in reservation station
         resStation[unit].push_back( resP );

         //incrementing PC only if ROB and RS are not full
         PC                     = PC + 4;

         //update TAG at register File with ROB entry if destination exists
         if(instruct.dstValid){
            if(instruct.dstF)
               set_fp_reg_tag(instruct.dst, robIndex, true); 
            else
               set_int_reg_tag(instruct.dst, robIndex, true); 
         }

         break;
      }
      //Break if Branch to create a basic block
      //Since BP = always not taken, do nothing
   }
}

// The following function is for IS
void sim_ooo::dispatch(){
   //To iterate through reservation station units
   for(int unit = 0; unit < RS_TOTAL; unit++) {
      //To iterate through individual units
      for(unsigned payIndex = 0; payIndex < resStation[unit].size(); payIndex++) {
         //Checking if both operands are ready, hence instruction is ready and it is not in execute stage
         resStationT* resP = resStation[unit][payIndex];

         bool instReady        = true;
         bool bypassReady      = false;
         uint32_t bypassValue  = UNDEFINED;
         bool is_store         = resP->dInstP->is_store;
         bool is_load          = resP->dInstP->is_load;
         uint32_t addr         = agen(*(resP->dInstP));

         if( is_load ){
            instReady      = isConflictingStore(resP->tagD, addr, bypassReady, bypassValue);
         } 

         if ( !resP->inExec && resP->vjR && resP->vkR && instReady ){
            int execUnit   = opcodeToExUnit(resP->dInstP->opcode);
            for(int laneId = 0; laneId < execFp[execUnit].numLanes; laneId++){
               //checking for free execution units
               if(execFp[execUnit].lanes[laneId].ttl == 0) {
                  resP->inExec                            = true;
                  execFp[execUnit].lanes[laneId].payloadP = resP;
                  // How much time will the operation take to complete
                  // 1. Stores take 1 cycle
                  // 2. Bypassed loads take 1 cycle
                  // 3. Remaining takes set cycles
                  uint32_t ttl                            = (is_store ? 1 : ((is_load && bypassReady) ? 1 : execFp[execUnit].latency) );
                  // Adding 1 to model 1 unit latency in Write Result
                  execFp[execUnit].lanes[laneId].ttl      = ttl + 1;
                  
                  // Setting up outputs
                  execFp[execUnit].lanes[laneId].outputReady = is_load && bypassReady;
                  execFp[execUnit].lanes[laneId].output      = (is_load && bypassReady) ? bypassValue : UNDEFINED;
                  if( is_store ){
                     // ROB is acting as a store buffer
                     // Update addr in ROB
                     rob.peekIndex( resP->tagD )->dest    = addr;
                  }
                  break;
               }
            }
         }
      }
   }
}

//checking for conflicting store with a load instruction
bool sim_ooo::isConflictingStore(int loadTag, unsigned memAddress, bool& bypassReady, uint32_t& bypassValue ){
   bool conflict               = false;
   bypassReady                 = false;
   for(int i = 0; i < rob.getCount(); i++){
      //getting the current tag
      int tag                  = rob.genIndex(i);
      //Get the ROB entry
      robT* robEntryP          = rob.peekNth(i);

      //checking if the opcode is store
      if( robEntryP->dInstP->is_store ){
         //if the store is not complete (a.k.a ??), then there is a conflict
         if(!robEntryP->ready){
            conflict           = true;
            bypassReady        = false;
         }
         //if store is complete and match the address, no conflict.
         //values are stored from this store to load temporarily
         else if(robEntryP->dest == memAddress){
            conflict           = false;
            bypassReady        = true;
            bypassValue        = robEntryP->value;
         }
      }
      if(tag == loadTag)
         //returns the most recent conflict entry
         return conflict;
   }
}

//-------------------------------issue stage begin-----------------------------------------------------------//
void sim_ooo::issue() {
   fetch();
   dispatch();
}
//-------------------------------issue stage end-------------------------------------------------------------//
//-------------------------------------------------------------execute stage---------------------------------//
void sim_ooo::execute(){
   //iterating through execution units
   for(int i = 0; i < EX_TOTAL; i++){
      //iterating through number of instances of an EXEC UNIT
      for(int j = 0; j < execFp[i].numLanes; j++){
         execWrLaneT* laneP = &(execFp[i].lanes[j]);

         //Checking if TTL of the lane is not zero
         if( laneP->ttl != 0 ) {
            laneP->ttl--;

            //local variable for payloadP in exec unit
            resStationT* resP             = laneP->payloadP;
            // 1 implies Write Result
            if( laneP->ttl == 1 ) {
               // It's time to execute!!
               if( !laneP->outputReady )
                  laneP->output           = aluGetOutput(resP->dInstP, rob.peekIndex( resP->tagD )->misPred);
               laneP->outputReady         = true;

               // vk has to be updated for all loads
               if( resP->dInstP->is_load )
                  resP->vk                = laneP->output;
            }
         } 
      }
   }
}

uint32_t sim_ooo::aluGetOutput(dynInstructT* dInstP, bool& misPred){
   uint32_t src1   = dInstP->src1;
   uint32_t src2   = dInstP->src2;
   bool src1F      = dInstP->src1F;
   bool src2F      = dInstP->src2F;
   opcode_t opcode = dInstP->opcode;

   switch(opcode) {
      case LW ... SWS:
         return read_memory (resP->addr);
         break;

      case ADD ... DIV:
      case ADDS ... DIVS:
         return alu(regRead(src1, src1F), regRead(src2, src2F), src1F, src2F, opcode);
         break;

      case ADDI ... ANDI:
         return alu(regRead(src1, src1F), instruct.imm, src1F, false, opcode);
         break;

      case BLTZ:
         misPred = regRead(src1, src1F) < 0;
         return alu(npc, instruct.imm, false, false, opcode);
         break;

      case BNEZ:
         misPred = regRead(src1, src1F) != 0;
         return alu(npc, instruct.imm, false, false, opcode);
         break;

      case BEQZ:
         misPred = regRead(src1, src1F) == 0;
         return alu(npc, instruct.imm, false, false, opcode);
         break;

      case BGTZ:
         misPred = regRead(src1, src1F) > 0;
         return alu(npc, instruct.imm, false, false, opcode);
         break;

      case BGEZ:
         misPred = regRead(src1, src1F) >= 0;
         return alu(npc, instruct.imm, false, false, opcode);
         break;

      case BLEZ:
         misPred = regRead(src1, src1F) <= 0;
         return alu(npc, instruct.imm, false, false, opcode);
         break;

      case JUMP:
         misPred = true;
         return alu(npc, instruct.imm, false, false, opcode);
         break;

      case NOP:
      case EOP:
         break;

      default:
         ASSERT(false, "Unknown operation encountered");
         return UNDEFINED;
         break;
   }
   return UNDEFINED;
}
//-----------------------------------WRITE RESULT STAGE MOSTLY---------------------------------------------//
void sim_ooo::writeResult(){
   for(int i = 0; i < EX_TOTAL; i++){
      for(int j = 0; j < execFp[i].numLanes; j++){
         if(execFp[i].lanes[j].ttl == 1){
            resStationT* resP         = execFp[i].lanes[j].payloadP;
            execWrLaneT* laneP        = &(execFp[i].lanes[j]);
            ASSERT( laneP->outputReady, "At WriteResult, output not ready!" );

            int resDelIndex           = -1;
            res_station_t resDelUnit;
            //wake up all res stations by searching for tagD
            for( int unit = 0; unit < RS_TOTAL; unit++ ){
               for(int k = 0; k < resStation[unit].size(); k++) {

                  if( resP->tagD == resStation[unit][k].tagD ){
                     resDelIndex      = k;
                     resDelUnit       = unit;
                  }

                  if(resStation[unit][k].qj == resP->tagD) {
                     resStation[unit][k].vj = laneP->output;
                     resStation[unit][k].qj = UNDEFINED;
                  }
                  if(resStation[unit][k].qk == resP->tagD) {
                     resStation[unit][k].vk = laneP->output;
                     resStation[unit][k].qk = UNDEFINED;
                  }
               }
            }
            // updating ROB
            rob.peekIndex( resP->tagD )->value = laneP->output;
            rob.peekIndex( resP->tagD )->ready = true;

            //remove entry from res station
            ASSERT( resDelIndex != -1, "resDelIndex == -1" );
            delete resStation[resDelUnit][resDelUnit];
            resStation[resDelUnit].erase( resDelIndex );
         }
      }
   }
}

void sim_ooo::commit(){
   for(int i = 0; i < issueWidth && !rob.isEmpty(); i++){
      robT* head       = rob.peekHead();
      uint32_t headTag = rob.getHeadIndex();
      if(head->ready){
         //--------------- STORE ---------------
         if(head->dInstP->is_store) {
            if(head->memLatency != 0){
               head->memLatency--;
               break;
            }
            else
               write_memory(head->value, head->dest);
         }

         bool gSquash = head->dInstP->is_branch && head->misPred;
         uint32_t npc = head->value;
       
         // Update RF
         if(head->dInstP->dstValid){
            if(head->dInstP->dstF){
               fpFile[head->dInstP->dst].value    = head->value;
               // Clear busy if the latest tag in RF is being committed
               if( headTag == fpFile[head->dInstP->dst].tag )
                  fpFile[head->dInstP->dst].busy  = false; 
            } 
            else {
               gprFile[head->dInstP->dst].value   = head->value;
               // Clear busy if the latest tag in RF is being committed
               if( headTag == gprFile[head->dInstP->dst].tag )
                  gprFile[head->dInstP->dst].busy = false; 
            }
         }

         // Commit
         bool underflow;
         delete rob.pop(underflow);
         ASSERT(!underflow, "ROB underflown");

         //--------------- BRANCH --------------
         if(gSquash){
            PC = npc;
            squash();
            break;
         }
      }
   }
}

//---------------------------------------------------------------------------------------------------------//

void sim_ooo::run(unsigned cycles){
}

//reset the state of the sim_oooulator
void sim_ooo::reset(){
   for(unsigned i = 0; i < data_memory_size; i++) {
      data_memory[i]   = UNDEFINED; 
   }

   //initializing GPRs to UNDEFINED
   for(int i = 0; i < NUM_GP_REGISTERS; i++) {
      gprFile[i].value = UNDEFINED;
   }

   //initializing FP registers to UNDEFINED
   for(int i = 0; i < NUM_FP_REGISTERS; i++) {
      fpFile[i].value  = UNDEFINED;
   }
   // Squash/Flush the pipeline
   squash();
}

void sim_ooo::squash(){
   //Clearing Res Station
   for(int i = 0; i < RS_TOTAL; i++){
      resStation[i].clear();
   }
   //Clearing ROB
   rob.popAll();

   // Flash clear busy bits
   for(int i = 0; i < NUM_FP_REGISTERS; i++) {
      fpFile[i].busy  = false;
   }

   for(int i = 0; i < NUM_GP_REGISTERS; i++) {
      gprFile[i].busy = false;
   }
}

//--------------------------------------- IMPORTANT FUNCTIONS ---------------------------------------------//

bool sim_ooo::regBusy(uint32_t regNo, bool isF) {
   return isF ? fpFile[regNo].busy : gprFile[regNo].busy;
}

exe_unit_t sim_ooo::opcodeToExUnit(opcode_t opcode){
   exe_unit_t unit;
   switch( opcode ){
      case ADD ... AND:
      case BEQZ ... SWS:
         unit = INTEGER;
         break;

      case ADDS:
      case SUBS:
         unit = ADDER;
         break;

      case MULTS:
      case MULT:
         unit = MULTIPLIER;
         break;

      case DIVS:
      case DIV:
         unit = DIVIDER;
         break;

      case LW:
      case LWS:
      case SW:
      case SWS:
         unit = MEMORY;

      default: 
         ASSERT (false, "Opcode not supported");
         unit = INTEGER;
         break;
   }
   ASSERT( execFp[unit].numLanes > 0, "No lanes found for opcode: %s", opcode_str[opcode].c_str());
   return unit;
}

int sim_ooo::exLatency(opcode_t opcode) {
   return execFp[opcodeToExUnit(opcode)].latency;
}

uint32_t sim_ooo::agen ( instructT instruct) {
   return (instruct.imm + regRead(instruct.src1, instruct.src1F));
}

//ALU function for floating point operations
unsigned sim_ooo::aluF (unsigned _value1, unsigned _value2, bool value1F, bool value2F, opcode_t opcode){
   float output;
   float value1 = value1F ? unsigned2float(_value1) : _value1;
   float value2 = value2F ? unsigned2float(_value2) : _value2; 

   switch( opcode ){
      case ADD:
      case BEQZ ... ADDI:
      case JUMP ... ADDS:
         output      = value1 + value2;
         break;

      case SUB:
      case SUBI:
      case SUBS:
         output      = value1 - value2;
         break;

      case XOR:
      case XORI:
         output      = (unsigned)value1 ^ (unsigned)value2;
         break;

      case AND:
      case ANDI:
         output      = (unsigned)value1 & (unsigned)value2;
         break;

      case OR:
      case ORI:
         output      = (unsigned)value1 | (unsigned)value2;
         break;

      case MULT:
      case MULTS:   
         output      = value1 * value2;
         break;

      case DIV:
      case DIVS:
         output      = value1 / value2;
         break;

      default: 
         output      = UNDEFINED;
         break;
   }
   return float2unsigned(output);
}

//function to perform ALU operations
unsigned sim_ooo::alu (unsigned _value1, unsigned _value2, bool value1F, bool value2F, opcode_t opcode){

   if( value1F || value2F ) return aluF(_value1, _value2, value1F, value2F, opcode);

   unsigned output;
   unsigned value1   = value1F ? float2unsigned(_value1) : _value1;
   unsigned value2   = value2F ? float2unsigned(_value2) : _value2; 

   switch( opcode ){
      case ADD:
      case BEQZ ... ADDI:
      case JUMP ... ADDS:
         output      = value1 + value2;
         break;

      case SUB:
      case SUBI:
      case SUBS:
         output      = value1 - value2;
         break;

      case XOR:
      case XORI:
         output      = (unsigned)value1 ^ (unsigned)value2;
         break;

      case AND:
      case ANDI:
         output      = (unsigned)value1 & (unsigned)value2;
         break;

      case OR:
      case ORI:
         output      = (unsigned)value1 | (unsigned)value2;
         break;

      case MULT:
      case MULTS:   
         output      = value1 * value2;
         break;

      case DIV:
      case DIVS:
         output      = value1 / value2;
         break;

      default: 
         output      = UNDEFINED;
         break;
   }
   return output;
}

//---------------------------------------------------------------------------------------------------------//

//------------------------------------SETTER/GETTER functions----------------------------------------------//

//TODO check for better way to call value/tag using same function
unsigned sim_ooo::regRead(unsigned reg, bool isF){
   if(regBusy(reg, isF)) return UNDEFINED;
   return isF ? float2unsigned(fpFile[reg].value) : gprFile[reg].value;
}
unsigned sim_ooo::regTag(unsigned reg, bool isF){
   if(regBusy(reg, isF)) return isF ? fpFile[reg].tag : gprFile[reg].tag;
   return UNDEFINED;
}

int sim_ooo::get_int_register(unsigned reg){
	return gprFile[reg].value; 
}

//TODO: Check if this is necessary
void sim_ooo::set_int_register(unsigned reg, int value){
   gprFile[reg].value = value;
}

float sim_ooo::get_fp_register(unsigned reg){
	return fpFile[reg].value;
}

//TODO: Check if this is necessary
void sim_ooo::set_fp_register(unsigned reg, float value){
   fpFile[reg].value = value;
}
void sim_ooo::set_fp_reg_tag(unsigned reg, int tag, bool busy){
   fpFile[reg].tag   = tag;
   fpFile[reg].busy  = busy;
}

void sim_ooo::set_int_reg_tag(unsigned reg, int tag, bool busy){
   gprFile[reg].tag   = tag;
   gprFile[reg].busy  = busy;
}

unsigned sim_ooo::get_pending_int_register(unsigned reg){
	return UNDEFINED; //fill here
}

unsigned sim_ooo::get_pending_fp_register(unsigned reg){
	return UNDEFINED; //fill here
}
//-------------------------------------------------------------------------------------------------------//

void sim_ooo::print_status(){
	print_pending_instructions();
	print_rob();
	print_reservation_stations();
	print_registers();
}

void sim_ooo::print_memory(unsigned start_address, unsigned end_address){
	cout << "DATA MEMORY[0x" << hex << setw(8) << setfill('0') << start_address << ":0x" << hex << setw(8) << setfill('0') <<  end_address << "]" << endl;
	for (unsigned i=start_address; i<end_address; i++){
		if (i%4 == 0) cout << "0x" << hex << setw(8) << setfill('0') << i << ": "; 
		cout << hex << setw(2) << setfill('0') << int(data_memory[i]) << " ";
		if (i%4 == 3){
			cout << endl;
		}
	} 
}

//---------------------------READ AND WRITE MEMORY FUNCTIONS BEGIN--------------------------------------//

void sim_ooo::write_memory(unsigned address, unsigned value){
   ASSERT( address % 4 == 0, "Unaligned memory access found at address %x", address ); 
   ASSERT ( (address >= 0) && (address < data_memory_size), "Out of bounds memory accessed: Seg Fault!!!!" );
	unsigned2char(value,data_memory+address);
}

unsigned sim_ooo::read_memory(unsigned address){
   ASSERT( address % 4 == 0, "Unaligned memory access found at address %x", address ); 
   ASSERT ( (address >= 0) && (address < data_memory_size), "Out of bounds memory accessed: Seg Fault!!!!" );
   return char2unsigned(data_memory+address);
}
//---------------------------READ AND WRITE MEMORY FUNCTIONS END----------------------------------------//

//------------------------------------------------------------------------------------------------------//
void sim_ooo::print_registers(){
        unsigned i;
	cout << "GENERAL PURPOSE REGISTERS" << endl;
	cout << setfill(' ') << setw(8) << "Register" << setw(22) << "Value" << setw(5) << "ROB" << endl;
        for (i=0; i< NUM_GP_REGISTERS; i++){
                if (get_pending_int_register(i)!=UNDEFINED) 
			cout << setfill(' ') << setw(7) << "R" << dec << i << setw(22) << "-" << setw(5) << get_pending_int_register(i) << endl;
                else if (get_int_register(i)!=(int)UNDEFINED) 
			cout << setfill(' ') << setw(7) << "R" << dec << i << setw(11) << get_int_register(i) << hex << "/0x" << setw(8) << setfill('0') << get_int_register(i) << setfill(' ') << setw(5) << "-" << endl;
        }
	for (i=0; i< NUM_GP_REGISTERS; i++){
                if (get_pending_fp_register(i)!=UNDEFINED) 
			cout << setfill(' ') << setw(7) << "F" << dec << i << setw(22) << "-" << setw(5) << get_pending_fp_register(i) << endl;
                else if (get_fp_register(i)!=UNDEFINED) 
			cout << setfill(' ') << setw(7) << "F" << dec << i << setw(11) << get_fp_register(i) << hex << "/0x" << setw(8) << setfill('0') << float2unsigned(get_fp_register(i)) << setfill(' ') << setw(5) << "-" << endl;
	}
	cout << endl;
}

void sim_ooo::print_rob(){
	cout << "REORDER BUFFER" << endl; 
	cout << setfill(' ') << setw(5) << "Entry" << setw(6) << "Busy" << setw(7) << "Ready" << setw(12) << "PC" << setw(10) << "State" << setw(6) << "Dest" << setw(12) << "Value" << endl;
	
	//fill here
	
	cout << endl;
}

void sim_ooo::print_reservation_stations(){
	cout << "RESERVATION STATIONS" << endl;
	cout  << setfill(' ');
	cout << setw(7) << "Name" << setw(6) << "Busy" << setw(12) << "PC" << setw(12) << "Vj" << setw(12) << "Vk" << setw(6) << "Qj" << setw(6) << "Qk" << setw(6) << "Dest" << setw(12) << "Address" << endl; 
	
	// fill here
	
	cout << endl;
}

void sim_ooo::print_pending_instructions(){
	cout << "PENDING INSTRUCTIONS STATUS" << endl;
	cout << setfill(' ');
	cout << setw(10) << "PC" << setw(7) << "Issue" << setw(7) << "Exe" << setw(7) << "WR" << setw(7) << "Commit";
	cout << endl;
}

//-------------------------------------------------------------------------------------------------------------------------------------------//
void sim_ooo::print_log(){
}

float sim_ooo::get_IPC(){
   return (double) get_instructions_executed() / (double) cycleCount;
}
	
unsigned sim_ooo::get_instructions_executed(){
	return instCount; 
}

unsigned sim_ooo::get_clock_cycles(){
   return cycleCount; 
}

//-------------------------------- Fifo FUNCS BEGIN -------------------------------
template <class T> Fifo<T>::Fifo( uint32_t size ){
   this->head              = 0;
   this->tail              = 0;
   this->count             = 0;
   this->size              = size;
   this->array             = new T[ size ];
}

template <class T> Fifo<T>::Fifo(){
   Fifo(0);
}

template <class T> Fifo<T>::~Fifo(){
   delete array;
}

template <class T> uint32_t Fifo<T>::push( T entry ){
   // Push only if space is available
   // Overflow check condition
   uint32_t push_index = -1;
   if( count < size ){
      // Place data on tail and increment tail and count
      array[ tail ]        = entry;
      push_index           = tail;
      tail                 = ( tail + 1 ) % size;
      count++;
      // Un-necessary assert to check fifo sanity
      assert( count == getCountHeadTail( head, tail, true ) );
   }
   assert( push_index != -1 );
   return push_index;
}

template <class T> T Fifo<T>::pop( bool &underflow ){
   // return data from head, increment head and decrement count
   T entry      = array[ head ];
   // Underflow condition check
   if( count > 0 ){
      head      = ( head + 1 ) % size;
      count--;
      underflow = false;
   } else{
      underflow = true;
   }
   // Un-necessary assert to check fifo sanity
   assert( count == getCountHeadTail( head, tail ) );

   // FIXME: replace
   assert( !underflow );
   return entry;
}

template <class T> T* Fifo<T>::peekHead(){
   return &( array[ head ] );
}

// n is assumed to be indexed from 0
// 0 means head, count - 1 means tail
template <class T> T* Fifo<T>::peekNth( uint32_t n ){
   assert( n < count );
   uint32_t index = ( head + n ) % size;
   return &( array[ index ] );
}

// Peek a custom physical index
// NOTE: should be between head and tail
// TODO: verify
template <class T> T* Fifo<T>::peekIndex( uint32_t index ){
   assert( index < size );
   // ----- head ---- index ----- tail -----
   // -- index --- tail --------- head -----
   // ----- tail -------- head ---- index --
   assert( (index >= head && index <= tail && tail >= head)  || 
         (tail >= index && head >= tail && head >= index)  ||
         (head >= tail && index >= head && index >= tail) );
   return &( array[ index ] );
}

template <class T> uint32_t Fifo<T>::getHeadIndex(){
   return head;
}

template <class T> uint32_t Fifo<T>::getTailIndex(){
   return tail;
}

template <class T> uint32_t Fifo<T>::getCount(){
   return count;
}

template <class T> void Fifo<T>::popAll(){
   head           = tail;
   count          = 0;
}

// Phase: true if head == tail is to be considered full
//        false if head == tail is empty
template <class T> uint32_t Fifo<T>::getCountHeadTail( uint32_t head, uint32_t tail, bool phase){
   uint32_t count = ( tail > head ) ? ( tail - head ) : ( size - head + tail );
   return ( head == tail ) ? ( phase ? size : 0 ) : count;
}

// Phase: true if head == tail is to be considered full
//        false if head == tail is empty
template <class T> void Fifo<T>::moveHead( uint32_t head, bool phase, uint32_t count ){
   head           = head % size;
   uint32_t temp_count = getCountHeadTail( head, tail, phase );

   // Verify head pointer movement if requested (count != -1)
   assert( ( tail == head && ( count == 0 || count == size ) ) || count == -1 || temp_count == count );

   this->head     = head;
   this->count    = temp_count;
}

// Phase: true if head == tail is to be considered full
//        false if head == tail is empty
template <class T> void Fifo<T>::moveTail( uint32_t tail, bool phase, uint32_t count ){
   tail                = tail % size;
   uint32_t temp_count = getCountHeadTail( head, tail, phase );

   // Verify tail pointer movement if requested (count != -1)
   assert( ( tail == head && ( count == 0 || count == size ) ) || count == -1 || temp_count == count );

   this->tail     = tail;
   this->count    = temp_count;
}

template <class T> bool Fifo<T>::isFull(){
   return ( count == size );
}

template <class T> uint32_t Fifo<T>::genIndex( uint32_t ith ){
   return (head + ith) % size;
}

template <class T> bool Fifo<T>::isEmpty(){
   return ( count == 0 );
}

template <class T> uint32_t Fifo<T>::getSize(){
   return size;
}
//--------------------------------- Fifo FUNCS END---------------------------------

//----------------------------------------------PARSING OPERATION BEGINS---------------------------------//
int sim_ooo::indexToOffset( uint32_t line_index, uint32_t pc_index ){
   return ((line_index - pc_index - 1) * 4);
}

int sim_ooo::labelResolve(string label, 
                            map <string, int>& label_to_linenum,
                            map <string, vector <int>>& unresolved_label_index,
                            int line_num ){
   // (2) Check label in storage when branch is encoutered
   if( label_to_linenum.find(label) != label_to_linenum.end() ){
      // Label is found! Immediately compute offset (3)
      return indexToOffset( label_to_linenum[label], line_num );
   } else{
      // Label is not found! Store in vector (4) 
      unresolved_label_index[label].push_back( line_num );
   }
   return UNDEFINED;
}

void sim_ooo::getReg( istringstream& buff_iss, uint32_t& reg, bool& regF, bool with_brackets ){
   string reg_i_or_f, open_bracket, closed_bracket;

   if( with_brackets )
      buff_iss >> setw(1) >> open_bracket >> setw(1) >> reg_i_or_f >> reg >> closed_bracket;
   else
      buff_iss >> setw(1) >> reg_i_or_f >> reg;

   regF  = ( reg_i_or_f == "F" ) || ( reg_i_or_f == "f" );
}

/*
 * Details     : 1. Parses a given file using c++ ifstream, 
 *                  delimits string based on opcode and its
 *                  corresponding arguments
 *               2. Implements runtime label disambiguation
 *                  process for fast label to PC/offset
 *                  lookup
 *                  Disambiguation details:
 *                  1. Each label encountered in runtime,
 *                     is saved onto a map along with its
 *                     line number
 *                  2. When a branch is encountered, its
 *                     label is checked in the storage
 *                     created in (1)
 *                  3. If the label is found, offset is
 *                     updated immediately
 *                  4. If not found, its index is held in 
 *                     a vector
 *                  5. When a new label is processed, we
 *                     check for all previous encounters 
 *                     using vector in (4) and then
 *                     disambiguate
 * Returns     : Number of instructions successfully parsed
 * Side Effects: Populates class variable instMemory
 *
 * NOTES       : -NA-
 */
int sim_ooo::parse( const string filename, unsigned base_address ){
   int line_num = 0;
   string buff;

   // ASM handle to file
   ifstream asm_h;
   asm_h.open( filename, ifstream::in );
   ASSERT( asm_h.is_open(), "Unable to open file: %s", filename.c_str() ); 

   // Map for label to line number lookup used in offset
   // calculation (1)
   map <string, int> label_to_linenum;

   // Store all index to instMemory that have unresolved labels (4)
   map <string, vector <int>> unresolved_label_index;

   // Loop to iterate through each line in asm and extract
   // instructions
   // getline fetches string till a new line character is reached
   while( getline( asm_h, buff ) ) {
      instMemory               = (instructPT*) realloc(instMemory, (line_num + 1)*sizeof(instructPT));
      instructPT instructP     = new instructT;
      instructP->pc            = (line_num * 4) + base_address;

      instMemory[line_num]     = instructP;

      // At this point in code, buff has the entire line including
      // opcode and its arguments (eg. ADDI	R2 R0 0xA000)
      
      // Making istringstream out of buff gives us the capability
      // to extract strings delimited by whitespace easily
      // for ADDI	R2 R0 0xA000, istringstream would yeild 4 strings
      // namely, ADDI, R2, R0, 0xA000 which can be quite handy
      // NOTE: We cannot use extraction operator (described below)
      // on strings and hence, we use istringstream
      istringstream buff_iss(buff);

      // String to hold opcode
      string opcode;
      // String to hold info on integer/floating identifier
      // in instruction
      string label;
      string imm;

      // Loading up 1st string a.k.a opcode from buff_iss
      // >> is also called 'extraction' operator which extracts
      // 1st string out of buff_iss and assigns it to opcode
      // NOTE: opcode will be moved (i.e., will be removed from buff_is)
      buff_iss >> opcode;

      if( opcode_2str.count( opcode ) <= 0 ){
         // string.back() gives the last character in a string
         ASSERT( opcode.back() == ':', "Unkown 1st token(%s) encountered", opcode.c_str() );

         // At this point in code, opcode actually stores label except for last
         // character which is a ':'
         string label              = opcode.substr(0, opcode.length() - 1);

         // label must be saved along with line number for quick 
         // lookup and disambiguation process (1)
         label_to_linenum[ label ] = line_num;

         // Since we have got a new label, we should check for any ambiguities
         // and resolve them at this step (5)
         // NOTE: opcode here is the label
         if( unresolved_label_index.find( label ) != unresolved_label_index.end() ){
            // We do have some unresolved instructions..
            vector <int> unresolved_index = unresolved_label_index[label];
            for( int index = 0; index < unresolved_index.size(); index++ ){
               // Disambiguate all previous encounters
               int inst_index              = unresolved_index[index];
               instMemory[inst_index]->imm = indexToOffset( line_num, inst_index );
            }
            // Delete the entry from unresolved list
            unresolved_label_index.erase( label );
         }

         // Since the assumed opcode was a label, let's extract next token
         // and assign it to opcode
         buff_iss >> opcode;
      }

      // Translate string to enum for handy usage
      instructP->opcode        = opcode_2str[ opcode ];

      //FIX_ME #1: use reg_i_or_f to flood struct field
      // setw(n) sets the number of characters to extract
      // eg., let's assume buff_iss has "R2 R0 0xA000"
      // buff_iss >> setw(1) >> reg_i_or_f
      // will extract R (1 character) from "R2" and stash
      // it in reg_i_or_f
      switch( instructP->opcode ){
         case ADD ... DIV:
         case ADDS ... DIVS:
            // The following 3 lines store respective I/F in
            // intermediate values of reg_i_or_f and converts
            // string to int implicitly as dst, src1, src2 
            // are of type int
            getReg( buff_iss, instructP->dst , instructP->dstF );
            getReg( buff_iss, instructP->src1, instructP->src1F );
            getReg( buff_iss, instructP->src2, instructP->src2F );
            instructP->dstValid   = true;
            instructP->src1Valid  = true;
            instructP->src2Valid  = true;
            break;

         case BEQZ ... BGEZ:
            getReg( buff_iss, instructP->src1, instructP->src1F );
            buff_iss >> label;
            instructP->imm        = labelResolve( label, label_to_linenum, unresolved_label_index, line_num );
            instructP->src1Valid  = true;
            instructP->is_branch  = true;
            break;

         case ADDI ... ANDI:
            getReg( buff_iss, instructP->dst , instructP->dstF );
            getReg( buff_iss, instructP->src1, instructP->src1F );
            buff_iss >> imm;
            // stod takes care of decimal, hex (0x) string internally
            instructP->imm        = stod(imm);
            instructP->dstValid   = true;
            instructP->src1Valid  = true;
            break;

         case JUMP:
            buff_iss >> label;
            instructP->imm        = labelResolve( label, label_to_linenum, unresolved_label_index, line_num );
            instructP->is_branch  = true;
            break;

         case LW:
         case LWS:
            getReg( buff_iss, instructP->dst , instructP->dstF );
            // Format to parse at this point of code: %d(R%d)
            // Interestingly, ->imm field is of type int
            // extracting from buff_iss onto ->imm will stop right
            // before "(" which is exactly what we want
            buff_iss >> instructP->imm;
            // Following line is self explanatory on what is happening
            // at this point in code
            // Format to parse at this point of code: (R%d)
            getReg( buff_iss, instructP->src1, instructP->src1F, true );
            instructP->dstValid   = true;
            instructP->src1Valid  = true;
            instructP->is_load    = true;
            break;

         case SW:
         case SWS:
            getReg( buff_iss, instructP->src2, instructP->src2F );
            // Format to parse at this point of code: %d(R%d)
            // Interestingly, ->imm field is of type int
            // extracting from buff_iss onto ->imm will stop right
            // before "(" which is exactly what we want
            buff_iss >> instructP->imm;
            // Following line is self explanatory on what is happening
            // at this point in code
            // Format to parse at this point of code: (R%d)
            getReg( buff_iss, instructP->src1, instructP->src1F, true );
            instructP->src2Valid  = true;
            instructP->src1Valid  = true;
            instructP->is_store   = true;
            break;

         case EOP:
         case NOP:
            break;

         default:
            ASSERT(false, "Unknown operation encountered");
            break;
      }
      line_num++;
   }

   // At this point in code, there should be no ambiguity in labels
   uint32_t disambiguities = unresolved_label_index.size();
   ASSERT( disambiguities == 0, "Unable to disambiguate successfully (=%u)", disambiguities);
   return line_num;
}

//----------------------------------------------PARSING OPERATION ENDS-----------------------------------//



