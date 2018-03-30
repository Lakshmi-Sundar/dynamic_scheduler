#include "sim_ooo.h"

using namespace std;

//used for debugging purposes
static const char *stage_names[NUM_STAGES] = {"ISSUE", "EXE", "WR", "COMMIT"};
static const char *instr_names[NUM_OPCODES] = {"LW", "SW", "ADD", "ADDI", "SUB", "SUBI", "XOR", "XORI", "OR", "ORI", "AND", "ANDI", "MULT", "DIV", "BEQZ", "BNEZ", "BLTZ", "BGTZ", "BLEZ", "BGEZ", "JUMP", "EOP", "LWS", "SWS", "ADDS", "SUBS", "MULTS", "DIVS"};
static const char *res_station_names[5]={"Int", "Add", "Mult", "Load"};
map <string, opcode_t> opcode_2str = { {"LW", LW}, {"SW", SW}, {"ADD", ADD}, {"ADDI", ADDI}, {"SUB", SUB}, {"SUBI", SUBI}, {"XOR", XOR}, {"XORI", XORI}, {"OR", OR}, {"ORI", ORI}, {"AND", AND}, {"ANDI", ANDI}, {"MULT", MULT}, {"DIV", DIV}, {"BEQZ", BEQZ}, {"BNEZ", BNEZ}, {"BLTZ", BLTZ}, {"BGTZ", BGTZ}, {"BLEZ", BLEZ}, {"BGEZ", BGEZ}, {"JUMP", JUMP}, {"EOP", EOP}, {"NOP", NOP}, {"LWS", LWS}, {"SWS", SWS}, {"ADDS", ADDS}, {"SUBS", SUBS}, {"MULTS", MULTS}, {"DIVS", DIVS}};

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

   resStSize              = new int[RS_TOTAL];
   resStSize[INTEGER_RS]  = num_int_res_stations;
   resStSize[ADD_RS]      = num_add_res_stations;
   resStSize[MULT_RS]     = num_mul_res_stations;
   resStSize[LOAD_B]      = num_load_res_stations;

   //Allocating issue queue, ROB, reservation stations
	data_memory            = new unsigned char[data_memory_size];
   issueQ                 = new instructT[max_issue];
   robP                   = new Fifo<robT>( rob_size );

   reset();
}
	
sim_ooo::~sim_ooo(){
}

void sim_ooo::init_exec_unit(exe_unit_t exec_unit, unsigned latency, unsigned instances){
   //TODO: Check the latency and how it is implemented
   execFp[exec_unit].init(instances, latency);
}

void sim_ooo::load_program(const char *filename, unsigned base_address){
   instMemSize            = parse(string(filename), base_address);
   this->PC               = base_address;
}

instructT sim_ooo::fetchInstruction ( unsigned pc ) {
   int      index     = (pc - this->baseAddress)/4;
   ASSERT((index >= 0) && (index < instMemSize), "out of bound access of instruction memory %d", index);
   instructT instruct = *(instMemory[index]);
   if(instruct.opcode != EOP)
      instCount++;
   return instruct;
}

void sim_ooo::fetch(){
    for (int j = 0; j < issueWidth && robP->isFull(); j++){
      //fetching instruction according to PC
      instructT instruct   = fetchInstruction ( PC );
      //finding the execution unit of the opcode
      exe_unit_t unit      = opcodeToExUnit(instruct.opcode);

      if (resStation[unit].size() != resStSize[unit]) {
         //Checking if reservation station is not busy
         robT robEntry       = robT(&instruct);
         uint32_t robIndex   = robP->push(robEntry);
         resStationT* resP   = new resStationT(robEntry.dInstP);
         resP->busy          = true;
         resP->pcRs          = PC;
         resP->vj            = (instruct.src1Valid) ? regRead(instruct.src1, instruct.src1F) : UNDEFINED;
         resP->vk            = (instruct.src2Valid) ? regRead(instruct.src2, instruct.src2F) : UNDEFINED;
         resP->qj            = (instruct.src1Valid) ? regTag(instruct.src1, instruct.src1F) : UNDEFINED;
         resP->qk            = (instruct.src2Valid) ? regTag(instruct.src2, instruct.src2F) : UNDEFINED;
         resP->tagD          = robIndex;

         //setting bits for values ready/not ready
         if(instruct.src1Valid && regBusy(instruct.src1, instruct.src1F)) 
            resP->vjR  = false; 
         if(instruct.src2Valid && regBusy(instruct.src2, instruct.src2F))
            resP->vkR  = false; 

         //Updating address field of ROB according to memory unit
         if( unit == MEMORY )
            resUnit[i].addr          = instruct.imm;

         //incrementing PC only if ROB and RS are not full
         PC                          = PC + 4;

         //update TAG at register File with ROB entry if destination exists
         if(instruct.dstValid)
            if(instruct.dstF)
               set_fp_reg_tag(instruct.dst, robIndex, true); 
            else
               set_int_reg_tag(instruct.dst, robIndex, true); 
         resStation[unit].push_back( resP );
         break;
      }
      //Break if Branch to create a basic block
      //Since BP = always not taken, do nothing
   }
}

//TODO: 0. Load / Store latency modelling, check for dependent, or pending stores
//1. Maintain program order while issuing into exec. 
//2. SWS after LWS need not require LWS Stalling
//3. If SWS (not ready) before LWS (ready), LWS has to stall.
//4. WHATEVER IS IN RES STATION HAS ALREADY GONE INTO EXEC UNIT (inExec checks for that)
void sim_ooo::dispatch(){
   //To iterate through reservation station units
   for(int unit = 0; unit < RS_TOTAL; unit++) {
      //To iterate through individual units
      for(int payIndex = 0; payIndex < resStation[unit].size(); payIndex++) {
         //Checking if both operands are ready, hence instruction is ready and it is not in execute stage
         resStationT* resP = resStation[unit][payIndex];
         bool isLoad       = (resP->dInstP->opcode == LW) || (resP->dInstP->opcode == LWS);
         uint32_t addr     = agen(*(resP->dInstP));
         if (resP->busy && !resP->inExec && resP->vjR && resP->vkR && (!isLoad || (isLoad && isConflictingStore(resP->tagD, addr))) ){
            int execUnit   = opcodeToExUnit(resP->dInstP->opcode);
            for(int laneId = 0; laneId < execFp[execUnit].numLanes; laneId++){
               //checking for free execution units
               if(execFp[execUnit].lanes[laneId].ttl == 0) {
                  resP->inExec                               = true;
                  execFp[execUnit].lanes[laneId].payloadP    = resP;
                  // Adding 1 to model 1 unit latency in Write Result
                  execFp[execUnit].lanes[laneId].ttl         = execFp[execUnit].latency + 1;
                  break;
               }
            }
         }
      }
   }
}
bool sim_ooo::isConflictingStore(int loadTag, unsigned memAddress){
   bool conflict      = false;
   for(int i = 0; i < robP->getCount(); i++){
      int tag         = robP->genIndex(i);
      opcode_t opcode = robP->peekNth(i)->dInstP->opcode;

      if( (opcode == SW || opcode == SWS) ) {
         if(!robP->peekNth(i)->ready)
            bool conflict      = true;
         else if(robP->peekNth(i)->dest == memAddress)
            bool conflict      = false;
      }
      if(tag == loadTag)
         return conflict;
   }
}

//-------------------------------issue stage begin-----------------------------------------------------------//
//TODO: Check for the following
//1. Structural hazards taken care before populating ROB and RS?
//2. ROB Updated?
//3. RS Updated with all entries?
//4. Register file updated with Tag and set busy?

//TODO:
//1. Check if EXEC UNITS are FREE and if source values are available.
//2. Push into Exec Unit when both conditions are satisfied.
void sim_ooo::issue( ) {
   //Issuing N instructions and checking if ROB is full
   fetch();
   dispatch();
}

//-------------------------------issue stage end-------------------------------------------------------------//


//-------------------------------------------------------------execute stage---------------------------------//
instructT sim_ooo::execInst(int& count, uint32_t& b, uint32_t& npc){
   instructT instruct;
   count = 0;
   for(int i = 0; i < EXEC_UNIT_TOTAL; i++){
      for(int j = 0; j < execFp[i].numLanes; j++){
         if( execFp[i].lanes[j].ttl != 0 ) {
            execFp[i].lanes[j].ttl--;
            if( execFp[i].lanes[j].ttl == 0 ) {
            }
         } else{
         }
      }
   }
   ASSERT ( count <= 1, "STRUCTURAL HAZARD AT MEM DETECTED" );
   return instruct;
}

//To get the maximum ttl at a particular lane in the execution unit
int sim_ooo::getMaxTtl() {
   int ttl = 0;
   for(int i = 0; i < EXEC_UNIT_TOTAL; i++){
      for(int j = 0; j < execFp[i].numLanes; j++){
         ttl = max(ttl, execFp[i].lanes[j].ttl);
      }
   }
   return ttl;
}

void sim_ooo::execute() {

   for(int unit = 0; unit < RS_TOTAL; unit++) { 
      for(int j = 0; j < execFp[unit].numLanes; j++){
         if(execFp[unit].lanes[j].ttl == 0) {
            //-----------------------//
         }
      }
   }

   for(int j = 0; j < execFp[opcodeToExUnit(instruct.opcode)].numLanes; j++){
      if(execFp[opcodeToExUnit(instruct.opcode)].lanes[j].ttl == 0) {
         execFp[opcodeToExUnit(instruct.opcode)].lanes[j].instruct = instruct;
         if(instruct.opcode == EOP) {
            execFp[opcodeToExUnit(instruct.opcode)].lanes[j].ttl   = getMaxTtl() + 1;
         }
         else 
            execFp[opcodeToExUnit(instruct.opcode)].lanes[j].ttl   = exLatency(instruct.opcode);
         break;
      }
   }

   int count;
   uint32_t b;
   uint32_t npc;
   instruct = execInst(count, b, npc);

   if(count != 0) {
      uint32_t src1 = instruct.src1;
      uint32_t src2 = instruct.src2;
      bool src1F    = instruct.src1F;
      bool src2F    = instruct.src2F;

      this->pipeReg[MEM][B] = b;
      switch(instruct.opcode) {
         case LW ... SWS:
            pipeReg[MEM][ALU_OUTPUT] = agen (instruct);
            break;

         case ADD ... DIV:
         case ADDS ... DIVS:
            pipeReg[MEM][ALU_OUTPUT] = alu(regRead(src1, src1F), regRead(src2, src2F), src1F, src2F, instruct.opcode);
            break;

         case ADDI ... ANDI:
            pipeReg[MEM][ALU_OUTPUT] = alu(regRead(src1, src1F), instruct.imm, src1F, false, instruct.opcode);
            break;

         case BLTZ:
            pipeReg[MEM][COND]       = regRead(src1, src1F) < 0;
            pipeReg[MEM][ALU_OUTPUT] = alu(npc, instruct.imm, false, false, instruct.opcode);
            break;

         case BNEZ:
            pipeReg[MEM][ALU_OUTPUT] = alu(npc, instruct.imm, false, false, instruct.opcode);
            pipeReg[MEM][COND]       = regRead(src1, src1F) != 0;
            break;

         case BEQZ:
            pipeReg[MEM][ALU_OUTPUT] = alu(npc, instruct.imm, false, false, instruct.opcode);
            pipeReg[MEM][COND]       = regRead(src1, src1F) == 0;
            break;

         case BGTZ:
            pipeReg[MEM][ALU_OUTPUT] = alu(npc, instruct.imm, false, false, instruct.opcode);
            pipeReg[MEM][COND]       = regRead(src1, src1F) > 0;
            break;

         case BGEZ:
            pipeReg[MEM][ALU_OUTPUT] = alu(npc, instruct.imm, false, false, instruct.opcode);
            pipeReg[MEM][COND]       = regRead(src1, src1F) >= 0;
            break;

         case BLEZ:
            pipeReg[MEM][ALU_OUTPUT] = alu(npc, instruct.imm, false, false, instruct.opcode);
            pipeReg[MEM][COND]       = regRead(src1, src1F) <= 0;
            break;

         case JUMP:
            pipeReg[MEM][ALU_OUTPUT] = alu(npc, instruct.imm, false, false, instruct.opcode);
            pipeReg[MEM][COND]       = 1;
            break;

         case NOP:
         case EOP:
            break;

         default:
            ASSERT(false, "Unknown operation encountered");
            break;
      }
   }
   this->instrArray[MEM] = instruct;
   memFlag               = memLatency;
}

//---------------------------------------------------------------------------------------------------------//


void sim_ooo::run(unsigned cycles){
}

//reset the state of the sim_oooulator
void sim_ooo::reset(){
   this->data_memory       = new unsigned char[dataMemSize];
   for(unsigned i = 0; i < this->dataMemSize; i++) {
      this->data_memory[i] = UNDEFINED; 
   }

   //initializing Instruction Array to UNDEFINED
   for(int i = 0; i < NUM_STAGES; i++) {
      this->instrArray[i].stall();
   }

   //initializing GPRs to UNDEFINED
   for(int i = 0; i < NUM_GP_REGISTERS; i++) {
      this->gprFile[i].value = UNDEFINED;
      this->gprFile[i].tag   = UNDEFINED;
      this->gprFile[i].busy  = false;
   }

   //initializing FP registers to UNDEFINED
   for(int i = 0; i < NUM_FP_REGISTERS; i++) {
      this->fpFile[i].value = UNDEFINED;
      this->fpFile[i].tag   = UNDEFINED;
      this->fpFile[i].busy  = false;
   }
   //Clearing Res Station
   for(int i = 0; i < RS_TOTAL; i++){
      resStation[i].clear();
   }
   //Clearing ROB
   robP->popAll();
}

//--------------------------------------- IMPORTANT FUNCTIONS ---------------------------------------------//

bool sim_ooo::regBusy(uint32_t regNo, bool isF) {
   return isF ? fpFile[regNo].busy : gprFile[regNo].busy;
}

bool sim_ooo::intBranch(){
   for(int i = 0; i < execFp[INTEGER].numLanes; i++) {
      if( execFp[INTEGER].lanes[i].instruct.is_branch ) {
         return true;
      }
   }
   return false;
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

void sim_ooo::write_memory(unsigned address, unsigned value){
	unsigned2char(value,data_memory+address);
}

//-------------------------------------------------------------------------------------------------------------------------------------------//
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



