// Template for Northwestern - CompEng 361 - Lab3
// Groupname: RISCtakers
// NetIDs: rap4819, 

// ------------------------------- OPCODES ----------------------------------
`define WORD_WIDTH 32
`define NUM_REGS 32

`define OPCODE_COMPUTE     7'b0110011
`define OPCODE_BRANCH      7'b1100011
`define OPCODE_LOAD        7'b0000011
`define OPCODE_STORE       7'b0100011 
`define OPCODE_COMPUTE_IMM 7'b0010011
`define OPCODE_MULDIV      7'b0110011
`define OPCODE_JAL         7'b1101111
`define OPCODE_JALR        7'b1100111
`define OPCODE_LUI         7'b0110111
`define OPCODE_AUIPC       7'b0010111

// ------------------------------- FUNC3 --------------------------------------
`define FUNC_ADD      3'b000  // same for subtraction
`define FUNC_XOR      3'b100
`define FUNC_OR       3'b110
`define FUNC_AND      3'b111
`define FUNC_SLL      3'b001
`define FUNC_SRL      3'b101
`define FUNC_SRA      3'b101
`define FUNC_SLT      3'b010
`define FUNC_SLTU     3'b011

// for clarity: redefined for loads
`define FUNC_LB      3'b000
`define FUNC_LH      3'b001
`define FUNC_LW      3'b010
`define FUNC_LBU     3'b100
`define FUNC_LHU     3'b101

// ----------------------------- AUX_FUNC --------------------------------------
`define AUX_FUNC_ADD  7'b0000000 // same for xor,or,and,sll,srl,slt,sltu
`define AUX_FUNC_SUB  7'b0100000 // same for sra

`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10

module SingleCycleCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [`WORD_WIDTH-1:0] PC, InstWord;
   wire [`WORD_WIDTH-1:0] DataAddr, StoreData, DataWord;
   wire [1:0]  MemSize;
   wire        MemWrEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst;
   wire [`WORD_WIDTH-1:0] Rdata1, Rdata2, RWrdata, EU_out;
   wire [`WORD_WIDTH-1 :0] immediate;
   wire        RWrEn;
   wire        ALUSrc, EACalc;
   wire        Inv_R_type, Inv_I_type, Inv_Loads;

   wire [`WORD_WIDTH-1:0] NPC, PC_Plus_4;
   wire [6:0]  opcode;

   wire [6:0]  funct7;
   wire [2:0]  funct3;
   
   wire invalid_op;
   
   // Only supports R-TYPE and I-TYPE
   assign halt = invalid_op;

   assign Inv_R_type = ((opcode == `OPCODE_COMPUTE) && 
                       (((funct7 == `AUX_FUNC_SUB) && ((funct3 == `FUNC_ADD) || (funct3 == `FUNC_SRA)))||
                       ((funct7 == `AUX_FUNC_ADD) && ((funct3 == `FUNC_ADD)||(funct3 == `FUNC_XOR) || (funct3 == `FUNC_OR) || 
                       (funct3 == `FUNC_AND) || (funct3 == `FUNC_SLL) || (funct3 == `FUNC_SRL) || 
                       (funct3 == `FUNC_SLT) || (funct3 == `FUNC_SLTU)))
                        ));

   assign Inv_I_type = ((opcode == `OPCODE_COMPUTE_IMM) && ((funct3 == `FUNC_ADD || funct3 == `FUNC_XOR 
                        || funct3 == `FUNC_OR || funct3 == `FUNC_AND || funct3 == `FUNC_SLL || funct3 == `FUNC_SRL ||
                        funct3 == `FUNC_SRA || funct3 == `FUNC_SLT || funct3 == `FUNC_SLTU)));

   assign Inv_Loads = ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LB || funct3 == `FUNC_LH || funct3 == `FUNC_LW
                       || funct3 == `FUNC_LBU || funct3 == `FUNC_LHU));
         
   assign invalid_op = ! (Inv_R_type || Inv_I_type || Inv_Loads); 
     
   // System State 
   Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
            .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
	      .AddrB(Rsrc2), .DataOutB(Rdata2), 
	      .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   Reg PC_REG(.Din(NPC), .Qout(PC), .WE(1'b1), .CLK(clk), .RST(rst));

   // Instruction Decode
   assign opcode = InstWord[6:0];   
   assign Rdst = InstWord[11:7]; 
   assign Rsrc1 = InstWord[19:15]; 
   assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
 
   assign immediate = {{20{InstWord[31]}}, InstWord[31:20]};
   assign Rsrc2 = InstWord[24:20];
   assign funct7 = InstWord[31:25];  // R-Type

   // control signal for R-type vs I-type instruction (0 for R, 1 for immediate)
   assign ALUSrc = (opcode == `OPCODE_COMPUTE_IMM) ? 1'b1 : 1'b0;
   assign EACalc_control = (opcode == `OPCODE_LOAD) ? 1'b1: 1'b0;

   assign MemWrEn = (opcode == `OPCODE_STORE);

   assign DataAddr = (opcode == `OPCODE_LOAD) ? EU_out: 32'hXXXXXXXX;

   assign MemSize = ((opcode == `OPCODE_LOAD) && ((funct3 == `FUNC_LB) || (funct3 == `FUNC_LBU))) ? SIZE_BYTE :
                     ((opcode == `OPCODE_LOAD) && ((funct3 == `FUNC_LH) || (funct3 == `FUNC_LHU))) ? SIZE_HWORD :
                     ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LW)) ? SIZE_WORD : 2'bXX; 
                     

   assign RWrEn = (opcode == `OPCODE_COMPUTE || opcode == `OPCODE_COMPUTE_IMM || opcode == `OPCODE_LOAD);  
   assign RWrdata = (opcode == `OPCODE_LOAD) ? DataWord : 
                    (opcode == `OPCODE_COMPUTE || opcode == `OPCODE_COMPUTE_IMM) ? EU_out : 32'hXXXXXXXX;

   // Supports R-Type and I-type instructions -- please add muxes and other control signals
   
   ExecutionUnit EU(.out(EU_out),
                    .opA(Rdata1), 
                    .opB(Rdata2), 
                    .func(funct3), 
                    .auxFunc(funct7), 
                    .imm(immediate), 
                    .aluSrc(ALUSrc),
                    .EACalc(EACalc_control));

   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
   assign NPC = PC_Plus_4;
   
endmodule // SingleCycleCPU

// responsible for immediate calculations
module ALU_imm(out1, rs1, imm1, funca, eaCalc);
   output [`WORD_WIDTH-1:0] out1;
   input [`WORD_WIDTH-1:0]  rs1, imm1;
   input [2:0] 	 funca;
   input eaCalc;

   wire [`WORD_WIDTH-1:0] add = rs1 + imm1; 
   wire [`WORD_WIDTH-1:0] subtract = rs1 - imm1;
   wire [`WORD_WIDTH-1:0] ored = rs1 | imm1;
   wire [`WORD_WIDTH-1:0] anded = rs1 & imm1;
   wire [`WORD_WIDTH-1:0] xored = rs1 ^ imm1;
   wire [`WORD_WIDTH-1:0] signed_lt = ($signed(rs1) < $signed(imm1)) ? 32'b1 : 32'b0; // imm
   wire [`WORD_WIDTH-1:0] unsigned_lt = (rs1 < imm1) ? 32'b1 : 32'b0;
   wire [`WORD_WIDTH-1:0] sll = rs1 << imm1[4:0];
   wire [`WORD_WIDTH-1:0] srl = rs1 >> imm1[4:0];
   wire [`WORD_WIDTH-1:0] sra =($signed(rs1)) >>> imm1[4:0];
   
   assign out1 =(eaCalc)? add:
                (funca == 3'b000) ? add :
                (funca == 3'b100) ? xored:
                (funca == 3'b110) ? ored :
                (funca == 3'b111) ? anded:
                (funca == 3'b010) ? signed_lt :
                (funca == 3'b011) ? unsigned_lt :
                (funca == 3'b001 && imm1[11:5] == 7'b0000000) ? sll :
                (funca == 3'b101 && imm1[11:5] == 7'b0000000) ? srl :
                (funca == 3'b101 && imm1[11:5] == 7'b0100000) ? sra : 32'hXXXXXXXX;
endmodule

module ALU_reg(out2, inA, inB, funcb, auxFuncb);
   output [`WORD_WIDTH-1:0] out2;
   input [`WORD_WIDTH-1:0]  inA, inB;
   input [2:0] 	 funcb;
   input [6:0] 	 auxFuncb;

   wire [`WORD_WIDTH-1:0] add = inA + inB; 
   wire [`WORD_WIDTH-1:0] subtract = inA - inB;
   wire [`WORD_WIDTH-1:0] ored = inA | inB;
   wire [`WORD_WIDTH-1:0] anded = inA & inB;
   wire [`WORD_WIDTH-1:0] xored = inA ^ inB;
   wire [`WORD_WIDTH-1:0] signed_lt = ($signed(inA) < $signed(inB)) ? 32'b1 : 32'b0; // imm
   wire [`WORD_WIDTH-1:0] unsigned_lt = (inA < inB) ? 32'b1 : 32'b0;
   wire [`WORD_WIDTH-1:0] sll = inA << inB[4:0];
   wire [`WORD_WIDTH-1:0] srl = inA >> inB[4:0];
   wire [`WORD_WIDTH-1:0] sra =($signed(inA)) >>> inB[4:0];
   
   assign out2 = (funcb == 3'b000 && auxFuncb == 7'b0000000) ? add :
                (funcb == 3'b000 && auxFuncb == 7'b0100000) ? subtract :
                (funcb == 3'b001 && auxFuncb == 7'b0000000) ? sll :
                (funcb == 3'b010 && auxFuncb == 7'b0000000) ? signed_lt :
                (funcb == 3'b011 && auxFuncb == 7'b0000000) ? unsigned_lt :
                (funcb == 3'b100 && auxFuncb == 7'b0000000) ? xored:
                (funcb == 3'b101 && auxFuncb == 7'b0000000) ? srl :
                (funcb == 3'b101 && auxFuncb == 7'b0100000) ? sra :
                (funcb == 3'b110 && auxFuncb == 7'b0000000) ? ored :
                (funcb == 3'b111 && auxFuncb == 7'b0000000) ? anded : 32'hXXXXXXXX;
endmodule

module ALU_mux(out3, aluSrc1, eaCalc, immresult, regresult);
   output [`WORD_WIDTH-1 :0] out3;
   input  aluSrc1, eaCalc;
   input [`WORD_WIDTH-1:0]  immresult, regresult;

   assign out3 = (eaCalc || aluSrc1) ? immresult: regresult;

endmodule

// You will need to extend it. Feel free to modify the interface also
module ExecutionUnit(out, opA, opB, func, auxFunc, imm, aluSrc, EACalc);
   output [`WORD_WIDTH-1:0] out;
   input [`WORD_WIDTH-1:0]  opA, opB, imm;
   input [2:0] 	 func;
   input [6:0] 	 auxFunc;
   input aluSrc, EACalc;
   
   wire [`WORD_WIDTH-1:0] imm_result;
   wire [`WORD_WIDTH-1:0] reg_result;


   ALU_imm imm_ops(.out1(imm_result), 
                   .rs1(opA), 
                   .imm1(imm), 
                   .funca(func),
                   .eaCalc(EACalc));
   
   ALU_reg reg_ops(.out2(reg_result),
                   .inA(opA),
                  .inB(opB), 
                  .funcb(func), 
                  .auxFuncb(auxFunc));
   
   ALU_mux mux (.out3(out),
               .aluSrc1(aluSrc),
               .eaCalc(EACalc),
               .immresult(imm_result),
               .regresult(reg_result));

endmodule // ExecutionUnit