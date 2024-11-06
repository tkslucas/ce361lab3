/*
    CompEng 361 - Northwestern Unviersity

    Mini LC3 Single Cycle Datapath and Control

    Described in Lecture 06 (10/14/2024)

    Supports:
     - Operate (ADD,AND,NOT)
     - Memory Transfer (LD/ST)
     - Control (JMP)

*/

`define WORD_WIDTH  16
`define REG_COUNT 8
`define OP_SIZE 2
`define RF_ADDR_WIDTH 3
`define MEM_SIZE 65536
`define FN_SIZE 2



`define ADD_OPCODE 4'b0001
`define AND_OPCODE 4'b0101
`define LD_OPCODE 4'b0010
`define ST_OPCODE 4'b0011
`define NOT_OPCODE 4'b1001
`define JMP_OPCODE 4'b1100


module LC3DataPath(
        output [15:0] instAddr, 
        input [15:0]  instData,
        output [15:0] dataAddr,
        output [15:0] dataWrite,
        input [15:0] dataRead,
        output   dataWrEn,
        input clk, input rst);

    parameter StartAddr = 16'h3000;

   
    wire [`WORD_WIDTH-1:0] SR1_out, SR2_out, DR_in;
    wire [`RF_ADDR_WIDTH-1:0] SR1, SR2, DR;
    wire [`WORD_WIDTH-1:0] IR;
    wire [`WORD_WIDTH-1:0] alu_out, alu_opa, alu_opb;
    wire [`WORD_WIDTH-1:0] PC, NPC, PC_plus_one;
    wire [`WORD_WIDTH-1:0] immediate, offset;
    wire [1:0] alu_fn;

    wire [3:0] opcode;

    assign invalid_op = (IR != `ADD_OPCODE) &&
            (IR != `AND_OPCODE) && (IR != `NOT_OPCODE) &&
            (IR != `LD_OPCODE) && (IR != `ST_OPCODE) &&
            (IR != `JMP_OPCODE);

    assign instAddr = PC;
    assign IR = instData; // Not actually a register

    // Basic decode fields
    assign opcode = IR[15:12];
    assign SR1 = IR[8:6];
    assign SR2 = IR[2:0];
    assign DR = IR[11:9];

    assign dataAddr = alu_out;
    assign dataWrite = SR1_out;
    assign dataWrEn = (opcode == `ST_OPCODE); // only stores will write to MEM

    // op type
    assign mem_op = (opcode == `LD_OPCODE) || (opcode == `ST_OPCODE);
    assign alu_op = (opcode == `ADD_OPCODE) || (opcode == `AND_OPCODE) ||
                    (opcode == `NOT_OPCODE);

    // ALU input muxes
    assign alu_opa =  (mem_op) ? PC_plus_one : SR1_out;
    assign alu_opb = (mem_op) ? offset : 
            ((IR[5]) ? immediate : SR2_out);

     assign alu_fn = (opcode == `NOT_OPCODE) ? 2'b11 : (opcode == `AND_OPCODE) ? 2'b10 :
        2'b00;

    // sign extend immediate and offset
    assign immediate = {{12{IR[4]}}, IR[3:0]};
    assign offset = {{8{IR[8]}}, IR[7:0]};

    // Next PC Logic
    assign PC_plus_one = PC + 1;
    assign NPC = (opcode == `JMP_OPCODE) ? SR1_out : PC_plus_one;
    
    // RF Writeback
    assign DR_we = ((opcode != `JMP_OPCODE) && (opcode != `ST_OPCODE));
    assign DR_in = (opcode == `LD_OPCODE) ? dataRead : alu_out;

    RegFile RF( .SrcAddr1(SR1), .SrcData1(SR1_out), .SrcAddr2(SR2), .SrcData2(SR2_out), 
                .DstAddr(DR), .DstData(DR_in),
                .WriteEn(DR_we), .clk(clk));

    ALU LC3ALU( .result(alu_out),
                .opA(alu_opa),
                .opB(alu_opb),
                .fn(alu_fn));

    // The PC
    Register #(16,StartAddr)  PC_REG  ( .q(PC),     
                  .d(NPC), 
                 .we(1'b1),
                 .clk(clk),
                 .rst(rst));

endmodule


module LC3Memory(
    input [`WORD_WIDTH-1:0] InstAddr, 
    output [`WORD_WIDTH-1:0] InstData, 
    input [`WORD_WIDTH-1:0] DataAddr, 
    input [`WORD_WIDTH-1:0] DataWrite, 
    output [`WORD_WIDTH-1:0] DataRead,
    input WriteEn, 
    input clk);

    reg [`WORD_WIDTH-1:0] mem[0:`MEM_SIZE-1];

    assign InstData = mem[InstAddr];
    assign DataRead = mem[DataAddr];

    always @ (posedge clk) begin
        if (WriteEn)
            mem[DataAddr] <= DataWrite;
    end


endmodule

module RegFile(
    input [`RF_ADDR_WIDTH-1:0] SrcAddr1, 
    output [`WORD_WIDTH-1:0] SrcData1, 
    input [`RF_ADDR_WIDTH-1:0] SrcAddr2, 
    output [`WORD_WIDTH-1:0] SrcData2, 
    input [`RF_ADDR_WIDTH-1:0] DstAddr,
    input [`WORD_WIDTH-1:0] DstData,
    input WriteEn, 
    input clk);

    reg [`WORD_WIDTH-1:0] mem[0:`REG_COUNT-1];

    assign SrcData1 = mem[SrcAddr1];
    assign SrcData2 = mem[SrcAddr2];

    always @ (posedge clk)
        if (WriteEn)
            mem[DstAddr] <= DstData;
endmodule

module ALU(
    output [`WORD_WIDTH-1:0] result,
    input [`WORD_WIDTH-1:0] opA,
    input [`WORD_WIDTH-1:0] opB,
    input [`FN_SIZE-1:0] fn);

    wire [`WORD_WIDTH-1:0] addResult, andResult, notResult;

    assign addResult = opA + opB;
    assign andResult = opA & opB;
    assign notResult = ~opA;

    assign result = (fn == 2'b11) ? notResult : 
        (fn[1] == 1'b0) ? addResult : andResult;

endmodule

module Register(
    output [width-1:0] q, 
    input [width-1:0] d,
    input we,
    input clk,
    input rst);
    parameter width = 16;
    parameter rst_value = 0;

    reg [width-1:0] q;

    always @ (posedge clk or negedge rst)
        if (~rst)
            q <= rst_value;
        else 
            q <= (we) ? d : q;

endmodule