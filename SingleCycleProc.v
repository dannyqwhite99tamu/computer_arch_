
`include "registerfile.v"
`include "SingleCycleControl.v"
`include "alu.v"
`include "DataMemory.v"
`include "nextpc.v"
`include "InstructionMemory.v"
`include "ImmGenerator.v"

`timescale 1ns / 1ps

module singlecycle(
    input resetl,
    input [63:0] startpc,
    output reg [63:0] currentpc,
    output [63:0] dmemout,
    input CLK
);

    // Next PC connections
    wire [63:0] nextpc;       // The next PC, to be updated on clock cycle

    // Instruction Memory connections
    wire [31:0] instruction;  // The current instruction

    // Parts of instruction
    wire [4:0] rd;            // The destination register
    wire [4:0] rm;            // Operand 1
    wire [4:0] rn;            // Operand 2
   // wire [5:0] shamt;
    wire [10:0] opcode;

   

    // Control wires
    wire reg2loc;
    wire alusrc;
    wire mem2reg;
    wire regwrite;
    wire memread;
    wire memwrite;
    wire branch;
    wire uncond_branch;
    wire [3:0] aluctrl;
    wire [1:0] signop;

    // Register file connections
    wire [63:0] regoutA;     // Output A
    wire [63:0] regoutB;     // Output B
    wire [63:0] reginW;	     // Input W

    // ALU connections
    wire [63:0] aluout;
    wire [63:0] aluin;
    wire zero;

	//sign extend	
    wire [63:0] extimm; 
    wire [25:0] imm;


    NextPCLogic nextPC(.NextPC(nextpc),
		        .CurrentPC(currentpc),
			.SignExtImm64(extimm),
			.Branch(branch),
			.ALUZero(zero),
			.Uncondbranch(uncond_branch));
     InstructionMemory imem(
        .Data(instruction),
        .Address(currentpc)
    );

    // PC update logic
    always @(negedge CLK)
    begin
        if (~resetl)
            currentpc <= startpc;
        else
            currentpc <= nextpc;
    end

    // Parts of instruction
    assign rd = instruction[4:0];
    assign rn = instruction[9:5];
    assign rm = reg2loc ? instruction[4:0] : instruction[20:16];
    assign opcode = instruction[31:21];
    assign imm = instruction[25:0];

   

    SingleCycleControl control(
        .reg2loc(reg2loc),
        .alusrc(alusrc),
        .mem2reg(mem2reg),
        .regwrite(regwrite),
        .memread(memread),
        .memwrite(memwrite),
        .branch(branch),
        .uncond_branch(uncond_branch),
        .aluop(aluctrl),
        .signop(signop),
        .opcode(opcode)
    );

    /*
    * Connect the remaining datapath elements below.
    * Do not forget any additional multiplexers that may be required.
    */

	
    //reg_file
    RegisterFile registers(.BusA(regoutA), 
	.BusB(regoutB),
	.BusW(reginW),
	.RA(rn),
	.RB(rm),
	.RW(rd),
	.RegWr(regwrite),
	.Clk(CLK));
   	
	
    // IMMEDIATE GENERATOR
    ImmGenerator ImmGen(.Imm64(extimm), .Imm26(imm), .Ctrl(signop));
	
	
    // ALU_src mux
    assign aluin = alusrc ? extimm : regoutB; 
    
    // ALU
    alu ALU(.busW(aluout), 
	    .zero(zero),
	    .busA(regoutA),
	    .busB(aluin),
	    .ctrl(aluctrl));

    //data memory
    DataMemory data(.ReadData(dmemout),
	  	    .Address(aluout),
		    .WriteData(regoutB),
		    .MemoryRead(memread),
		    .MemoryWrite(memwrite),
		    .Clock(CLK));

    // Mem_To_Reg mux
    assign reginW = mem2reg ? dmemout : aluout;



endmodule
