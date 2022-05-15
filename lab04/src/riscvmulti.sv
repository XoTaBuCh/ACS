typedef enum logic[6:0] {r_type_op=7'b0110011, i_type_alu_op=7'b0010011, lw_op=7'b0000011, sw_op=7'b0100011, beq_op=7'b1100011, jal_op=7'b1101111} opcodetype;

module controller(input  logic       clk,
                  input  logic       reset,  
                  input  opcodetype  op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       Zero,
                  output logic [1:0] ImmSrc,
                  output logic [1:0] ALUSrcA, ALUSrcB,
                  output logic [1:0] ResultSrc, 
                  output logic       AdrSrc,
                  output logic [2:0] ALUControl,
                  output logic       IRWrite, PCWrite, 
                  output logic       RegWrite, MemWrite);

logic [1:0] ALUOp;
logic Branch, PCUpdate, t1;

aludec alu_decoder(op[5], funct3, funct7b5, ALUOp, ALUControl);

immsrc instr_decorder(op, ImmSrc);

mainfsm main_fsm(clk, reset, op, Branch, PCUpdate, IRWrite, RegWrite, MemWrite, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, ALUOp);
and g1(t1, Zero, Branch);
or g2(PCWrite, t1, PCUpdate);						
						
endmodule

module mainfsm(input  logic       clk,
               input  logic       reset,
               input  opcodetype  op,
					output logic Branch,
					output logic PCUpdate,
               output logic       IRWrite, 
               output logic       RegWrite, MemWrite,
               output logic [1:0] ALUSrcA, ALUSrcB,
					output logic [1:0] ResultSrc, 
               output logic       AdrSrc,
					output logic [1:0] ALUOp);

typedef enum logic[3:0] {s0=4'b0000, s1=4'b0001, s2=4'b0010, s3=4'b0011, s4=4'b0100, s5=4'b0101, s6=4'b0110, s7=4'b0111, s8=4'b1000, s9=4'b1001,  s10=4'b1010} statetype;

statetype state, nextstate;

  always @(posedge clk, posedge reset)
    begin
    	if (reset) 	state <= s0;
        else		state <= nextstate;
    end

  always_comb
    case (state)
    s0: nextstate = s1;
 
    s1: if (op == lw_op || op == sw_op) nextstate = s2;
        else if (op == r_type_op) nextstate = s6;
        else if (op == i_type_alu_op) nextstate = s8;
        else if (op == jal_op) nextstate = s9;
        else if (op == beq_op) nextstate = s10;
	else  nextstate = s1;
 
    s2: if (op == lw_op) nextstate = s3;
        else if (op == sw_op) nextstate = s5;
        else nextstate = s2;

    s3: nextstate = s4;
 
    s4: nextstate = s0;
 
    s5: nextstate = s0;
 
    s6: nextstate = s7;
 
    s7: nextstate = s0;
 
    s8: nextstate = s7;
 
    s9: nextstate = s7;
 
    s10: nextstate = s0;
 
    default: nextstate = s0;
 
    endcase

//output logic
  assign Branch = (state == s10);
  assign AdrSrc = (state == s3 || state == s5);
  assign IRWrite = (state == s0);
  assign PCUpdate = (state == s0 || state == s9);
  assign RegWrite = (state == s4 || state == s7);
  assign MemWrite = (state == s5);
  assign ALUSrcA[1] = (state == s2 || state == s6 || state == s8 || state == s10);
  assign ALUSrcA[0] = (state == s1 || state == s9);
  assign ALUSrcB[1] = (state == s0 || state == s9);
  assign ALUSrcB[0] = (state == s1 || state == s2  || state == s8);
  assign ALUOp[1] = (state == s6  || state == s8);
  assign ALUOp[0] = (state == s10);
  assign ResultSrc[1] = (state == s0);
  assign ResultSrc[0] = (state == s4);

endmodule


module immsrc(input opcodetype  op,
	      output logic [1:0] ImmSrc);
always_comb
    case (op)
    sw_op: ImmSrc = 2'b01;
    beq_op: ImmSrc = 2'b10;
    jal_op: ImmSrc = 2'b11;
    default: ImmSrc = 2'b00;
    endcase
	

endmodule 

module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] ALUOp,
              output logic [2:0] ALUControl);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction


  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 3'b000; // addition
      2'b01:                ALUControl = 3'b001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControl = 3'b001; // sub
                          else          
                            ALUControl = 3'b000; // add, addi
                 3'b010:    ALUControl = 3'b101; // slt, slti
                 3'b110:    ALUControl = 3'b011; // or, ori
                 3'b111:    ALUControl = 3'b010; // and, andi
                 default:   ALUControl = 3'bxxx; // ???
               endcase
    endcase
endmodule

module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);

  logic [31:0] ReadData;
  
  // instantiate processor and memories
  riscvmulti rvmulti(clk, reset, MemWrite, DataAdr, 
                       WriteData, ReadData);
  mem mem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

module mem(input  logic        clk, WE,
           input  logic [31:0] A, WD,
           output logic [31:0] ReadData);

  logic [31:0] RAM[63:0];
  
  initial
      $readmemh("riscvtest.txt",RAM);

  assign ReadData = RAM[A[31:2]];// word aligned

  always_ff @(posedge clk)
    if (WE) RAM[A[31:2]] <= WD;
endmodule

module riscvmulti(input  logic        clk, reset,
                  output logic        MemWrite,
                  output logic [31:0] Adr, WD,
                  input  logic [31:0] ReadData);

  logic       AdrSrc, Zero;
  logic       IRWrite, PCWrite, RegWrite;
  logic [1:0] ResultSrc, ImmSrc, ALUSrcA, ALUSrcB;
  logic [2:0] ALUControl;
  logic [31:0] Instr;

  controller c(clk, reset, 
					opcodetype'(Instr[6:0]),
					Instr[14:12],
					Instr[30], 
					Zero, 
					ImmSrc,
					ALUSrcA, ALUSrcB,
					ResultSrc, 
					AdrSrc,
					ALUControl,
					IRWrite, PCWrite,
					RegWrite, MemWrite);
					
	datapath dp(clk, reset,
	            PCWrite, AdrSrc, IRWrite,
					RegWrite,
					ResultSrc, ALUSrcA, ALUSrcB, ImmSrc,
					ALUControl,
					ReadData,
					Adr, WD, Instr,
					Zero);
endmodule


module datapath(input  logic        clk, reset,
                input  logic        PCWrite, AdrSrc, IRWrite,
					 input  logic        RegWrite,
                input  logic [1:0]  ResultSrc, ALUSrcA, ALUSrcB, ImmSrc,
					 input  logic [2:0]  ALUControl,
					 input  logic [31:0] ReadData,
					 output logic [31:0] Adr, WD, Instr,
					 output logic Zero);

  logic [31:0] Data, ImmExt, RD1, RD2, A;
  logic [31:0] PC, OldPC, SrcA, SrcB;
  logic [31:0] ALUResult, ALUOut, Result;
  
  // PC logic
  floprx #(32) pc1(clk, reset, PCWrite, Result, PC); // flopr result to PC if PCWrite
  mux2 #(32)   pc2(PC, Result, AdrSrc, Adr); // PC or Result to Adr if AdrSrc
  floprx #(32) pc3(clk, reset, IRWrite, PC, OldPC); // flopr PC to OldPC if IRWrite

  // register file logic
  regfile     rf(clk, RegWrite, Instr[19:15], Instr[24:20], Instr[11:7], Result, RD1, RD2);
  extend      ext(Instr[31:7], ImmSrc, ImmExt);
  flopr #(32) rf1(clk, reset, RD1, A); // flopr RD1 to A
  flopr #(32) rf2(clk, reset, RD2, WD); // flopr RD2 to WD
 
  // ALU logic
  mux3 #(32)  alu1(PC, OldPC, A, ALUSrcA, SrcA); // PC or OldPC or A to SrcA if ALUSrcA
  mux3 #(32)  alu2(WD, ImmExt, 32'd4, ALUSrcB, SrcB); // WD or ImmExt or 4 to SrcB if ALUSrcB
  alu         alu(SrcA, SrcB, ALUControl, ALUResult, Zero);
  flopr #(32) alu3(clk, reset, ALUResult, ALUOut); // flopr ALUResult to ALUOut
  
  // read data logic
  flopr #(32)  rd1(clk, reset, ReadData, Data); // flopr ReadData to Data
  floprx #(32) rd2(clk, reset, IRWrite, ReadData, Instr); // flopr ReadData to Instr if IRWrite
  
  // result logic
  mux3 #(32)  res1(ALUOut, Data, ALUResult, ResultSrc, Result); // ALUOut or Data or ALUResult to Result if ResultSrc
  
  
endmodule

module regfile(input  logic        clk, 
               input  logic        WE3, 
               input  logic [ 4:0] A1, A2, A3, 
               input  logic [31:0] WD3, 
               output logic [31:0] RD1, RD2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (WE3) rf[A3] <= WD3;	

  assign RD1 = (A1 != 0) ? rf[A1] : 0;
  assign RD2 = (A2 != 0) ? rf[A2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
               // I-type 
      2'b00:   immext = {{20{instr[31]}}, instr[31:20]};  
               // S-type (stores)
      2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
               // B-type (branches)
      2'b10:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
               // J-type (jal)
      2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 
      default: immext = 32'bx; // undefined
    endcase             
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic	clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module floprx #(parameter WIDTH = 8)
                         (input  logic   clk, reset, x,
								  input  logic [WIDTH-1:0] d,
								  output logic [WIDTH-1:0] q);
								  
  always_ff @(posedge clk, posedge reset)
    if      (reset)     q <= 0;
    else if (x) q <= d;
	 
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];

  always_comb
    case (alucontrol)
      3'b000:  result = sum;         // add
      3'b001:  result = sum;         // subtract
      3'b010:  result = a & b;       // and
      3'b011:  result = a | b;       // or
      3'b100:  result = a ^ b;       // xor
      3'b101:  result = sum[31] ^ v; // slt
      3'b110:  result = a << b[4:0]; // sll
      3'b111:  result = a >> b[4:0]; // srl
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule


module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;
  logic [31:0] hash;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite);
  
  // initialize test
  initial
    begin
      hash <= 0;
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
        if(DataAdr === 100 & WriteData === 25) begin
          $display("Simulation succeeded");
 	   	  $display("hash = %h", hash);
          $stop;
        end else if (DataAdr !== 96) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end

  // Make 32-bit hash of instruction, PC, ALU
  always @(negedge clk)
    if (~reset) begin
      hash = hash ^ dut.rvmulti.dp.Instr ^ dut.rvmulti.dp.PC;
      if (MemWrite) hash = hash ^ WriteData;
      hash = {hash[30:0], hash[9] ^ hash[29] ^ hash[30] ^ hash[31]};
    end

endmodule

