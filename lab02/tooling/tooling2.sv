module aludecoder(input  logic [1:0] ALUOp,
                  input  logic [2:0] funct3,
                  input  logic op_5, funct7_5,
                  output logic [2:0] ALUControl);
              
    // For Lab 2, write a structural Verilog model 
    // use and, or, not
    // do not use assign statements, always blocks, or other behavioral Verilog
    // Example syntax to access bits to make ALUControl[0] = ~funct3[0]
    //  not g1(ALUControl[0], funct3[0]);
    // This is just an example; replace this with correct logic!
    logic n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16, n17, n18;
    logic nx1, nx2, nx3;

    // op10 = ALUOp[1] * ~ALUOp[0];
    not g1(n1, ALUOp[0]);
    and g2(n2, ALUOp[1], n1);

    // ALUControl[0] = ~ALUOp[1] * ALUOp[0] + n2 * ~(~funct3[2] * ~funct3[1] * ~funct3[0] * ~(op_5, funct7_5) + funct3[2] * funct3[1] * funct3[0]);
    not g3(n3, ALUOp[1]);
    and g4(n4, n3, ALUOp[0]);

    not g5(n5, funct3[2]);
    not g6(n6, funct3[1]);
    not g7(n7, funct3[0]);
    and g8(n8, n5, n6);
    and g9(n9, n8, n7);

    and gx1(nx1, op_5, funct7_5);
    not gx2(nx2, nx1);
    and gx3(nx3, n9, nx2);

    and g10(n10, funct3[2], funct3[1]);
    and g11(n11, n10, funct3[0]);
    
    or g12(n12, nx3, n11);
    not g13(n13,n12);

    and g14(n14, n2, n13);
    or go1(ALUControl[0], n4, n14); 


    //ALUControl[1] = n2 * funct3[2] * funct3[1];
    and g16(n16, n2, funct3[2]);
    and go2(ALUControl[1], n16, funct3[1]);


    //ALUControl[2] = n2 * ~funct3[2] * funct3[1] * ~funct3[0]
    and g17(n17, n2, n5);
    and g18(n18, n17, funct3[1]);
    and go3(ALUControl[2], n18, n7);

endmodule

module testbench #(parameter VECTORSIZE=10);
  logic                   clk;
  logic                   op_5, funct7_5;
  logic [1:0]             ALUOp;
  logic [2:0]             funct3;
  logic [2:0]             ALUControl, ALUControlExpected;
  logic [6:0]             hash;
  logic [31:0]            vectornum, errors;
  // 32-bit numbers used to keep track of how many test vectors have been
  logic [VECTORSIZE-1:0]  testvectors[1000:0];
  logic [VECTORSIZE-1:0]  DONE = 'bx;
  
  // instantiate device under test
  aludecoder dut(ALUOp, funct3, op_5, funct7_5, ALUControl);
  
  // generate clock
  always begin
   clk = 1; #5; clk = 0; #5; 
  end
  
  // at start of test, load vectors and pulse reset
  initial begin
    $readmemb("aludecoder.tv", testvectors); // Students may have to add a file path if ModelSim set up incorrectly
    vectornum = 0; errors = 0;
    hash = 0;
  end
    
  // apply test vectors on rising edge of clk
  always @(posedge clk) begin
    #1; {ALUOp, funct3, op_5, funct7_5, ALUControlExpected} = testvectors[vectornum];
  end
  
  // Check results on falling edge of clock.
  always @(negedge clk)begin
      if (ALUControl !== ALUControlExpected) begin // result is bad
      $display("Error: inputs=%b %b %b %b", ALUOp, funct3, op_5, funct7_5);
      $display(" outputs = %b (%b expected)", ALUControl, ALUControlExpected);
      errors = errors+1;
    end
    vectornum = vectornum + 1;
    hash = hash ^ {ALUControl};
    hash = {hash[5:0], hash[6] ^ hash[5]};
    if (testvectors[vectornum] === DONE) begin
      #2;
      $display("%d tests completed with %d errors", vectornum, errors);
      $display("Hash: %h", hash);
      $stop;
    end
  end
endmodule
