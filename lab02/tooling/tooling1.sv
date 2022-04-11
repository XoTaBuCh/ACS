module priorityencoder(input  logic [7:1] a,
                       output logic [2:0] y);
              
    // For Lab 2, write a structural Verilog model 
    // use and, or, not
    // do not use assign statements, always blocks, or other behavioral Verilog
  logic n7, n6, n5, n4, n3, n2;
  logic i1, i2, i3, i4, i5;

  not gn7(n7, a[7]);
  not gn6(n6, a[6]);
  not gn5(n5, a[5]);
  not gn4(n4, a[4]);
  not gn3(n3, a[3]);
  not gn2(n2, a[2]);

  and gi1(i1, a[4], n5);
  or gi2(i2, i1, a[5]);
  and gi3(i3, i2, n6);
  or gi4(i4, i3, a[6]);
  and gi5(i5, i4, n7);
  or go1(y[2], i5, a[7]);

  logic j1, j2, j3, j4, j5, j6, j7;
  and gj1(j1, n3, a[2]);
  or gj2(j2, a[3], j1);
  and gj3(j3, n4, j2);
  and gj4(j4, n5, j3);
  and gj5(j5, n6, j4);
  or gj6(j6, a[6], j5);
  and gj7(j7, n7, j6);
  or go2(y[1], a[7], j7);
 
  logic k1, k2, k3, k4, k5, k6;
  and gk1(k1, n2, a[1]);
  and gk2(k2, n3, k1);
  or gk3(k3, a[3], k2);
  and gk4(k4, n4, k3);
  and gk5(k5,n5, k4);
  or gk6(k6, a[5], k5);
  and gk7(k7, n6, k6);
  and gk8(k8,n7, k7);

  or go3(y[0], a[7], k8);

endmodule

module testbench #(parameter VECTORSIZE=10);
  logic                   clk;
  logic [7:1]             a;
  logic [2:0]             y, yexpected;
  logic [6:0]             hash;
  logic [31:0]            vectornum, errors;
  // 32-bit numbers used to keep track of how many test vectors have been
  logic [VECTORSIZE-1:0]  testvectors[1000:0];
  logic [VECTORSIZE-1:0]  DONE = 'bx;
  
  // instantiate device under test
  priorityencoder dut(a, y);
  
  // generate clock
  always begin
   clk = 1; #5; clk = 0; #5; 
  end
  
  // at start of test, load vectors and pulse reset
  initial begin
    $readmemb("priorityencoder.tv", testvectors);
    vectornum = 0; errors = 0;
    hash = 0;
  end
    
  // apply test vectors on rising edge of clk
  always @(posedge clk) begin
    #1; {a, yexpected} = testvectors[vectornum];
  end
  
  // Check results on falling edge of clock.
  always @(negedge clk)begin
      if (y !== yexpected) begin // result is bad
      $display("Error: inputs=%b", a);
      $display(" outputs = %b (%b expected)", y, yexpected);
      errors = errors+1;
    end
    vectornum = vectornum + 1;
    hash = hash ^ y;
    hash = {hash[5:0], hash[6] ^ hash[5]};
    if (testvectors[vectornum] === DONE) begin
      #2;
      $display("%d tests completed with %d errors", vectornum, errors);
      $display("Hash: %h", hash);
      $stop;
    end
  end
endmodule
