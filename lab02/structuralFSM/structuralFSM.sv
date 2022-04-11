module lightfsm(input  logic clk,
                input  logic reset,
                input  logic left, right,
                output logic la, lb, lc, ra, rb, rc);

logic d0, d1, d2, d3, d4, d5;
logic s0, s1, s2, s3, s4, s5;
logic ns0, ns2, ns3, ns5, t1, t2, t3;

flopr r0(clk, reset, d0, s0);
flopr r1(clk, reset, d1, s1);
flopr r2(clk, reset, d2, s2);
flopr r3(clk, reset, d3, s3);
flopr r4(clk, reset, d4, s4);
flopr r5(clk, reset, d5, s5);

not n1(ns0, s0);
not n2(ns2, s2);
not n3(ns3, s3);
not n5(ns5, s5);

and a1(d5, ns5, s4);
and a2(d4, ns5, s3);
and a31(t1, ns3, ns2);
and a32(t2, t1, left);
or o1(d3, d4, t2);

and a4(d0, s1, ns0);
and a5(d1, s2, ns0);
and a33(t3, t1, right);
or o2(d2, d1, t3);

assign la = s3;
assign lb = s4;
assign lc = s5;

assign ra = s2;
assign rb = s1;
assign rc = s0;


endmodule


// asynchronously resettable flip-flop
module flopr(input  logic clk, reset, d,
            output logic q);
            
  always_ff @(posedge clk or posedge reset)
    if (reset) q <= 0; // resets state to 0 on reset
    else       q <= d;
endmodule

module testbench(); 
  logic        clk, reset;
  logic        left, right, la, lb, lc, ra, rb, rc;
  logic [5:0]  expected;
  logic [6:0]  hash;
  logic [31:0] vectornum, errors;
  logic [7:0]  testvectors[10000:0];

  // instantiate device under test 
  lightfsm dut(clk, reset, left, right, la, lb, lc, ra, rb, rc); 

  // generate clock 
  always 
    begin
      clk=1; #5; clk=0; #5; 
    end 

  // at start of test, load vectors and pulse reset
  initial 
    begin
      $readmemb("lightfsm.tv", testvectors); 
      vectornum = 0; errors = 0; hash = 0; reset = 1; #22; reset = 0; 
    end 

  // apply test vectors on rising edge of clk 
  always @(posedge clk) 
    begin
      #1; {left, right, expected} = testvectors[vectornum]; 
    end 

  // check results on falling edge of clk 
  always @(negedge clk) 
    if (~reset) begin    // skip during reset
      if ({la, lb, lc, ra, rb, rc} !== expected) begin // check result 
        $display("Error: inputs = %b", {left, right});
        $display(" outputs = %b %b %b %b %b %b (%b expected)", 
          la, lb, lc, ra, rb, rc, expected); 
        errors = errors + 1; 
      end
      vectornum = vectornum + 1;
      hash = hash ^ {la, lb, lc, ra, rb, rc};
      hash = {hash[5:0], hash[6] ^ hash[5]};
      if (testvectors[vectornum] === 8'bx) begin 
        $display("%d tests completed with %d errors", vectornum, errors); 
        $display("Hash: %h", hash);
        $stop; 
      end 
    end 
endmodule 
 
