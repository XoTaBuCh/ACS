module adventuregame(input  logic clk, reset,
                     input  logic n, s, e, w,
                     output logic win, die);
logic sword1, sword2;

roomfsm room(clk, reset, n, s, e, w, sword1, sword2, win, die);

swordfsm sword(clk, reset, sword2, sword1);
					
endmodule

module roomfsm(input logic clk, reset,
	       input logic n, s, e, w, sword_in,
               output logic sword_out, win, die);

  typedef enum logic [2:0] {CC, TT, RR, SS, DD, GG, VV} statetype;
  statetype state, nextstate;

  always @(posedge clk, posedge reset)
    begin
    	if (reset) 	state <= CC;
        else		state <= nextstate;
    end

  always_comb
    case (state)
      CC: if (e) nextstate = TT;
          else   nextstate = CC;
      TT: if (s) nextstate = RR;
          else if (w) nextstate = CC;
          else nextstate = TT;
      RR: if (w) nextstate = SS;
          else if (e) nextstate = DD;
	  else nextstate = RR;
      SS: if (e) nextstate = RR;
          else nextstate = SS;
      DD: if (sword_in) nextstate = VV;
          else nextstate = GG;
      GG: nextstate = GG;
      VV: nextstate = VV;
      default: nextstate = CC;
    endcase

  //output logic
  assign sword_out = (state == SS);
  assign win = (state == VV);
  assign die = (state == GG);

endmodule


module swordfsm(input logic clk, reset,
		input logic sword_in,
		output logic sword_out);

  logic state, nextstate;

  always @(posedge clk, posedge reset)
    begin
    	if (reset) 	state <= 0;
        else		state <= nextstate;
    end


   always_comb
     case (state)
       0: if (sword_in) nextstate = 1;
          else nextstate = 0;
       1: nextstate = 1;
     endcase

   assign sword_out = (state == 1);
endmodule

module testbench(); 
  logic        clk, reset;
  logic        n, s, e, w, win, die, winexpected, dieexpected;
  logic [31:0] vectornum, errors;
  logic [5:0]  testvectors[10000:0];
  logic [6:0]  hash;

  // instantiate device under test 
  adventuregame  dut(clk, reset, n, s, e, w, win, die); 

  // generate clock 
  always 
    begin
      clk=1; #5; clk=0; #5; 
    end 

  // at start of test, load vectors 
  // and pulse reset
  initial 
    begin
      $readmemb("adventuregame.tv", testvectors); 
      vectornum = 0; errors = 0; hash = 0; reset = 1; #22; reset = 0; #70; reset = 1; #10; reset = 0;
    end 

  // apply test vectors on rising edge of clk 
  always @(posedge clk) 
    begin
      #1; {n, s, e, w, winexpected, dieexpected} = testvectors[vectornum]; 
    end 

  // check results on falling edge of clk 
  always @(negedge clk) 
    if (~reset) begin    // skip during reset
      if (win !== winexpected || die !== dieexpected) begin // check result 
        $display("Error: inputs = %b", {n, s, e, w});
        $display(" state = %b", dut.room.state);
        $display(" outputs = %b %b (%b %b expected)", 
                 win, die, winexpected, dieexpected); 
        errors = errors + 1; 
      end
      hash = hash ^ {win, die};
      hash = {hash[5:0], hash[6]^hash[5]};
      vectornum = vectornum + 1;
      if (testvectors[vectornum] === 6'bx) begin 
        $display("%d tests completed with %d errors", vectornum, errors); 
        $display("hash: %h", hash);
        $stop; 
      end 
    end 
endmodule 
