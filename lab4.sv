// Author: Makenzie Brian
// ECE 272 Lab 4
// November 9, 2015

module SevenSegmentDisplay(
	input clk_i,		
	input reset_n,		 
	output reg clk_o
		);
		reg [18:0] count;							
		always @ (posedge clk_i, negedge reset_n)			
			begin
				count <= count + 1;						
				if(!reset_n)
					begin
						clk_o <= 0;
						count <= 0;						
					end
				else
					if(count >= 300000)					
						begin						
							clk_o <= ~clk_o;	
							count <= 0;				
						end
			end
endmodule

module state_machine( //example of a Moore type state machine
	input clk_i,
	input reset_n,
	output reg [2:0] SEL
		); 
		reg [1:0] state;
		reg [1:0] state_n;
		parameter S0 = 2'b00;
		parameter S1 = 2'b01;
		parameter S2 = 2'b10;
		parameter S3 = 2'b11;
		always @ (posedge clk_i, negedge reset_n)
			begin
				if(!reset_n)
					state = S0;
				else
					state = state_n;
			end
		always @ (*)
			begin
				case(state)
					S0: state_n = S1;
					S1: state_n = S2;
					S2: state_n = S3;
					S3:	state_n = S0;
					
					default: state_n = S0;
				endcase
			end
		always @ (*)
			begin
				if(state == S0)
					SEL = 3'b000;
				else if(state == S1)
					SEL = 3'b001;
				else if(state == S2)
					SEL = 3'b011;
				else
					SEL = 3'b100;
			end
endmodule

module LED_top_module(
	input reset_n,
	input [7:0] num,
	
	output [6:0] SEGS
		); 
		
		wire clk;		
		wire clk_5;		
		wire [3:0] digit0;
		wire [3:0] digit1;
		wire [3:0] digit2;
		wire [3:0] digit3;
		wire [3:0] digits;
	
		OSCH #("2.03") osc_int (					
			.STDBY(1'b0),						
			.OSC(clk),							
			.SEDSTDBY());						
		
		SevenSegmentDisplay counter_1(
			.clk_i(clk),
			.reset_n(reset_n),
			.clk_o(clk_5));
			
		state_machine FSM_1(
			.clk_i(clk_5),
			.reset_n(reset_n),
			.SEL(SEL));
			
		DigitParser parsley(
			.clk_i(clk_5),
			.num(num),
			.digit0(digit0),
			.digit1(digit1),
			.digit2(digit2),
			.digit3(digit3));
			
		DigitMux muxifier(
			.clk_i(clk_5),
			.SEL(SEL),
			.digit0(digit0),
			.digit1(digit1),
			.digit2(digit2),
			.digit3(digit3),
			.digits(digits));

		SevenSegDriver vroomvroom(
			.data(digits),
			.segments(SEGS));
endmodule

module DigitParser(
	input [7:0] num,
	input clk_i,
	
	output reg [3:0] digit0,
	output reg [3:0] digit1,
	output reg [3:0] digit2,
	output reg [3:0] digit3
		);
always @ (posedge clk_i)
	begin
		digit0 <= num % 10;
		digit1 <= num*13/128 % 10;
		digit2 <= num*13/128*13/128 % 10;
		digit3 <= num*13/128*13/128*13/128 % 10;
	end
endmodule

module DigitMux(
	input wire [3:0] digit0,
	input wire [3:0] digit1,
	input wire [3:0] digit2,
	input wire [3:0] digit3,
	input clk_i,
	input wire [2:0] SEL,
	output reg [3:0] digits
		);

always @(posedge clk_i)
	begin	
		case( SEL )
			0: digits[0] = digit0;
			1: digits[1] = digit1;
			2: digits[2] = digit2;
			3: digits[3] = digit3;
		endcase
	end
endmodule

module SevenSegDriver ( 
	input [3:0] data,

	output reg [6:0] segments );
	
always @(*)
	case( data )
		0: segments = 7'b1111110;
		1: segments = 7'b0110000;
		2: segments = 7'b1101101;
		3: segments = 7'b1111001;
		4: segments = 7'b0110011;
		5: segments = 7'b1011011;
		6: segments = 7'b1011111;
		7: segments = 7'b1110000;
		8: segments = 7'b1111111;
		9: segments = 7'b1110011;
		default: segments = 7'b0000000;
	endcase
endmodule
