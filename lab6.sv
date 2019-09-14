// Author: Makenzie Brian
// ECE 272 Lab 6
// December 4, 2015
// Builds on lab 4 code

module ALU(
	input [3:0] a,b,
	input [5:0] op,
	input clk,
	output reg [7:0] out);
	
	always @(posedge clk) begin
			case(op)
				6'b111110: out = a + b;   //add
				6'b111101: out = a - b;   //subtract
				6'b111011: out = a / b;   //a divided by 
				6'b110111: out = b % a;   //b modulus a
				6'b101111: out = a % b;   //a modulus b 
				6'b011111: out = b * a;   //multiply
				default: out = 255;
			endcase
		end
endmodule


module TOPPO(
	input [3:0] a,b,
	input [5:0] op,
	input reset_n,

	output [6:0] SEGS,
	output [2:0] SEL);
	
	wire [7:0] num;
	wire clkk;
	
	OSCH #("2.08") osc_int (					
			.STDBY(1'b0),							
			.OSC(clkk),							
			.SEDSTDBY()			
			);	
	
	LED_top top1(
		.reset_n(reset_n),
		.num(num),
		.clk(clkk),
		.SEGS(SEGS),
		.SEL(SEL));
		
	ALU alu(
		.a(a),
		.b(b),
		.clk(clkk),
		.op(op),
		.out(num));
	
endmodule
