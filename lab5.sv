// Author: Makenzie Brian
// ECE 272 Lab 5
// November 23, 2015

module LED_top(
	input reset_n,
	input [7:0] num,
	input clk,
	
	output [6:0] SEGS,
	output [2:0] SEL
		); 
	
		wire clk_5;	
		wire [3:0] digit0;
		wire [3:0] digit1;
		wire [3:0] digit2;
		wire [3:0] digit3;
		wire [3:0] digits;
	
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

module divider(
	input [15:0] data,
	
	output reg [7:0] num
	);
	
	always @(*)
		begin
			num = data/256;
		end
endmodule

module AD7705(input reset, 
				input MISO, 
				input clk, 
				output reg [15:0] ADC_data, 
				output MOSI,
				output CS, 
				output reg SCK, 
				output reg RST );
	
	wire[15:0] read_value; 
	wire finish_in; 
	wire [2:0] finish; 
	reg [2:0] out_select; /
	reg [7:0] data; 
	reg en_out; 
	reg en_in; 
	reg CS_dis; 
	reg [2:0] state; 
	reg [2:0] state_next; 
	
	parameter S0 = 3'b000; 
	parameter S1 = 3'b001; 
	parameter S2 = 3'b010; 
	parameter S3 = 3'b011;
	parameter S4 = 3'b100; 
	parameter S5 = 3'b101; 
	parameter S6 = 3'b110;
	
	always @(posedge clk, posedge reset)
		begin
			if(reset)
				begin
					state <= S0;
					RST <= 0; 
				end
			else 
				begin
					state <= state_next;
					RST <= 1;
				end
		end
	
	reg [7:0]count=0;
	
	always@(posedge clk)
		begin
			if(count == 1)	
				begin
					count <= 2;
					SCK = 1;
				end
			else if(count == 2)
				begin
					count <= 3;
					SCK <= 1;
				end
			else if(count == 3)
				begin
					count <= 4;
					SCK <= 0;
				end
			else if(count == 4)
				begin
					count <= 1;
					SCK <= 0;
				end
			else
				begin
					count <= 1;
					SCK <= 0;
				end
		end
	
	always @(posedge clk)
		begin
			if(state == S0)
 				begin
					CS_dis <= 0; 
					out_select <= 3'b000; 
					data <= 8'b00100000; 
					en_out <= 1; 
					en_in <= 0; 
					if(finish == 3'b000)
						state_next <= S1; 
					else 
						state_next <= S0; 
				end	
			else if(state == S1)
				begin
					out_select <= 3'b001; 
					data <= 8'b00001100; 
					en_out <= 1; 
					en_in <= 0; 
					if(finish == 3'b001) 
						state_next <= S2;
					else
						state_next <= S1;
				end
			else if(state == S2)
				begin
					out_select <= 3'b010; 
					data <= 8'b00010000; 
					en_out <= 1; 
					en_in <= 0; 
					if(finish == 3'b010)
						state_next <= S3;
					else
						state_next <= S2;
				end
			else if(state == S3)
				begin
					out_select <= 3'b011;
					data <= 8'b00000000;
					en_out <= 1; 
					en_in <= 0; 
					if(finish == 3'b011)
						state_next <= S4;
					else
						state_next <= S3;
				end
			else if(state == S4)
				begin
					out_select <= 3'b100;
					data <= 8'b00111000;
					en_out <= 1; 
					en_in <= 0; 
					if(finish == 3'b100) 
						state_next <= S5;
					else
						state_next <= S4;
				end
			else if(state == S5)
				begin
					en_in <= 1; 
					en_out <= 0; 
					if(finish_in) 
						state_next <= S6; 
					else
						state_next <= S5;
				end
			else if(state == S6)
				begin
					CS_dis <= 1; 
					en_in <= 0; 
					en_out <= 0; 
					ADC_data <= read_value; 
					state_next <= S0;
				end
			else
				state_next <= S0;
		end

	spi_output write(
		.en(en_out),
		.clk(clk),
		.SCK(SCK),
		.data(data),
		.finish(finish),
		.spi_output(MOSI),
		.out_select(out_select),
		.CS(CS),
		.CS_dis(CS_dis)
		);
		
	spi_input read(
		.en(en_in),
		.serial_in(MISO),
		.SCK(SCK),
		.data(read_value),
		.finish(finish_in)
		);	
Endmodule

module spi_input (input en, 
					input SCK, 
					input serial_in,
						output reg [15:0] data,
						output reg finish);

	/////////////////////////////////////////////////////////////////////////
	//State machine setup
	reg[4:0] state;
	
	parameter S0 = 5'b00000; // Waiting to read
	parameter S1 = 5'b00001; // First bit
	parameter S2 = 5'b00010; //	Second bit
	parameter S3 = 5'b00011; //
	parameter S4 = 5'b00100; //
	parameter S5 = 5'b00101; //
	parameter S6 = 5'b00110; //
	parameter S7 = 5'b00111; // etc... for 16 bits
	parameter S8 = 5'b01000; //
	parameter S9 = 5'b01001; //
	parameter S10 = 5'b01010; //
	parameter S11 = 5'b01011; //
	parameter S12 = 5'b01100; //
	parameter S13 = 5'b01101; //
	parameter S14 = 5'b01110; //
	parameter S15 = 5'b01111; //
	parameter S16 = 5'b10000; // Last bit
	parameter S17 = 5'b10001; // Set finish to high, output and reset state
	
	///////////////////////////////////////////////////////////////////////////
	//State machine transitions
	always @(posedge SCK)
		begin
			case(state)
				S0: if (en) state <= S1;//Waits for en to go high before recording inputs
				S1: state <= S2;//
				S2: state <= S3;//
				S3: state <= S4;//
				S4: state <= S5;//
				S5: state <= S6;//
				S6: state <= S7;//
				S7: state <= S8;//Each of these states, 1 to 16, is recieving a single bit and setting the corresponding data bit.
				S8: state <= S9;//
				S9: state <= S10;//
				S10: state <= S11;//
				S11: state <= S12;//
				S12: state <= S13;//
				S13: state <= S14;//
				S14: state <= S15;//
				S15: state <= S16;//
				S16: state <= S17;//
				S17: state <= S0;// Starts over back into idle until it is time to read more bytes
				default: state <= S0;
			endcase
		end
	
	//////////////////////////////////////////////////////////////////////////////
	//Recieving values
	always @(posedge SCK)
		begin
			case(state)
				S0: begin 
						data <= 0;
						finish <= 0;
					end
				S1: begin
						finish <= 0; // finish is set to 0 until state S17 when the entire two bytes has been recieved
						if(serial_in)//Recieve the first bit from the ADC
							data[15] <= 1;
						else
							data[15] <= 1;
					end
				S2: begin
						finish <= 0;
						if(serial_in)
							data[14] <= 1;
						else
							data[14] <= 0;	
					end
				S3: begin
						finish <= 0;
						if(serial_in)
							data[13] <= 1;
						else
							data[13] <= 0;
					end
				S4: begin
						finish <= 0;
						if(serial_in)
							data[12] <= 1;
						else
							data[12] <= 0;
					end
				S5: begin
						finish <= 0;
						if(serial_in)
							data[11] <= 1;
						else
							data[11] <= 0;
					end
				S6: begin
						finish <= 0;
						if(serial_in)
							data[10] <= 1;
						else
							data[10] <= 0;
					end
				S7: begin
						finish <= 0;
						if(serial_in)
							data[9] <= 1;
						else
							data[9] <= 0;
					end
				S8: begin
						finish <= 0;
						if(serial_in)
							data[8] <= 1;
						else
							data[8] <= 0;
					end
				S9: begin
						finish <= 0;
						if(serial_in)
							data[7] <= 1;
						else
							data[7] <= 0;
					end
				S10: begin
						finish <= 0;
						if(serial_in)
							data[6] <= 1;
						else
							data[6] <= 0;
					end
				S11: begin	
						finish <= 0;
						if(serial_in)
							data[5] <= 1;
						else
							data[5] <= 0;
					end
				S12: begin
						finish <= 0;
						if(serial_in)
							data[4] <= 1;
						else
							data[4] <= 0;
					end
				S13: begin
						finish <= 0;
						if(serial_in)
							data[3] <= 1;
						else
							data[3] <= 0;
					end
				S14: begin
						finish <= 0;
						if(serial_in)
							data[2] <= 1;
						else
							data[2] <= 0;
					end
				S15: begin
						finish <= 0;
						if(serial_in)
							data[1] <= 1;
						else
							data[1] <= 0;
					end
				S16: begin
						finish <= 0;
						if(serial_in)
							data[0] <= 1;
						else
							data[0] <= 0;
					end
				S17: begin
						finish <= 1;
					end
				default: data <= 0;
			endcase
		end
endmodule

module spi_output(input en, 
						input clk, 
						input SCK,
						input [7:0] data,
						input CS_dis,
						input [2:0] out_select,
						output CS,
						output reg [2:0] finish, 
						output spi_output
						);
	
	
	///////////////////////////////////////////////////////////////////
	//Wire and reg declarations
	reg serial_out;
	reg CS_temp;
	assign spi_output = serial_out;
	assign CS = CS_temp;
		
	///////////////////////////////////////////////////////////////////
	//State machine setup
	reg [3:0] state;
	
	parameter S0 = 4'b0000; //Idle state, waiting for en to go high
	parameter S1 = 4'b0001; //Send first bit
	parameter S2 = 4'b0010; //
	parameter S3 = 4'b0011; //
	parameter S4 = 4'b0100; //Send intermediate bits
	parameter S5 = 4'b0101; //
	parameter S6 = 4'b0110; //
	parameter S7 = 4'b0111; //
	parameter S8 = 4'b1000; //Send last bit
	parameter S9 = 4'b1001; //Enable finish and restart state machine
	
	///////////////////////////////////////////////////////////////////
	//State machine transitions
	always @(posedge SCK)
		begin
			case(state)
				S0: if(en) state <= S1; //If en 
				S1: state <= S2;//
				S2: state <= S3;//
				S3: state <= S4;//
				S4: state <= S5;//Transitions between states sequentially one bit during each state
				S5: state <= S6;//
				S6: state <= S7;//
				S7: state <= S8;//
				S8: begin
						if(en)//If en is still high skip the idle state and start outputing
							state <= S1;
						else
							state <= S0;
					end
				default: state <= S0;
			endcase
		end

	//////////////////////////////////////////////////////////////////
	//State machine outputing values
	always @(posedge clk)
		begin
			case(state)
				S0: begin 
						if(CS_dis) // Disabes CS
							CS_temp <= 1;
						serial_out <= 1; //Idles with output of 1
					end
				S1:	begin
						CS_temp <= 0;//Enable CS(active low)
						if(CS_dis) //If CS_dis goes high then CS is disabled
							CS_temp <= 1;
						if(data[7]) // Output data, most significant bit first
							serial_out <= 1'b1;
						else
							serial_out <= 1'b0;
					end
				S2: begin
						if(CS_dis) //If CS_dis goes high then CS is disbaled
							CS_temp <= 1;
						if(data[6]) //Output data
							serial_out <= 1'b1;
						else
							serial_out <= 1'b0;
					end
				S3: begin					
						if(CS_dis)
							CS_temp <= 1;
						if(data[5])
							serial_out <= 1'b1;
						else
							serial_out <= 1'b0;
					end
				S4: begin
						if(CS_dis)
							CS_temp <= 1;
						if(data[4])
							serial_out <= 1'b1;
						else
							serial_out <= 1'b0;
					end
				S5: begin
						if(CS_dis)
							CS_temp <= 1;
						if(data[3])
							serial_out <= 1'b1;
						else
							serial_out <= 1'b0;
					end
				S6: begin
						if(CS_dis)
							CS_temp <= 1;
						if(data[2])
							serial_out <= 1'b1;
						else
							serial_out <= 1'b0;
					end
				S7: begin
						if(CS_dis)
							CS_temp <= 1;
						if(data[1])
							serial_out <= 1'b1;
						else
							serial_out <= 1'b0;
					end
				S8: begin
						if(CS_dis)//Each state is the same besides which data bit is sent
							CS_temp <= 1;
						if(data[0])
							serial_out <= 1'b1;
						else
							serial_out <= 1'b0;
					end
				default: serial_out <= 1;
			endcase
		end
	
	/////////////////////////////////////////////////////////
	//State machine controlling finish signal
	always @(posedge SCK)
		begin
			if(state == S8) //If it is in the last state, because of the timing with SCK and clk finish is set high while the last bit is being sent
				begin
					case(out_select) // Set finish to out_select indicating that the bit has been transmited
						3'b000: finish <= 3'b000;
						3'b001: finish <= 3'b001;
						3'b010: finish <= 3'b010;
						3'b011: finish <= 3'b011;
						3'b100: finish <= 3'b100;
						default: finish <= 3'b111;
					endcase
				end
			else
				begin //Else finish is not set to out_select inicating that it is still transmitting
					finish <= 3'b111; 
				end
		end
	
	
endmodule



`timescale 1 ns / 1 ns

module toplevel_tf();

    reg reset;
    reg serial_in;

    wire serial_out;
    wire SCK;
    wire CS;
    wire RST;
    wire sign;
    wire [15:0] data;

    toplevel UUT (
        .serial_out(serial_out), 
        .SCK(SCK), 
        .CS(CS), 
        .RST(RST), 
        //.sign(sign), 
        .data(data), 
        .reset(reset), 
        .serial_in(serial_in)
        );

    initial begin
            reset = 0;
            serial_in = 0;
    end

endmodule // toplevel_tf


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
			0: digits = digit1;
			1: digits = digit2;
			3: digits = digit3;
			4: digits = digit0;
		endcase
	end
	
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
		digit2 <= num*169/16384 % 10;
		digit3 <= num*2197/2097152 % 10;
	end
endmodule


module SevenSegDriver ( 
	input [3:0] data,

	output reg [6:0] segments );
	
always @(*)
	case( data )
		0: segments = 7'b0000001;
		1: segments = 7'b1001111;
		2: segments = 7'b0010010;
		3: segments = 7'b0000110;
		4: segments = 7'b1001100;
		5: segments = 7'b0100100;
		6: segments = 7'b0100000;
		7: segments = 7'b0001111;
		8: segments = 7'b0000000;
		9: segments = 7'b0001100;
		default: segments = 7'b1111111;
	endcase
endmodule


module SevenSegmentDisplay(
	input clk_i,		//often, "tags" are added to variables to denote what they do for the user
	input reset_n,		//here, 'i' is used for input and 'o' for the output, while 'n' specifies an active low signal ("not") 
	
	output reg clk_o
		);
		
		reg [18:0] count;								//register stores the counter value so that it can be modified on a clock edge. register size needs to store as large of a number as the counter reaches
												 		//for this implementation, count must reach 415999, so 2^n >= 415999, n = 19
		
		always @ (posedge clk_i)			
			begin
				count <= count + 1;						//at every positive edge, the counter is increased by 1
				if(!reset_n)
					begin
						clk_o <= 0;
						count <= 0;						//if reset (active low) is pushed, the counter is reset
					end
				else
					if(count >= 5000)					//count value of greater than or equal to this value causes the output clock to be inverted. the resulting frequency will be input_frequency/(1+count_value)
						begin							//for this implementation, a frequency of 5 Hz was desired, so 2.08e6/5 - 1 = 415999
							clk_o <= ~clk_o;	
							count <= 0;					//resets the counter after the output clock has been inverted
						end
			end
endmodule



module toplevel(	
	output serial_out, SCK, CS, RST,
	output [15:0] data,
	output clk,
	input reset, serial_in/*synthesis syn_force_pads=1 syn_noprune=1*/
					);
					

	
	OSCH #("2.08") osc_int (					
			.STDBY(1'b0),							
			.OSC(clk),							
			.SEDSTDBY()			
			);	
			

	AD7705 ADC(
		.reset(reset),
		.MISO(serial_in),
		.MOSI(serial_out),
		.SCK(SCK),
		.CS(CS),
		.RST(RST),
		.ADC_data(data),
		.clk(clk)
		);
endmodule


module state_machine( //example of a Moore type state machine
	input clk_i,
	input reset_n,
	
	output reg [2:0] SEL
		); 
		
		//state and next state registers
		reg [1:0] state;
		reg [1:0] state_n;
		
		//each possible value of the state register is given a unique name for easier use later
		parameter S0 = 2'b00;
		parameter S1 = 2'b01;
		parameter S2 = 2'b10;
		parameter S3 = 2'b11;

		//asynchronous reset will set the state to the start, S0, otherwise, the state is changed on the positive edge of the clock signal
		always @ (posedge clk_i, negedge reset_n)
			begin
				if(!reset_n)
					state = S0;
				else
					state = state_n;

			end

		//this section defines what the next state should be for each possible state. in this implementation, it simply rotates through each state automatically
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
		
		//this is the output definition section of the state machine. outputs are based on which state the state machine is currently on
		//hex values are used for the LEDs since there are 8 of them and hex values are an effective way of represiting this size
		//numbers are represented by the number of bits (not digits!), an apostrophe, the base system, then the value
		//common bases are binary, b, octal, o, decimal, d, hexadecimal, h
		always @ (*)
			begin
				if(state == S0)
					SEL = 0;
				else if(state == S1)
					SEL = 1;
				else if(state == S2)
					SEL = 3;
				else
					SEL = 4;
			end
endmodule



module TOPMODULE(
	input reset,
	input serial_in,
	input reset_n,
	
	output [6:0] SEGS,
	output [2:0] SEL,
	output CS,
	output SCK,
	output D_in,
	output RST);
	
	wire [7:0] num;
	wire [15:0] data;
	wire clk;
	
	
	toplevel toppo(
		.reset(reset),
		.serial_in(serial_in),
		.serial_out(D_in),
		.CS(CS),
		.SCK(SCK),
		.RST(RST),
		.data(data),
		.clk(clk));
	
	divider eldividero(
		.data(data),
		.num(num));
	
	LED_top suspense(
		.reset_n(reset_n),
		.clk(clk),
		.num(num),
		.SEGS(SEGS),
		.SEL(SEL));
		
endmodule

