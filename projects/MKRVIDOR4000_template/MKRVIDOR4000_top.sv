/*
* Copyright 2018 ARDUINO SA (http://www.arduino.cc/)
* This file is part of Vidor IP.
* Copyright (c) 2018
* Authors: Dario Pennisi
*
* This software is released under:
* The GNU General Public License, which covers the main part of 
* Vidor IP
* The terms of this license can be found at:
* https://www.gnu.org/licenses/gpl-3.0.en.html
*
* You can be released from the requirements of the above licenses by purchasing
* a commercial license. Buying such a license is mandatory if you want to modify or
* otherwise use the software for commercial activities involving the Arduino
* software without disclosing the source code of your own applications. To purchase
* a commercial license, send an email to license@arduino.cc.
*
*/

module MKRVIDOR4000_top
(
  // system signals
  input         iCLK,
  input         iRESETn,
  input         iSAM_INT,
  output        oSAM_INT,
  
  // SDRAM
  output        oSDRAM_CLK,
  output [11:0] oSDRAM_ADDR,
  output [1:0]  oSDRAM_BA, 
  output        oSDRAM_CASn,
  output        oSDRAM_CKE,
  output        oSDRAM_CSn,
  inout  [15:0] bSDRAM_DQ,
  output [1:0]  oSDRAM_DQM,
  output        oSDRAM_RASn,
  output        oSDRAM_WEn,

  // SAM D21 PINS
  inout         bMKR_AREF,  
  inout  [6:0]  bMKR_A,
  inout  [14:0] bMKR_D,
  
  // Mini PCIe
  inout         bPEX_RST,
  inout         bPEX_PIN6,
  inout         bPEX_PIN8,
  inout         bPEX_PIN10,
  input         iPEX_PIN11,
  inout         bPEX_PIN12,
  input         iPEX_PIN13,
  inout         bPEX_PIN14,
  inout         bPEX_PIN16,
  inout         bPEX_PIN20,
  input         iPEX_PIN23,
  input         iPEX_PIN25,
  inout         bPEX_PIN28,
  inout         bPEX_PIN30,
  input         iPEX_PIN31,
  inout         bPEX_PIN32,
  input         iPEX_PIN33,
  inout         bPEX_PIN42,
  inout         bPEX_PIN44,
  inout         bPEX_PIN45,
  inout         bPEX_PIN46,
  inout         bPEX_PIN47,
  inout         bPEX_PIN48,
  inout         bPEX_PIN49,
  inout         bPEX_PIN51,

  // NINA interface
  inout         bWM_PIO1,
  inout         bWM_PIO2,
  inout         bWM_PIO3,
  inout         bWM_PIO4,
  inout         bWM_PIO5,
  inout         bWM_PIO7,
  inout         bWM_PIO8,
  inout         bWM_PIO18,
  inout         bWM_PIO20,
  inout         bWM_PIO21,
  inout         bWM_PIO27,
  inout         bWM_PIO28,
  inout         bWM_PIO29,
  inout         bWM_PIO31,
  input         iWM_PIO32,
  inout         bWM_PIO34,
  inout         bWM_PIO35,
  inout         bWM_PIO36,
  input         iWM_TX,
  inout         oWM_RX,
  inout         oWM_RESET,

  // HDMI output
  output [2:0]  oHDMI_TX,
  output        oHDMI_CLK,

  inout         bHDMI_SDA,
  inout         bHDMI_SCL,
  
  input         iHDMI_HPD,
  
  // MIPI input
  input  [1:0]  iMIPI_D,
  input         iMIPI_CLK,
  inout         bMIPI_SDA,
  inout         bMIPI_SCL,
  inout  [1:0]  bMIPI_GP,

  // Q-SPI Flash interface
  output        oFLASH_SCK,
  output        oFLASH_CS,
  inout         oFLASH_MOSI,
  inout         iFLASH_MISO,
  inout         oFLASH_HOLD,
  inout         oFLASH_WP

);

assign bMKR_D[5:0] = PHASES;
reg [5:0] PHASES;
reg [10:0] pwm_delay;
reg [10:0] pwm;

always @(posedge wCLK24) begin: BLDC_COMMUTATION
	if(pwm_delay>(2047-pwm))begin
		if(bMKR_A[4] && ~bMKR_A[5] && bMKR_A[6]) begin
			PHASES <= 6'b100100;
		end 
		if(bMKR_A[4] && ~bMKR_A[5] && ~bMKR_A[6])begin
			PHASES <= 6'b100001;
		end 
		if(bMKR_A[4] && bMKR_A[5] && ~bMKR_A[6]) begin
			PHASES <= 6'b001001;
		end 
		if(~bMKR_A[4] && bMKR_A[5] && ~bMKR_A[6])begin
			PHASES <= 6'b011000;
		end 
		if(~bMKR_A[4] && bMKR_A[5] && bMKR_A[6]) begin
			PHASES <= 6'b010010;
		end 	
		if(~bMKR_A[4] && ~bMKR_A[5] && bMKR_A[6])begin
			PHASES <= 6'b000110;
		end 
	end else begin
		PHASES <= 0;
	end
	pwm_delay <= pwm_delay+1;
end

wire encoder_A, encoder_B, encoder_A_rising_edge, encoder_B_rising_edge;
assign encoder_A = bMKR_A[3];
assign encoder_B = bMKR_A[2];
assign encoder_A_rising_edge = (bMKR_A[3] && !encoder_A_prev);
assign encoder_B_rising_edge = (bMKR_A[2] && !encoder_B_prev);

reg encoder_A_prev, encoder_B_prev;

wire signed [31:0] position;

rot_enc_flt encoder_0(wCLK24, 0, 0, bMKR_A[3], bMKR_A[2], position);

//always @(posedge bMKR_A[3]) begin: OPTICAL_ENCODER
//	position <= position +1;
////	if(encoder_0_B_prev==1 && encoder_0_A_prev==0 && encoder_0_A) begin
////		position <= position +1;
////	end
////	if(encoder_0_A_prev && encoder_0_B_prev && ~encoder_0_B) begin
////		position <= position +1;
////	end
////	if(encoder_0_A_prev && ~encoder_0_A && ~encoder_0_B) begin
////		position <= position +1;
////	end
//	
////	if(encoder_0_B_prev && ~encoder_0_B && encoder_0_A) begin
////		position <= position -1;
////	end
////	if(~encoder_0_B_prev && encoder_0_A_prev && ~encoder_0_A) begin
////		position <= position -1;
////	end
////	if(~encoder_0_A_prev && ~encoder_0_B_prev && encoder_0_B) begin
////		position <= position -1;
////	end
////	if(~encoder_0_A_prev && encoder_0_A && encoder_0_B) begin
////		position <= position -1;
////	end
//end

wire [7:0] data_out;
wire [7:0] data_in;
assign data_in = fpga_to_samd[byte_counter];
reg [7:0] samd_to_fpga[19:0];
reg [7:0] fpga_to_samd[19:0];
wire data_out_valid;
reg data_out_valid_prev;
reg write;

spi_slave #(8,1'b0,1'b0,3) spi_slave0(
	.clk_i(wCLK24),
	.spi_sck_i(bMKR_D[9]),
   .spi_ssel_i(bMKR_D[7]),
   .spi_mosi_i(bMKR_D[8]),
   .spi_miso_o(bMKR_D[10]), 
   .di_i(data_in),
   .wren_i(write),
   .do_valid_o(data_out_valid),
   .do_o(data_out)
);

reg [31:0] byte_counter;

localparam IDLE = 0, WAIT_FOR_FRAME_TRANSMISSION = 1;
reg [7:0] state = IDLE;
integer j;

always @(posedge wCLK24) begin: SPICONTROL_SPILOGIC
	data_out_valid_prev <= data_out_valid;
	write <= 0;
	case(state) 
		IDLE: begin
			byte_counter <= 0;
			if(bMKR_D[6]==0) begin
				state=WAIT_FOR_FRAME_TRANSMISSION;
				write <= 1;
			end
		end
		WAIT_FOR_FRAME_TRANSMISSION: begin
			if(bMKR_D[6]==0) begin // receiving 
				if(data_out_valid_prev==0 && data_out_valid==1) begin
					samd_to_fpga[byte_counter] <= data_out;
					if(byte_counter<2)begin
						fpga_to_samd[byte_counter] <= data_out;
					end
					byte_counter <= byte_counter+1;
					write <= 1;
				end
			end else begin // receiving done
				pwm <= {samd_to_fpga[3],samd_to_fpga[2],samd_to_fpga[1],samd_to_fpga[0]};
				fpga_to_samd[4] <= position[7:0];
				fpga_to_samd[5] <= position[15:8];
				fpga_to_samd[6] <= position[23:16];
				fpga_to_samd[7] <= position[31:24];
				state <= IDLE;
			end 
		end
	endcase 
end
 
 
// signal declaration

wire        wOSC_CLK;

wire        wCLK8,wCLK24, wCLK64, wCLK120;

wire [31:0] wJTAG_ADDRESS, wJTAG_READ_DATA, wJTAG_WRITE_DATA, wDPRAM_READ_DATA;
wire        wJTAG_READ, wJTAG_WRITE, wJTAG_WAIT_REQUEST, wJTAG_READ_DATAVALID;
wire [4:0]  wJTAG_BURST_COUNT;
wire        wDPRAM_CS;

wire [7:0]  wDVI_RED,wDVI_GRN,wDVI_BLU;
wire        wDVI_HS, wDVI_VS, wDVI_DE;

wire        wVID_CLK, wVID_CLKx5;
wire        wMEM_CLK;

assign wVID_CLK   = wCLK24;
assign wVID_CLKx5 = wCLK120;
assign wCLK8      = iCLK;

// internal oscillator
cyclone10lp_oscillator   osc
  ( 
  .clkout(wOSC_CLK),
  .oscena(1'b1));

// system PLL
SYSTEM_PLL PLL_inst(
  .areset(1'b0),
  .inclk0(wCLK8),
  .c0(wCLK24),
  .c1(wCLK120),
  .c2(wMEM_CLK),
  .c3(oSDRAM_CLK),
  .locked());


reg [5:0] rRESETCNT;

always @(posedge wMEM_CLK)
begin
  if (!rRESETCNT[5])
  begin
  rRESETCNT<=rRESETCNT+1;
  end
end

endmodule
