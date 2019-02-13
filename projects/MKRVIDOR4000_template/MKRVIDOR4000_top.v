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

parameter NUMBER_OF_MOTORS = 6;

reg [15:0] avalon_address;
reg        avalon_write;
reg [31:0] avalon_writedata;
reg        avalon_read;
reg [31:0] avalon_readdata;
wire 		  avalon_waitrequest;

//assign bMKR_D[0] = wOSC_CLK;
MYOControl #(NUMBER_OF_MOTORS,24_000_000,0) (
	.clock(wCLK24),
	.miso(bMKR_D[1]),                 // myocontrol_0_conduit_end.miso
	.mosi(bMKR_D[0]),                 //                         .mosi
	.sck(bMKR_D[2]),                  //                         .sck
	.ss_n_o(bMKR_D[7:3]),                 //                         .ss_n
	.power_sense_n(1'b0),        //                         .power_sense_n
	.reset(1'b0),      
	.address(avalon_address),           // myocontrol_0_avalon_slave_0.address
	.write(avalon_write),             //                            .write
	.writedata(avalon_writedata),         //                            .writedata
	.read(avalon_read),              //                            .read
	.readdata(avalon_readdata),          //                            .readdata
	.waitrequest(avalon_waitrequest)  
);

spi_slave #(8,1'b0,1'b0,3) (
	.clk_i(wCLK24),
	.spi_sck_i(bMKR_D[9]),
   .spi_ssel_i(bMKR_A[1]),
   .spi_mosi_i(bMKR_D[8]),
   .spi_miso_o(bMKR_D[10]),
   .di_i(data_in),
   .wren_i(write),
   .do_valid_o(data_out_valid),
   .do_o(data_out)
);

reg write;
wire [7:0] data_out;
wire [7:0] data_in;
assign data_in = fpga_to_samd[byte_counter];
reg [7:0] samd_to_fpga[139:0];
reg [7:0] fpga_to_samd[139:0];
wire data_out_valid;
reg data_out_valid_prev;

reg [31:0] byte_counter;

localparam IDLE = 0,  WRITE = 1, READ = 2, WAITREQUEST_WRITE = 3, WAITREQUEST_READ = 4, WAIT_FOR_FRAME_TRANSMISSION = 5;
reg [7:0] state = IDLE;
reg [7:0] read_val;
reg [7:0] write_val;

always @(posedge wCLK24) begin: SPICONTROL_SPILOGIC
	write <= 0;
	data_out_valid_prev <= data_out_valid;
	case(state) 
		IDLE: begin
			byte_counter <= 0;
			if(bMKR_A[0]==0) begin
				state=WAIT_FOR_FRAME_TRANSMISSION;
				write <= 1;
			end
		end
		WAIT_FOR_FRAME_TRANSMISSION: begin
			if(bMKR_A[0]==0) begin // receiving 
				if(data_out_valid_prev==0 && data_out_valid==1) begin
					samd_to_fpga[byte_counter] = data_out;
					byte_counter <= byte_counter+1;
					write <= 1;
				end
			end else begin // receiving done
				avalon_write <= 0;
				avalon_read <= 0;
				state <= WRITE;
				write_val <= 0;
				read_val <= 0;
			end
		end
		WRITE: begin
			// lets set the address for the respective values
			case(write_val)
				// Kp
				0: avalon_address <= {8'h00, 8'h00};
				1: avalon_address <= {8'h00, 8'h01};
				2: avalon_address <= {8'h00, 8'h02};
				3: avalon_address <= {8'h00, 8'h03};
				4: avalon_address <= {8'h00, 8'h04};
				5: avalon_address <= {8'h00, 8'h05};
				// Ki
				6: avalon_address <= {8'h01, 8'h00};
				7: avalon_address <= {8'h01, 8'h01};
				8: avalon_address <= {8'h01, 8'h02};
				9: avalon_address <= {8'h01, 8'h03};
				10: avalon_address <= {8'h01, 8'h04};
				11: avalon_address <= {8'h01, 8'h05};
				// Kd
				12: avalon_address <= {8'h02, 8'h00};
				13: avalon_address <= {8'h02, 8'h01};
				14: avalon_address <= {8'h02, 8'h02};
				15: avalon_address <= {8'h02, 8'h03};
				16: avalon_address <= {8'h02, 8'h04};
				17: avalon_address <= {8'h02, 8'h05};
				// sp
				18: avalon_address <= {8'h03, 8'h00};
				19: avalon_address <= {8'h03, 8'h01};
				20: avalon_address <= {8'h03, 8'h02};
				21: avalon_address <= {8'h03, 8'h03};
				22: avalon_address <= {8'h03, 8'h04};
				23: avalon_address <= {8'h03, 8'h05};
				// outputPosMax
				24: avalon_address <= {8'h05, 8'h00};
				25: avalon_address <= {8'h05, 8'h01};
				26: avalon_address <= {8'h05, 8'h02};
				27: avalon_address <= {8'h05, 8'h03};
				28: avalon_address <= {8'h05, 8'h04};
				29: avalon_address <= {8'h05, 8'h05};
				// outputNegMax
				30: avalon_address <= {8'h06, 8'h00};
				31: avalon_address <= {8'h06, 8'h01};
				32: avalon_address <= {8'h06, 8'h02};
				33: avalon_address <= {8'h06, 8'h03};
				34: avalon_address <= {8'h06, 8'h04};
				35: avalon_address <= {8'h06, 8'h05};
				// IntegralPosMax
				36: avalon_address <= {8'h07, 8'h00};
				37: avalon_address <= {8'h07, 8'h01};
				38: avalon_address <= {8'h07, 8'h02};
				39: avalon_address <= {8'h07, 8'h03};
				40: avalon_address <= {8'h07, 8'h04};
				41: avalon_address <= {8'h07, 8'h05};
				// IntegralNegMax
				42: avalon_address <= {8'h08, 8'h00};
				43: avalon_address <= {8'h08, 8'h01};
				44: avalon_address <= {8'h08, 8'h02};
				45: avalon_address <= {8'h08, 8'h03};
				46: avalon_address <= {8'h08, 8'h04};
				47: avalon_address <= {8'h08, 8'h05};
				// deadband
				48: avalon_address <= {8'h09, 8'h00};
				49: avalon_address <= {8'h09, 8'h01};
				50: avalon_address <= {8'h09, 8'h02};
				51: avalon_address <= {8'h09, 8'h03};
				52: avalon_address <= {8'h09, 8'h04};
				53: avalon_address <= {8'h09, 8'h05};
				// reset_myo_control
				54: avalon_address <= {8'h0B, 8'h00};
				// spi_activate
				55: avalon_address <= {8'h0C, 8'h00};
				// reset_controller
				56: avalon_address <= {8'h0D, 8'h00};
				57: avalon_address <= {8'h0D, 8'h01};
				58: avalon_address <= {8'h0D, 8'h02};
				59: avalon_address <= {8'h0D, 8'h03};
				60: avalon_address <= {8'h0D, 8'h04};
				61: avalon_address <= {8'h0D, 8'h05};
				// control_mode0
				62: avalon_address <= {8'h0A, 8'h00};
				63: avalon_address <= {8'h0A, 8'h01};
				64: avalon_address <= {8'h0A, 8'h02};
				65: avalon_address <= {8'h0A, 8'h03};
				66: avalon_address <= {8'h0A, 8'h04};
				67: avalon_address <= {8'h0A, 8'h05};
				// outputDivider
				68: avalon_address <= {8'h14, 8'h00};
				69: avalon_address <= {8'h14, 8'h01};
				70: avalon_address <= {8'h14, 8'h02};
				71: avalon_address <= {8'h14, 8'h03};
				72: avalon_address <= {8'h14, 8'h04};
				73: avalon_address <= {8'h14, 8'h05};
			endcase 
			
			case(write_val)
				// Kp
				0: avalon_writedata <= {samd_to_fpga[1],samd_to_fpga[0]};
				1: avalon_writedata <= {samd_to_fpga[3],samd_to_fpga[2]};
				2: avalon_writedata <= {samd_to_fpga[5],samd_to_fpga[4]};
				3: avalon_writedata <= {samd_to_fpga[7],samd_to_fpga[6]};
				4: avalon_writedata <= {samd_to_fpga[9],samd_to_fpga[8]};
				5: avalon_writedata <= {samd_to_fpga[11],samd_to_fpga[10]};
				// Ki
				6: avalon_writedata <= {samd_to_fpga[13],samd_to_fpga[12]};
				7: avalon_writedata <= {samd_to_fpga[15],samd_to_fpga[14]};
				8: avalon_writedata <= {samd_to_fpga[17],samd_to_fpga[16]};
				9: avalon_writedata <= {samd_to_fpga[19],samd_to_fpga[18]};
				10: avalon_writedata <= {samd_to_fpga[21],samd_to_fpga[20]};
				11: avalon_writedata <= {samd_to_fpga[23],samd_to_fpga[22]};
				// Kd
				12: avalon_writedata <= {samd_to_fpga[25],samd_to_fpga[24]};
				13: avalon_writedata <= {samd_to_fpga[27],samd_to_fpga[26]};
				14: avalon_writedata <= {samd_to_fpga[29],samd_to_fpga[28]};
				15: avalon_writedata <= {samd_to_fpga[31],samd_to_fpga[30]};
				16: avalon_writedata <= {samd_to_fpga[33],samd_to_fpga[32]};
				17: avalon_writedata <= {samd_to_fpga[35],samd_to_fpga[34]};
				// sp
				18: avalon_writedata <= {samd_to_fpga[39],samd_to_fpga[38],samd_to_fpga[37],samd_to_fpga[36]};
				19: avalon_writedata <= {samd_to_fpga[43],samd_to_fpga[42],samd_to_fpga[41],samd_to_fpga[40]};
				20: avalon_writedata <= {samd_to_fpga[47],samd_to_fpga[46],samd_to_fpga[45],samd_to_fpga[44]};
				21: avalon_writedata <= {samd_to_fpga[51],samd_to_fpga[50],samd_to_fpga[49],samd_to_fpga[48]};
				22: avalon_writedata <= {samd_to_fpga[55],samd_to_fpga[54],samd_to_fpga[53],samd_to_fpga[52]};
				23: avalon_writedata <= {samd_to_fpga[59],samd_to_fpga[58],samd_to_fpga[57],samd_to_fpga[56]};
				// outputPosMax
				24: avalon_writedata <= {samd_to_fpga[61],samd_to_fpga[60]};
				25: avalon_writedata <= {samd_to_fpga[63],samd_to_fpga[62]};
				26: avalon_writedata <= {samd_to_fpga[65],samd_to_fpga[64]};
				27: avalon_writedata <= {samd_to_fpga[67],samd_to_fpga[66]};
				28: avalon_writedata <= {samd_to_fpga[69],samd_to_fpga[68]};
				29: avalon_writedata <= {samd_to_fpga[70],samd_to_fpga[70]};
				// outputNegMax
				30: avalon_writedata <= {samd_to_fpga[73],samd_to_fpga[72]};
				31: avalon_writedata <= {samd_to_fpga[75],samd_to_fpga[74]};
				32: avalon_writedata <= {samd_to_fpga[77],samd_to_fpga[76]};
				33: avalon_writedata <= {samd_to_fpga[79],samd_to_fpga[78]};
				34: avalon_writedata <= {samd_to_fpga[81],samd_to_fpga[80]};
				35: avalon_writedata <= {samd_to_fpga[83],samd_to_fpga[82]};
				// IntegralPosMax
				36: avalon_writedata <= {samd_to_fpga[85],samd_to_fpga[84]};
				37: avalon_writedata <= {samd_to_fpga[87],samd_to_fpga[86]};
				38: avalon_writedata <= {samd_to_fpga[89],samd_to_fpga[88]};
				39: avalon_writedata <= {samd_to_fpga[91],samd_to_fpga[90]};
				40: avalon_writedata <= {samd_to_fpga[93],samd_to_fpga[92]};
				41: avalon_writedata <= {samd_to_fpga[95],samd_to_fpga[94]};
				// IntegralNegMax
				42: avalon_writedata <= {samd_to_fpga[97],samd_to_fpga[96]};
				43: avalon_writedata <= {samd_to_fpga[99],samd_to_fpga[98]};
				44: avalon_writedata <= {samd_to_fpga[101],samd_to_fpga[100]};
				45: avalon_writedata <= {samd_to_fpga[103],samd_to_fpga[102]};
				46: avalon_writedata <= {samd_to_fpga[105],samd_to_fpga[104]};
				47: avalon_writedata <= {samd_to_fpga[107],samd_to_fpga[106]};
				// deadband
				48: avalon_writedata <= {samd_to_fpga[109],samd_to_fpga[108]};
				49: avalon_writedata <= {samd_to_fpga[111],samd_to_fpga[110]};
				50: avalon_writedata <= {samd_to_fpga[113],samd_to_fpga[112]};
				51: avalon_writedata <= {samd_to_fpga[115],samd_to_fpga[114]};
				52: avalon_writedata <= {samd_to_fpga[117],samd_to_fpga[116]};
				53: avalon_writedata <= {samd_to_fpga[119],samd_to_fpga[118]};
				// reset_myo_control
				54: avalon_writedata <= samd_to_fpga[120];
				// spi_activate
				55: avalon_writedata <= samd_to_fpga[121];
				// reset_controller
				56: avalon_writedata <= samd_to_fpga[122];
				57: avalon_writedata <= samd_to_fpga[123];
				58: avalon_writedata <= samd_to_fpga[124];
				59: avalon_writedata <= samd_to_fpga[125];
				60: avalon_writedata <= samd_to_fpga[126];
				61: avalon_writedata <= samd_to_fpga[127];
				// control_mode
				62: avalon_writedata <= samd_to_fpga[128];
				63: avalon_writedata <= samd_to_fpga[129];
				64: avalon_writedata <= samd_to_fpga[130];
				65: avalon_writedata <= samd_to_fpga[131];
				66: avalon_writedata <= samd_to_fpga[132];
				67: avalon_writedata <= samd_to_fpga[133];
				// outputDivider
				68: avalon_writedata <= samd_to_fpga[134];
				69: avalon_writedata <= samd_to_fpga[135];
				70: avalon_writedata <= samd_to_fpga[136];
				71: avalon_writedata <= samd_to_fpga[137];
				72: avalon_writedata <= samd_to_fpga[138];
				73: avalon_writedata <= samd_to_fpga[139];
			endcase 
			write_val <= write_val + 1;
			avalon_write <= 1;
			state <= WAITREQUEST_WRITE;
		end
		WAITREQUEST_WRITE: begin
				if(avalon_waitrequest==0) begin // myoControl could be requesting to wait, and we wait...
					avalon_write <= 0;
					if(write_val <= 49) begin
						state <= WRITE;
					end else begin
						state <= READ; // if we are done we move to read state
					end
				end
		end
		READ: begin
			if(read_val<=19) begin
				// lets set the address for the respective values
				case(read_val)
					// position
					0: avalon_address <= {8'h0B, 8'h00};
					1: avalon_address <= {8'h0B, 8'h01};
					2: avalon_address <= {8'h0B, 8'h02};
					3: avalon_address <= {8'h0B, 8'h03};
					4: avalon_address <= {8'h0B, 8'h04};
					5: avalon_address <= {8'h0B, 8'h05};
					// velocity
					6: avalon_address <= {8'h0C, 8'h00};
					7: avalon_address <= {8'h0C, 8'h01};
					8: avalon_address <= {8'h0C, 8'h02};
					9: avalon_address <= {8'h0C, 8'h03};
					10: avalon_address <= {8'h0C, 8'h04};
					11: avalon_address <= {8'h0C, 8'h05};
					// displacement
					12: avalon_address <= {8'h0E, 8'h00};
					13: avalon_address <= {8'h0E, 8'h01};
					14: avalon_address <= {8'h0E, 8'h02};
					15: avalon_address <= {8'h0E, 8'h03};
					16: avalon_address <= {8'h0E, 8'h04};
					17: avalon_address <= {8'h0E, 8'h05};
					// current
					18: avalon_address <= {8'h0D, 8'h00};
					19: avalon_address <= {8'h0D, 8'h01};
					20: avalon_address <= {8'h0D, 8'h02};
					21: avalon_address <= {8'h0D, 8'h03};
					22: avalon_address <= {8'h0D, 8'h04};
					23: avalon_address <= {8'h0D, 8'h05};
					// pwmRef
					24: avalon_address <= {8'h0F, 8'h00};
					25: avalon_address <= {8'h0F, 8'h01};
					26: avalon_address <= {8'h0F, 8'h02};
					27: avalon_address <= {8'h0F, 8'h03};
					28: avalon_address <= {8'h0F, 8'h04};
					29: avalon_address <= {8'h0F, 8'h05};
				endcase
				avalon_read <= 1;
				state <= WAITREQUEST_READ;
			end else begin
				state <= IDLE; // go back to IDLE
			end
		end
		WAITREQUEST_READ: begin
			if(avalon_waitrequest==0) begin // myoControl could be requesting to wait, and we wait...
				avalon_read <= 0;
				state <= READ;
				case(read_val)
					// position
					0: {fpga_to_samd[3],fpga_to_samd[2],fpga_to_samd[1],fpga_to_samd[0]} <= avalon_readdata;
					1: {fpga_to_samd[7],fpga_to_samd[6],fpga_to_samd[5],fpga_to_samd[4]} <= avalon_readdata;
					2: {fpga_to_samd[11],fpga_to_samd[10],fpga_to_samd[9],fpga_to_samd[8]} <= avalon_readdata;
					3: {fpga_to_samd[15],fpga_to_samd[14],fpga_to_samd[13],fpga_to_samd[12]} <= avalon_readdata;
					4: {fpga_to_samd[19],fpga_to_samd[18],fpga_to_samd[17],fpga_to_samd[16]} <= avalon_readdata;
					5: {fpga_to_samd[23],fpga_to_samd[22],fpga_to_samd[21],fpga_to_samd[20]} <= avalon_readdata;
					// velocity
					6: {fpga_to_samd[25],fpga_to_samd[24]} <= avalon_readdata;
					7: {fpga_to_samd[27],fpga_to_samd[26]} <= avalon_readdata;
					8: {fpga_to_samd[29],fpga_to_samd[28]} <= avalon_readdata;
					9: {fpga_to_samd[31],fpga_to_samd[30]} <= avalon_readdata;
					10: {fpga_to_samd[33],fpga_to_samd[32]} <= avalon_readdata;
					11: {fpga_to_samd[35],fpga_to_samd[34]} <= avalon_readdata;
					// displacement
					12: {fpga_to_samd[37],fpga_to_samd[36]} <= avalon_readdata;
					13: {fpga_to_samd[39],fpga_to_samd[38]} <= avalon_readdata;
					14: {fpga_to_samd[41],fpga_to_samd[40]} <= avalon_readdata;
					15: {fpga_to_samd[43],fpga_to_samd[42]} <= avalon_readdata;
					16: {fpga_to_samd[45],fpga_to_samd[44]} <= avalon_readdata;
					17: {fpga_to_samd[47],fpga_to_samd[46]} <= avalon_readdata;
					// current
					18: {fpga_to_samd[49],fpga_to_samd[48]} <= avalon_readdata;
					19: {fpga_to_samd[51],fpga_to_samd[50]} <= avalon_readdata;
					20: {fpga_to_samd[53],fpga_to_samd[52]} <= avalon_readdata;
					21: {fpga_to_samd[55],fpga_to_samd[54]} <= avalon_readdata;
					22: {fpga_to_samd[57],fpga_to_samd[56]} <= avalon_readdata;
					23: {fpga_to_samd[59],fpga_to_samd[58]} <= avalon_readdata;
					// pwmRef
					24: {fpga_to_samd[61],fpga_to_samd[60]} <= avalon_readdata;
					25: {fpga_to_samd[63],fpga_to_samd[62]} <= avalon_readdata;
					26: {fpga_to_samd[65],fpga_to_samd[64]} <= avalon_readdata;
					27: {fpga_to_samd[67],fpga_to_samd[66]} <= avalon_readdata;
					28: {fpga_to_samd[69],fpga_to_samd[68]} <= avalon_readdata;
					29: {fpga_to_samd[71],fpga_to_samd[70]} <= avalon_readdata;
				endcase
				read_val <= read_val + 1;
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
  .c4(wFLASH_CLK),
   
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
