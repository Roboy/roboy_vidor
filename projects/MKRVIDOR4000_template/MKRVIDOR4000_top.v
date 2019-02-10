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

parameter NUMBER_OF_MOTORS = 4;

reg [15:0] avalon_address;
reg        avalon_write;
reg [31:0] avalon_writedata;
reg        avalon_read;
reg [31:0] avalon_readdata;
wire 		  avalon_waitrequest;

forearm_control (
		.clk_clk(iCLK),                                       //                      clk.clk
		.myocontrol_0_conduit_end_miso(bMKR_D[1]),                 // myocontrol_0_conduit_end.miso
		.myocontrol_0_conduit_end_mosi(bMKR_D[0]),                 //                         .mosi
		.myocontrol_0_conduit_end_sck(bMKR_D[2]),                  //                         .sck
		.myocontrol_0_conduit_end_ss_n(bMKR_D[5:3]),                 //                         .ss_n
		.myocontrol_0_conduit_end_power_sense_n(1'b0),        //                         .power_sense_n
		.reset_reset_n(1'b1),      
		.myocontrol_0_avalon_slave_0_address(avalon_address),           // myocontrol_0_avalon_slave_0.address
		.myocontrol_0_avalon_slave_0_write(avalon_write),             //                            .write
		.myocontrol_0_avalon_slave_0_writedata(avalon_writedata),         //                            .writedata
		.myocontrol_0_avalon_slave_0_read(avalon_read),              //                            .read
		.myocontrol_0_avalon_slave_0_readdata(avalon_readdata),          //                            .readdata
		.myocontrol_0_avalon_slave_0_waitrequest(avalon_waitrequest)  
);

spi_slave #(8,1'b0,1'b0,3) (
	.clk_i(iCLK),
	.spi_sck_i(bMKR_D[9]),
   .spi_ssel_i(bMKR_D[7]),
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
assign data_in = samd_to_fpga[byte_counter];
reg [7:0] samd_to_fpga[86:0];
reg [7:0] fpga_to_samd[86:0];
wire data_out_valid;
reg data_out_valid_prev;

reg [31:0] byte_counter;

localparam IDLE = 0,  WRITE = 1, READ = 2, WAITREQUEST_WRITE = 3, WAITREQUEST_READ = 4, WAIT_FOR_FRAME_TRANSMISSION = 5;
reg [7:0] state = IDLE;
reg [7:0] read_val;
reg [7:0] write_val;

always @(posedge iCLK) begin: SPICONTROL_SPILOGIC
	write <= 0;
	data_out_valid_prev <= data_out_valid;
	case(state) 
		IDLE: begin
//			write <= 0;
//			avalon_write <= 0;
//			avalon_read <= 0;
//			if(data_out_valid) begin
//				state <= WRITE;
//				write_val <= 0;
//				read_val <= 0;
//			end
			byte_counter <= 0;
			if(bMKR_D[6]==0) begin
				state=WAIT_FOR_FRAME_TRANSMISSION;
				write <= 1;
			end
		end
		WAIT_FOR_FRAME_TRANSMISSION: begin
			if(bMKR_D[6]==0) begin // receiving 
				if(data_out_valid_prev==0 && data_out_valid==1) begin
					samd_to_fpga[byte_counter] = data_out;
					byte_counter <= byte_counter+1;
					write <= 1;
				end
			end else begin // receiving aborted
				state= IDLE;
			end
		end
	endcase
//		WRITE: begin
//			// lets set the address for the respective values
//			case(write_val)
//				// Kp
//				0: avalon_address <= {8'h00, 8'h00};
//				1: avalon_address <= {8'h00, 8'h01};
//				2: avalon_address <= {8'h00, 8'h02};
//				3: avalon_address <= {8'h00, 8'h03};
//				// Ki
//				4: avalon_address <= {8'h01, 8'h00};
//				5: avalon_address <= {8'h01, 8'h01};
//				6: avalon_address <= {8'h01, 8'h02};
//				7: avalon_address <= {8'h01, 8'h03};
//				// Kd
//				8: avalon_address <= {8'h02, 8'h00};
//				9: avalon_address <= {8'h02, 8'h01};
//				10: avalon_address <= {8'h02, 8'h02};
//				11: avalon_address <= {8'h02, 8'h03};
//				// sp
//				12: avalon_address <= {8'h03, 8'h00};
//				13: avalon_address <= {8'h03, 8'h01};
//				14: avalon_address <= {8'h03, 8'h02};
//				15: avalon_address <= {8'h03, 8'h03};
//				// outputPosMax
//				16: avalon_address <= {8'h05, 8'h00};
//				17: avalon_address <= {8'h05, 8'h01};
//				18: avalon_address <= {8'h05, 8'h02};
//				19: avalon_address <= {8'h05, 8'h03};
//				// outputNegMax
//				20: avalon_address <= {8'h06, 8'h00};
//				21: avalon_address <= {8'h06, 8'h01};
//				22: avalon_address <= {8'h06, 8'h02};
//				23: avalon_address <= {8'h06, 8'h03};
//				// IntegralPosMax
//				24: avalon_address <= {8'h07, 8'h00};
//				25: avalon_address <= {8'h07, 8'h01};
//				26: avalon_address <= {8'h07, 8'h02};
//				27: avalon_address <= {8'h07, 8'h03};
//				// IntegralNegMax
//				28: avalon_address <= {8'h08, 8'h00};
//				29: avalon_address <= {8'h08, 8'h01};
//				30: avalon_address <= {8'h08, 8'h02};
//				31: avalon_address <= {8'h08, 8'h03};
//				// deadband
//				32: avalon_address <= {8'h09, 8'h00};
//				33: avalon_address <= {8'h09, 8'h01};
//				34: avalon_address <= {8'h09, 8'h02};
//				35: avalon_address <= {8'h09, 8'h03};
//				// reset_myo_control
//				36: avalon_address <= {8'h0B, 8'h03};
//				// spi_activate
//				37: avalon_address <= {8'h0C, 8'h03};
//				// reset_controller
//				38: avalon_address <= {8'h0D, 8'h03};
//				39: avalon_address <= {8'h0D, 8'h03};
//				40: avalon_address <= {8'h0D, 8'h03};
//				41: avalon_address <= {8'h0D, 8'h03};
//				// control_mode0
//				42: avalon_address <= {8'h0A, 8'h03};
//				43: avalon_address <= {8'h0A, 8'h03};
//				44: avalon_address <= {8'h0A, 8'h03};
//				45: avalon_address <= {8'h0A, 8'h03};
//				// outputDivider
//				46: avalon_address <= {8'h14, 8'h00};
//				47: avalon_address <= {8'h14, 8'h01};
//				48: avalon_address <= {8'h14, 8'h02};
//				49: avalon_address <= {8'h14, 8'h03};
//			endcase 
//			
//			case(write_val)
//				// Kp
//				0: avalon_writedata <= samd_to_fpga[15:0];
//				1: avalon_writedata <= samd_to_fpga[31:16];
//				2: avalon_writedata <= samd_to_fpga[47:32];
//				3: avalon_writedata <= samd_to_fpga[63:48];
//				// Ki
//				4: avalon_writedata <= samd_to_fpga[79:64];
//				5: avalon_writedata <= samd_to_fpga[95:80];
//				6: avalon_writedata <= samd_to_fpga[111:96];
//				7: avalon_writedata <= samd_to_fpga[127:112];
//				// Kd
//				8: avalon_writedata <= samd_to_fpga[143:128];
//				9: avalon_writedata <= samd_to_fpga[159:144];
//				10: avalon_writedata <= samd_to_fpga[175:160];
//				11: avalon_writedata <= samd_to_fpga[191:176];
//				// sp
//				12: avalon_writedata <= samd_to_fpga[223:192];
//				13: avalon_writedata <= samd_to_fpga[255:224];
//				14: avalon_writedata <= samd_to_fpga[287:256];
//				15: avalon_writedata <= samd_to_fpga[319:288];
//				// outputPosMax
//				16: avalon_writedata <= samd_to_fpga[335:320];
//				17: avalon_writedata <= samd_to_fpga[351:336];
//				18: avalon_writedata <= samd_to_fpga[367:352];
//				19: avalon_writedata <= samd_to_fpga[383:368];
//				// outputNegMax
//				20: avalon_writedata <= samd_to_fpga[399:384];
//				21: avalon_writedata <= samd_to_fpga[415:400];
//				22: avalon_writedata <= samd_to_fpga[431:416];
//				23: avalon_writedata <= samd_to_fpga[447:432];
//				// IntegralPosMax
//				24: avalon_writedata <= samd_to_fpga[463:448];
//				25: avalon_writedata <= samd_to_fpga[479:464];
//				26: avalon_writedata <= samd_to_fpga[495:480];
//				27: avalon_writedata <= samd_to_fpga[511:496];
//				// IntegralNegMax
//				28: avalon_writedata <= samd_to_fpga[527:512];
//				29: avalon_writedata <= samd_to_fpga[543:528];
//				30: avalon_writedata <= samd_to_fpga[559:544];
//				31: avalon_writedata <= samd_to_fpga[575:560];
//				// deadband
//				32: avalon_writedata <= samd_to_fpga[591:576];
//				33: avalon_writedata <= samd_to_fpga[607:592];
//				34: avalon_writedata <= samd_to_fpga[623:608];
//				35: avalon_writedata <= samd_to_fpga[639:624];
//				// reset_myo_control
//				36: avalon_writedata <= samd_to_fpga[647];
//				// spi_activate
//				37: avalon_writedata <= samd_to_fpga[646];
//				// reset_controller
//				38: avalon_writedata <= samd_to_fpga[645];
//				39: avalon_writedata <= samd_to_fpga[644];
//				40: avalon_writedata <= samd_to_fpga[643];
//				41: avalon_writedata <= samd_to_fpga[642];
//				// control_mode
//				42: avalon_writedata <= samd_to_fpga[655];
//				43: avalon_writedata <= samd_to_fpga[654];
//				44: avalon_writedata <= samd_to_fpga[653];
//				45: avalon_writedata <= samd_to_fpga[652];
//				// outputDivider
//				46: avalon_writedata <= samd_to_fpga[663:656];
//				47: avalon_writedata <= samd_to_fpga[671:664];
//				48: avalon_writedata <= samd_to_fpga[687:672];
//				49: avalon_writedata <= samd_to_fpga[695:688];
//			endcase 
//			write_val <= write_val + 1;
//			avalon_write <= 1;
//			state <= WAITREQUEST_WRITE;
//		end
//		WAITREQUEST_WRITE: begin
//				if(avalon_waitrequest==0) begin // myoControl could be requesting to wait, and we wait...
//					avalon_write <= 0;
//					if(write_val <= 49) begin
//						state <= WRITE;
//					end else begin
//						state <= READ; // if we are done we move to read state
//						fpga_to_samd <= 0; // clear all previous data
//					end
//				end
//		end
//		READ: begin
//			if(read_val<=19) begin
//				// lets set the address for the respective values
//				case(read_val)
//					// position
//					0: avalon_address <= {8'h0B, 8'h00};
//					1: avalon_address <= {8'h0B, 8'h01};
//					2: avalon_address <= {8'h0B, 8'h02};
//					3: avalon_address <= {8'h0B, 8'h03};
//					// velocity
//					4: avalon_address <= {8'h0C, 8'h00};
//					5: avalon_address <= {8'h0C, 8'h01};
//					6: avalon_address <= {8'h0C, 8'h02};
//					7: avalon_address <= {8'h0C, 8'h03};
//					// displacement
//					8: avalon_address <= {8'h0E, 8'h00};
//					9: avalon_address <= {8'h0E, 8'h01};
//					10: avalon_address <= {8'h0E, 8'h02};
//					11: avalon_address <= {8'h0E, 8'h03};
//					// current
//					12: avalon_address <= {8'h0D, 8'h00};
//					13: avalon_address <= {8'h0D, 8'h01};
//					14: avalon_address <= {8'h0D, 8'h02};
//					15: avalon_address <= {8'h0D, 8'h03};
//					// pwmRef
//					16: avalon_address <= {8'h0F, 8'h00};
//					17: avalon_address <= {8'h0F, 8'h01};
//					18: avalon_address <= {8'h0F, 8'h02};
//					19: avalon_address <= {8'h0F, 8'h03};
//				endcase
//				avalon_read <= 1;
//				state <= WAITREQUEST_READ;
//			end else begin
//				write <= 1; // latch data into spi module
//				state <= IDLE; // go back to IDLE
//			end
//		end
//		WAITREQUEST_READ: begin
//			if(avalon_waitrequest==0) begin // myoControl could be requesting to wait, and we wait...
//				avalon_read <= 0;
//				state <= READ;
//				case(read_val)
//					// position
//					0: fpga_to_samd[383:352] <= avalon_readdata;
//					1: fpga_to_samd[351:320] <= avalon_readdata;
//					2: fpga_to_samd[319:288] <= avalon_readdata;
//					3: fpga_to_samd[287:256] <= avalon_readdata;
//					// velocity
//					4: fpga_to_samd[255:240] <= avalon_readdata;
//					5: fpga_to_samd[239:224] <= avalon_readdata;
//					6: fpga_to_samd[223:208] <= avalon_readdata;
//					7: fpga_to_samd[207:192] <= avalon_readdata;
//					// displacement
//					8: fpga_to_samd[191:176] <= avalon_readdata;
//					9: fpga_to_samd[175:160] <= avalon_readdata;
//					10: fpga_to_samd[159:144] <= avalon_readdata;
//					11: fpga_to_samd[143:128] <= avalon_readdata;
//					// current
//					12: fpga_to_samd[127:112] <= avalon_readdata;
//					13: fpga_to_samd[111:96] <= avalon_readdata;
//					14: fpga_to_samd[95:80] <= avalon_readdata;
//					15: fpga_to_samd[79:64] <= avalon_readdata;
//					// pwmRef
//					16: fpga_to_samd[63:48] <= avalon_readdata;
//					17: fpga_to_samd[47:32] <= avalon_readdata;
//					18: fpga_to_samd[31:16] <= avalon_readdata;
//					19: fpga_to_samd[15:0] <= avalon_readdata;
//				endcase
//				read_val <= read_val + 1;
//			end
//		end
//	endcase
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
