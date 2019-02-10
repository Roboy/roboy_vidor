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
assign data_in = fpga_to_samd[byte_counter];
reg [7:0] samd_to_fpga[88:0];
reg [7:0] fpga_to_samd[88:0];
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
				// Ki
				4: avalon_address <= {8'h01, 8'h00};
				5: avalon_address <= {8'h01, 8'h01};
				6: avalon_address <= {8'h01, 8'h02};
				7: avalon_address <= {8'h01, 8'h03};
				// Kd
				8: avalon_address <= {8'h02, 8'h00};
				9: avalon_address <= {8'h02, 8'h01};
				10: avalon_address <= {8'h02, 8'h02};
				11: avalon_address <= {8'h02, 8'h03};
				// sp
				12: avalon_address <= {8'h03, 8'h00};
				13: avalon_address <= {8'h03, 8'h01};
				14: avalon_address <= {8'h03, 8'h02};
				15: avalon_address <= {8'h03, 8'h03};
				// outputPosMax
				16: avalon_address <= {8'h05, 8'h00};
				17: avalon_address <= {8'h05, 8'h01};
				18: avalon_address <= {8'h05, 8'h02};
				19: avalon_address <= {8'h05, 8'h03};
				// outputNegMax
				20: avalon_address <= {8'h06, 8'h00};
				21: avalon_address <= {8'h06, 8'h01};
				22: avalon_address <= {8'h06, 8'h02};
				23: avalon_address <= {8'h06, 8'h03};
				// IntegralPosMax
				24: avalon_address <= {8'h07, 8'h00};
				25: avalon_address <= {8'h07, 8'h01};
				26: avalon_address <= {8'h07, 8'h02};
				27: avalon_address <= {8'h07, 8'h03};
				// IntegralNegMax
				28: avalon_address <= {8'h08, 8'h00};
				29: avalon_address <= {8'h08, 8'h01};
				30: avalon_address <= {8'h08, 8'h02};
				31: avalon_address <= {8'h08, 8'h03};
				// deadband
				32: avalon_address <= {8'h09, 8'h00};
				33: avalon_address <= {8'h09, 8'h01};
				34: avalon_address <= {8'h09, 8'h02};
				35: avalon_address <= {8'h09, 8'h03};
				// reset_myo_control
				36: avalon_address <= {8'h0B, 8'h03};
				// spi_activate
				37: avalon_address <= {8'h0C, 8'h03};
				// reset_controller
				38: avalon_address <= {8'h0D, 8'h03};
				39: avalon_address <= {8'h0D, 8'h03};
				40: avalon_address <= {8'h0D, 8'h03};
				41: avalon_address <= {8'h0D, 8'h03};
				// control_mode0
				42: avalon_address <= {8'h0A, 8'h03};
				43: avalon_address <= {8'h0A, 8'h03};
				44: avalon_address <= {8'h0A, 8'h03};
				45: avalon_address <= {8'h0A, 8'h03};
				// outputDivider
				46: avalon_address <= {8'h14, 8'h00};
				47: avalon_address <= {8'h14, 8'h01};
				48: avalon_address <= {8'h14, 8'h02};
				49: avalon_address <= {8'h14, 8'h03};
			endcase 
			
			case(write_val)
				// Kp
				0: avalon_writedata <= {samd_to_fpga[1],samd_to_fpga[0]};
				1: avalon_writedata <= {samd_to_fpga[3],samd_to_fpga[2]};
				2: avalon_writedata <= {samd_to_fpga[5],samd_to_fpga[4]};
				3: avalon_writedata <= {samd_to_fpga[7],samd_to_fpga[6]};
				// Ki
				4: avalon_writedata <= {samd_to_fpga[9],samd_to_fpga[8]};
				5: avalon_writedata <= {samd_to_fpga[11],samd_to_fpga[10]};
				6: avalon_writedata <= {samd_to_fpga[13],samd_to_fpga[12]};
				7: avalon_writedata <= {samd_to_fpga[15],samd_to_fpga[14]};
				// Kd
				8: avalon_writedata <= {samd_to_fpga[17],samd_to_fpga[16]};
				9: avalon_writedata <= {samd_to_fpga[19],samd_to_fpga[18]};
				10: avalon_writedata <= {samd_to_fpga[21],samd_to_fpga[20]};
				11: avalon_writedata <= {samd_to_fpga[23],samd_to_fpga[22]};
				// sp
				12: avalon_writedata <= {samd_to_fpga[27],samd_to_fpga[26],samd_to_fpga[25],samd_to_fpga[24]};
				13: avalon_writedata <= {samd_to_fpga[31],samd_to_fpga[30],samd_to_fpga[29],samd_to_fpga[28]};
				14: avalon_writedata <= {samd_to_fpga[35],samd_to_fpga[34],samd_to_fpga[33],samd_to_fpga[32]};
				15: avalon_writedata <= {samd_to_fpga[39],samd_to_fpga[38],samd_to_fpga[37],samd_to_fpga[36]};
				// outputPosMax
				16: avalon_writedata <= {samd_to_fpga[41],samd_to_fpga[40]};
				17: avalon_writedata <= {samd_to_fpga[43],samd_to_fpga[42]};
				18: avalon_writedata <= {samd_to_fpga[45],samd_to_fpga[44]};
				19: avalon_writedata <= {samd_to_fpga[47],samd_to_fpga[46]};
				// outputNegMax
				20: avalon_writedata <= {samd_to_fpga[49],samd_to_fpga[48]};
				21: avalon_writedata <= {samd_to_fpga[51],samd_to_fpga[50]};
				22: avalon_writedata <= {samd_to_fpga[53],samd_to_fpga[52]};
				23: avalon_writedata <= {samd_to_fpga[55],samd_to_fpga[54]};
				// IntegralPosMax
				24: avalon_writedata <= {samd_to_fpga[57],samd_to_fpga[56]};
				25: avalon_writedata <= {samd_to_fpga[59],samd_to_fpga[58]};
				26: avalon_writedata <= {samd_to_fpga[61],samd_to_fpga[60]};
				27: avalon_writedata <= {samd_to_fpga[63],samd_to_fpga[62]};
				// IntegralNegMax
				28: avalon_writedata <= {samd_to_fpga[65],samd_to_fpga[64]};
				29: avalon_writedata <= {samd_to_fpga[67],samd_to_fpga[66]};
				30: avalon_writedata <= {samd_to_fpga[69],samd_to_fpga[68]};
				31: avalon_writedata <= {samd_to_fpga[71],samd_to_fpga[70]};
				// deadband
				32: avalon_writedata <= {samd_to_fpga[73],samd_to_fpga[72]};
				33: avalon_writedata <= {samd_to_fpga[75],samd_to_fpga[74]};
				34: avalon_writedata <= {samd_to_fpga[77],samd_to_fpga[76]};
				35: avalon_writedata <= {samd_to_fpga[79],samd_to_fpga[78]};
				// reset_myo_control
				36: avalon_writedata <= samd_to_fpga[80][7];
				// spi_activate
				37: avalon_writedata <= samd_to_fpga[80][6];
				// reset_controller
				38: avalon_writedata <= samd_to_fpga[80][5];
				39: avalon_writedata <= samd_to_fpga[80][4];
				40: avalon_writedata <= samd_to_fpga[80][3];
				41: avalon_writedata <= samd_to_fpga[80][2];
				// control_mode
				42: avalon_writedata <= samd_to_fpga[81];
				43: avalon_writedata <= samd_to_fpga[82];
				44: avalon_writedata <= samd_to_fpga[83];
				45: avalon_writedata <= samd_to_fpga[84];
				// outputDivider
				46: avalon_writedata <= samd_to_fpga[85];
				47: avalon_writedata <= samd_to_fpga[86];
				48: avalon_writedata <= samd_to_fpga[87];
				49: avalon_writedata <= samd_to_fpga[88];
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
					// velocity
					4: avalon_address <= {8'h0C, 8'h00};
					5: avalon_address <= {8'h0C, 8'h01};
					6: avalon_address <= {8'h0C, 8'h02};
					7: avalon_address <= {8'h0C, 8'h03};
					// displacement
					8: avalon_address <= {8'h0E, 8'h00};
					9: avalon_address <= {8'h0E, 8'h01};
					10: avalon_address <= {8'h0E, 8'h02};
					11: avalon_address <= {8'h0E, 8'h03};
					// current
					12: avalon_address <= {8'h0D, 8'h00};
					13: avalon_address <= {8'h0D, 8'h01};
					14: avalon_address <= {8'h0D, 8'h02};
					15: avalon_address <= {8'h0D, 8'h03};
					// pwmRef
					16: avalon_address <= {8'h0F, 8'h00};
					17: avalon_address <= {8'h0F, 8'h01};
					18: avalon_address <= {8'h0F, 8'h02};
					19: avalon_address <= {8'h0F, 8'h03};
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
					// velocity
					4: {fpga_to_samd[17],fpga_to_samd[16]} <= avalon_readdata;
					5: {fpga_to_samd[19],fpga_to_samd[18]} <= avalon_readdata;
					6: {fpga_to_samd[21],fpga_to_samd[20]} <= avalon_readdata;
					7: {fpga_to_samd[23],fpga_to_samd[22]} <= avalon_readdata;
					// displacement
					8: {fpga_to_samd[25],fpga_to_samd[24]} <= avalon_readdata;
					9: {fpga_to_samd[27],fpga_to_samd[26]} <= avalon_readdata;
					10: {fpga_to_samd[29],fpga_to_samd[28]} <= avalon_readdata;
					11: {fpga_to_samd[31],fpga_to_samd[30]} <= avalon_readdata;
					// current
					12: {fpga_to_samd[33],fpga_to_samd[32]} <= avalon_readdata;
					13: {fpga_to_samd[35],fpga_to_samd[34]} <= avalon_readdata;
					14: {fpga_to_samd[37],fpga_to_samd[36]} <= avalon_readdata;
					15: {fpga_to_samd[39],fpga_to_samd[38]} <= avalon_readdata;
					// pwmRef
					16: {fpga_to_samd[41],fpga_to_samd[40]} <= avalon_readdata;
					17: {fpga_to_samd[43],fpga_to_samd[42]} <= avalon_readdata;
					18: {fpga_to_samd[45],fpga_to_samd[44]} <= avalon_readdata;
					19: {fpga_to_samd[47],fpga_to_samd[46]} <= avalon_readdata;
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
