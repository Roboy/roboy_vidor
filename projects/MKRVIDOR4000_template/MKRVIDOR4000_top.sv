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

MyoControlInterface #(.NUMBER_OF_MOTORS(NUMBER_OF_MOTORS)) myo_control_interface();

assign bMKR_D[13] = data_out_valid;
MYOControl #(NUMBER_OF_MOTORS,24_000_000,0) myocontrol(  
	.clock(wCLK24),
	.miso(bMKR_D[1]),                 // myocontrol_0_conduit_end.miso
	.mosi(bMKR_D[0]),                 //                         .mosi
	.sck(bMKR_D[2]),                  //                         .sck
	.ss_n_o({bMKR_A[2],bMKR_D[7:3]}),                 //                         .ss_n
	.power_sense_n(1'b0),        //                         .power_sense_n
	.reset(1'b0),
	.address(avalon_address),           // myocontrol_0_avalon_slave_0.address
	.write(avalon_write),             //                            .write
	.writedata(avalon_writedata),         //                            .writedata
	.read(avalon_read),              //                            .read
	.readdata(avalon_readdata),          //                            .readdata
	.waitrequest(avalon_waitrequest),
	.interf(myo_control_interface.child)
)/* synthesis preserve */; 

spi_slave #(8,1'b0,1'b0,3) spi_slave0(
	.clk_i(wCLK24),
	.spi_sck_i(bMKR_D[9]),
   .spi_ssel_i(bMKR_A[1]),
   .spi_mosi_i(bMKR_D[8]),
   .spi_miso_o(bMKR_D[10]),
   .di_i(data_in),
   .wren_i(write),
   .do_valid_o(data_out_valid),
   .do_o(data_out)
)/* synthesis preserve */;

reg write;
wire [7:0] data_out;
wire [7:0] data_in;
assign data_in = fpga_to_samd[byte_counter];
reg [7:0] samd_to_fpga[113:0];
reg [7:0] fpga_to_samd[113:0];
wire data_out_valid;
reg data_out_valid_prev;

reg [31:0] byte_counter;

struct packed {     
	logic [7:0] motor;
	logic [31:0] Kp_f;
	logic [31:0] Ki_f;
	logic [31:0] Kd_f;
	logic [31:0] sp_f;
	logic [31:0] outputLimit;
	logic [7:0] control_mode;
	logic [15:0] controlFlags;
	logic [31:0] update_frequency;
	logic [31:0] pos_encoder_multiplier_f;
	logic [31:0] dis_encoder_multiplier_f;
	logic [15:0] pwmRef;
	logic [31:0] position;
	logic [15:0] velocity;
	logic [15:0] current;
	logic [31:0] displacement;
	logic [31:0] position_raw_f;
	logic [31:0] velocity_raw_f;
	logic [31:0] displacement_raw_f;
	logic [31:0] position_conv_f;
	logic [31:0] velocity_conv_f;
	logic [31:0] displacemen_conv_f;
	logic [31:0] displacement_myo_brick_conv_f;
	logic [31:0] position_err;
	logic [31:0] velocity_err;
	logic [31:0] displacement_err;
	logic [31:0] displacement_myo_brick_err;
	logic [31:0] position_res;
	logic [31:0] velocity_res;
	logic [31:0] displacement_res;
	logic [31:0] displacement_myo_brick_res;
	logic [31:0] actual_update_frequency;
} spi_values;

assign spi_values.motor = samd_to_fpga[0];
assign spi_values.Kp_f = {samd_to_fpga[4],samd_to_fpga[3],samd_to_fpga[2],samd_to_fpga[1]};
assign spi_values.Ki_f = {samd_to_fpga[8],samd_to_fpga[7],samd_to_fpga[6],samd_to_fpga[5]};
assign spi_values.Kd_f = {samd_to_fpga[12],samd_to_fpga[11],samd_to_fpga[10],samd_to_fpga[9]};
assign spi_values.sp_f = {samd_to_fpga[16],samd_to_fpga[15],samd_to_fpga[14],samd_to_fpga[13]};
assign spi_values.outputLimit = {samd_to_fpga[20],samd_to_fpga[19],samd_to_fpga[18],samd_to_fpga[17]};
assign spi_values.control_mode = samd_to_fpga[21];
assign spi_values.controlFlags = {samd_to_fpga[23],samd_to_fpga[22]};
assign spi_values.update_frequency = {samd_to_fpga[27],samd_to_fpga[26],samd_to_fpga[25],samd_to_fpga[24]};
assign spi_values.pos_encoder_multiplier_f = {samd_to_fpga[31],samd_to_fpga[30],samd_to_fpga[29],samd_to_fpga[28]};
assign spi_values.dis_encoder_multiplier_f = {samd_to_fpga[35],samd_to_fpga[34],samd_to_fpga[33],samd_to_fpga[32]};
assign spi_values.pwmRef = 16'hBEEF;//{fpga_to_samd[37],fpga_to_samd[36]};
assign spi_values.position = {fpga_to_samd[41],fpga_to_samd[40],fpga_to_samd[39],fpga_to_samd[38]};
assign spi_values.velocity = {fpga_to_samd[43],fpga_to_samd[42]};
assign spi_values.current = {fpga_to_samd[45],fpga_to_samd[44]};
assign spi_values.displacement = {fpga_to_samd[49],fpga_to_samd[48],fpga_to_samd[47],fpga_to_samd[46]};
assign spi_values.position_raw_f = {fpga_to_samd[53],fpga_to_samd[52],fpga_to_samd[51],fpga_to_samd[50]};
assign spi_values.velocity_raw_f = {fpga_to_samd[57],fpga_to_samd[56],fpga_to_samd[55],fpga_to_samd[54]};
assign spi_values.displacement_raw_f = {fpga_to_samd[61],fpga_to_samd[60],fpga_to_samd[59],fpga_to_samd[58]};
assign spi_values.position_conv_f = {fpga_to_samd[65],fpga_to_samd[64],fpga_to_samd[63],fpga_to_samd[62]};
assign spi_values.velocity_conv_f = {fpga_to_samd[69],fpga_to_samd[68],fpga_to_samd[67],fpga_to_samd[66]};
assign spi_values.displacemen_conv_f = {fpga_to_samd[73],fpga_to_samd[72],fpga_to_samd[71],fpga_to_samd[70]};
assign spi_values.displacement_myo_brick_conv_f = {fpga_to_samd[77],fpga_to_samd[76],fpga_to_samd[75],fpga_to_samd[74]};
assign spi_values.position_err = {fpga_to_samd[81],fpga_to_samd[80],fpga_to_samd[79],fpga_to_samd[78]};
assign spi_values.velocity_err = {fpga_to_samd[85],fpga_to_samd[84],fpga_to_samd[83],fpga_to_samd[82]};
assign spi_values.displacement_err = {fpga_to_samd[89],fpga_to_samd[88],fpga_to_samd[87],fpga_to_samd[86]};
assign spi_values.displacement_myo_brick_err = {fpga_to_samd[93],fpga_to_samd[92],fpga_to_samd[91],fpga_to_samd[90]};
assign spi_values.position_res = {fpga_to_samd[97],fpga_to_samd[96],fpga_to_samd[95],fpga_to_samd[94]};
assign spi_values.velocity_res = {fpga_to_samd[101],fpga_to_samd[100],fpga_to_samd[99],fpga_to_samd[98]};
assign spi_values.displacement_res = {fpga_to_samd[105],fpga_to_samd[104],fpga_to_samd[103],fpga_to_samd[102]};
assign spi_values.displacement_myo_brick_res = {fpga_to_samd[109],fpga_to_samd[108],fpga_to_samd[107],fpga_to_samd[106]};
assign spi_values.actual_update_frequency = {fpga_to_samd[113],fpga_to_samd[112],fpga_to_samd[111],fpga_to_samd[110]};

localparam IDLE = 0, WAIT_FOR_FRAME_TRANSMISSION = 1;
reg [7:0] state = IDLE;
integer j;

always @(posedge wCLK24) begin: SPICONTROL_SPILOGIC
	data_out_valid_prev <= data_out_valid;
	case(state) 
		IDLE: begin
			byte_counter <= 0;
			if(bMKR_A[0]==0) begin
				state=WAIT_FOR_FRAME_TRANSMISSION;
				for(j=0;j<114;j=j+1)begin
					fpga_to_samd[j] <= j;
				end
			end
		end
		WAIT_FOR_FRAME_TRANSMISSION: begin
			if(bMKR_A[0]==0) begin // receiving 
				if(data_out_valid_prev==0 && data_out_valid==1) begin
					samd_to_fpga[byte_counter] = data_out;
					byte_counter <= byte_counter+1;
				end
			end else begin // receiving done
				for(j=0; j<NUMBER_OF_MOTORS;j++) begin
					if(j==samd_to_fpga[0])begin
//						myo_control_interface.child.Kp_f[j] <= spi_values.Kp_f;
//						myo_control_interface.child.Ki_f[j] <= spi_values.Ki_f;
//						myo_control_interface.child.Kd_f[j] <= spi_values.Kd_f;
//						myo_control_interface.child.sp_f[j] <= spi_values.sp_f;
//						myo_control_interface.child.outputLimit[j] <= spi_values.outputLimit;
//						myo_control_interface.child.control_mode[j] <= spi_values.control_mode;
//						myo_control_interface.child.controlFlags[j] <= spi_values.controlFlags;
//						myo_control_interface.child.update_frequency[j] <= spi_values.update_frequency;
//						myo_control_interface.child.pos_encoder_multiplier_f[j] <= spi_values.pos_encoder_multiplier_f;
//						myo_control_interface.child.dis_encoder_multiplier_f[j] <= spi_values.dis_encoder_multiplier_f;
//						spi_values.pwmRef <= myo_control_interface.child.pwmRefs[j];
//						spi_values.position <= myo_control_interface.child.positions[j];
//						spi_values.velocity <= myo_control_interface.child.velocities[j];
//						spi_values.current <= myo_control_interface.child.currents[j];
//						spi_values.displacement <= myo_control_interface.child.displacements[j];
//						spi_values.position_raw_f <= myo_control_interface.child.positions_raw_f[j];
//						spi_values.velocity_raw_f <= myo_control_interface.child.velocities_raw_f[j];
//						spi_values.displacement_raw_f <= myo_control_interface.child.displacements_raw_f[j];
//						spi_values.position_conv_f <= myo_control_interface.child.positions_conv_f[j];
//						spi_values.velocity_conv_f <= myo_control_interface.child.velocities_conv_f[j];
//						spi_values.displacemen_conv_f <= myo_control_interface.child.displacements_conv_f[j];
//						spi_values.displacement_myo_brick_conv_f <= myo_control_interface.child.displacements_myo_brick_conv_f[j];
//						spi_values.position_err <= myo_control_interface.child.positions_err_f[j];
//						spi_values.velocity_err <= myo_control_interface.child.velocities_err_f[j];
//						spi_values.displacement_err <= myo_control_interface.child.displacements_err_f[j];
//						spi_values.displacement_myo_brick_err <= myo_control_interface.child.displacements_myo_brick_err_f[j];
//						spi_values.position_res <= myo_control_interface.child.positions_res_f[j];
//						spi_values.velocity_res <= myo_control_interface.child.velocities_res_f[j];
//						spi_values.displacement_res <= myo_control_interface.child.displacements_res_f[j];
//						spi_values.displacement_myo_brick_res <= myo_control_interface.child.displacements_myo_brick_res_f[j];
//						spi_values.actual_update_frequency <= myo_control_interface.child.actual_update_frequency[j];
					end
				end
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
