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

MyoControlInterface #(.NUMBER_OF_MOTORS(NUMBER_OF_MOTORS)) myo_control_interface();

assign bMKR_D[13] = wCLK24;
MYOControl #(NUMBER_OF_MOTORS,24_000_000,0) (
	.clock(wCLK24),
	.miso(bMKR_D[1]),                 // myocontrol_0_conduit_end.miso
	.mosi(bMKR_D[0]),                 //                         .mosi
	.sck(bMKR_D[2]),                  //                         .sck
	.ss_n_o({bMKR_A[2],bMKR_D[7:3]}),                 //                         .ss_n
	.power_sense_n(1'b0),        //                         .power_sense_n
	.reset(1'b0),      
	.interf(myo_control_interface.child)
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
reg [7:0] samd_to_fpga[113:0];
reg [7:0] fpga_to_samd[113:0];
wire data_out_valid;
reg data_out_valid_prev;

reg [31:0] byte_counter;

localparam IDLE = 0,  WRITE = 1, READ = 2, WAITREQUEST_WRITE = 3, WAITREQUEST_READ = 4, WAIT_FOR_FRAME_TRANSMISSION = 5;
reg [7:0] state = IDLE;
integer j;

typedef struct packed {   
	logic [113:0] data;
} data_t;

typedef struct packed {     
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
} spi_values_t;

typedef union packed{
	data_t data;
	spi_values_t values;
} spi_frame_t;

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
				for(j=0; j<NUMBER_OF_MOTORS;j++) begin
					if(j==samd_to_fpga[0])begin
						myo_control_interface.child.Kp_f[j] <= {samd_to_fpga[4],samd_to_fpga[3],samd_to_fpga[2],samd_to_fpga[1]};
						myo_control_interface.child.Ki_f[j] <= {samd_to_fpga[8],samd_to_fpga[7],samd_to_fpga[6],samd_to_fpga[5]};
						myo_control_interface.child.Kd_f[j] <= {samd_to_fpga[12],samd_to_fpga[11],samd_to_fpga[10],samd_to_fpga[9]};
						myo_control_interface.child.sp_f[j] <= {samd_to_fpga[16],samd_to_fpga[15],samd_to_fpga[14],samd_to_fpga[13]};
						myo_control_interface.child.outputLimit[j] <= {samd_to_fpga[20],samd_to_fpga[19],samd_to_fpga[18],samd_to_fpga[17]};
						myo_control_interface.child.control_mode[j] <= samd_to_fpga[21];
						myo_control_interface.child.controlFlags[j] <= {samd_to_fpga[22],samd_to_fpga[21]};
						myo_control_interface.child.update_frequency[j] <= {samd_to_fpga[26],samd_to_fpga[25],samd_to_fpga[24],samd_to_fpga[23]};
						myo_control_interface.child.pos_encoder_multiplier_f[j] <= {samd_to_fpga[30],samd_to_fpga[29],samd_to_fpga[28],samd_to_fpga[27]};
						myo_control_interface.child.dis_encoder_multiplier_f[j] <= {samd_to_fpga[34],samd_to_fpga[33],samd_to_fpga[32],samd_to_fpga[31]};
						{fpga_to_samd[36],fpga_to_samd[35]} <= myo_control_interface.child.pwmRefs[j];
						{fpga_to_samd[36],fpga_to_samd[35]} <= myo_control_interface.child.pwmRefs[j];
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
