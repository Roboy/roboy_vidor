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
reg [9:0] pwm_delay;
reg signed [31:0] pwm;

always @(posedge wCLK24) begin: BLDC_COMMUTATION
	if( pwm>=0 && pwm_delay>(1023-pwm))begin
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
	end else if ( pwm<0 && pwm_delay>(1023+pwm)) begin
		if(bMKR_A[4] && ~bMKR_A[5] && bMKR_A[6]) begin
			PHASES <= 6'b011000;
		end 
		if(bMKR_A[4] && ~bMKR_A[5] && ~bMKR_A[6])begin
			PHASES <= 6'b010010;
		end 
		if(bMKR_A[4] && bMKR_A[5] && ~bMKR_A[6]) begin
			PHASES <= 6'b000110;
		end 
		if(~bMKR_A[4] && bMKR_A[5] && ~bMKR_A[6])begin
			PHASES <= 6'b100100; 
		end 
		if(~bMKR_A[4] && bMKR_A[5] && bMKR_A[6]) begin
			PHASES <= 6'b100001;
		end 	
		if(~bMKR_A[4] && ~bMKR_A[5] && bMKR_A[6])begin
			PHASES <= 6'b001001;
		end 
	end else begin
		PHASES <= 0;
	end
	pwm_delay <= pwm_delay+1;
end

wire signed [31:0] position;
reg signed [31:0] setpoint;
reg signed [31:0] position_prev;
reg signed [31:0] velocity;
reg signed [31:0] displacement;
reg signed [31:0] current;
reg [7:0] control_mode;
reg update_controller;

always @(posedge wCLK24) begin: PID_CONTROLLER_PID_CONTROLLERLOGIC
	reg signed [31:0] integral;
	reg signed [31:0] lastError;
	reg signed [31:0] err;
	reg signed [31:0] pterm;
	reg signed [31:0] dterm;
	reg signed [31:0] displacement_offset;
	reg update_controller_prev;
	reg signed [31:0] result;
	localparam integer outputNegMax = -1023;
	localparam integer outputPosMax = 1023;
	localparam integer Kp = 1;
	localparam integer Kd = 0;
	
	case(control_mode) 
		0: err <= (setpoint - position); 
		1: err <= (setpoint - velocity);
		2: err <= (setpoint - displacement);
		default: pwm <= setpoint;
	endcase;
	if(control_mode<3) begin
		pterm <= (Kp * err);
		dterm <= ((err - lastError) * Kd);
		result <= (pterm + dterm)>>>7;
		lastError <= err;
		// limit output
		if ((result < outputNegMax)) begin
			 result <= outputNegMax;
		end else if ((result > outputPosMax)) begin
			 result <= outputPosMax;
		end
		pwm <= result;
	end
end

rot_enc_flt encoder_0(wCLK120, 0, 0, bMKR_A[1], bMKR_A[0], position);

wire [7:0] data_out;
wire [7:0] data_in;
assign data_in = fpga_to_samd[byte_counter];
reg [7:0] samd_to_fpga[24:0];
reg [7:0] fpga_to_samd[24:0];
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
reg signed [31:0] counter;
integer j;

always @(posedge wCLK24) begin: SPICONTROL_SPILOGIC
	data_out_valid_prev <= data_out_valid;
	counter <= counter+1;
	write <= 0;
	update_controller<=0;
//	start_spi_transmission<=0;
	case(state) 
		IDLE: begin
			byte_counter <= 0;
			if(bMKR_D[6]==0) begin
				state=WAIT_FOR_FRAME_TRANSMISSION;
				position_prev <= position;
				if(counter!=0) begin
					velocity <= (position-position_prev)/counter;
				end
				counter <= 0;
				write <= 1;
				update_controller<=1;
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
//					start_spi_transmission<= 1;
				end
			end else begin // receiving done
				setpoint <= {samd_to_fpga[3],samd_to_fpga[2],samd_to_fpga[1],samd_to_fpga[0]};
				fpga_to_samd[0] <= setpoint[7:0];
				fpga_to_samd[1] <= setpoint[15:8];
				fpga_to_samd[2] <= setpoint[23:16];
				fpga_to_samd[3] <= setpoint[31:24];
				fpga_to_samd[4] <= position[7:0];
				fpga_to_samd[5] <= position[15:8];
				fpga_to_samd[6] <= position[23:16];
				fpga_to_samd[7] <= position[31:24];
				fpga_to_samd[8] <= velocity[7:0];
				fpga_to_samd[9] <= velocity[15:8];
				fpga_to_samd[10] <= velocity[23:16];
				fpga_to_samd[11] <= velocity[31:24];
				fpga_to_samd[12] <= displacement[7:0];
				fpga_to_samd[13] <= displacement[15:8];
				fpga_to_samd[14] <= displacement[23:16];
				fpga_to_samd[15] <= displacement[31:24];
				fpga_to_samd[16] <= current[7:0];
				fpga_to_samd[17] <= current[15:8];
				fpga_to_samd[18] <= current[23:16];
				fpga_to_samd[19] <= current[31:24];
				fpga_to_samd[20] <= pwm[7:0];
				fpga_to_samd[21] <= pwm[15:8];
				fpga_to_samd[22] <= pwm[23:16];
				fpga_to_samd[23] <= pwm[31:24];
				fpga_to_samd[24] <= control_mode;
				control_mode <= samd_to_fpga[24];
				state <= IDLE;
			end 
		end
	endcase 
end
 
//wire di_req, wr_ack, do_valid, wren, spi_done;
//wire [15:0] current_sensor_data_out;
//wire signed [15:0] current;
//reg start_spi_transmission;
//
//// control logic for handling myocontrol frame
//TLI4970SpiControl spi_control(
//	.clock(clock),
//	.reset(1'b0),
//	.di_req(di_req),
//	.write_ack(wr_ack),
//	.data_read_valid(do_valid),
//	.data_read(current_sensor_data_out[15:0]),
//	.start(start_spi_transmission),
//	.wren(wren),
//	.spi_done(spi_done),
//	.current(current),
//	.ss_n(bMKR_A[2])
//);
//
//// SPI specs: 2MHz, 16bit MSB, clock phase of 1
//spi_master #(16, 1'b0, 1'b1, 2, 6) spi(
//	.sclk_i(wCLK24),
//	.pclk_i(wCLK24),
//	.rst_i(1'b0),
//	.spi_miso_i(miso),
//	.di_i(16'hFFFF),
//	.wren_i(wren),
//	.spi_ssel_o(ss_n),
//	.spi_sck_o(sck),
//	.spi_mosi_o(mosi),
//	.di_req_o(di_req),
//	.wr_ack_o(wr_ack),
//	.do_valid_o(do_valid),
//	.do_o(current_sensor_data_out[15:0])
//);
 
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
