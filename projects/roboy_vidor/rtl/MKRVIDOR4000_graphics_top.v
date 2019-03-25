module MKRVIDOR4000_graphics_top
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

  // S-SPI Flash interface
//  output        oFLASH_MOSI,
//  input         iFLASH_MISO,
//  output        oFLASH_SCK,
//  output        oFLASH_CS,
//  output        oFLASH_WP,
//  output        oFLASH_HOLD

  // Q-SPI Flash interface
  output        oFLASH_SCK,
  output        oFLASH_CS,
  inout         oFLASH_MOSI,
  inout         iFLASH_MISO,
  inout         oFLASH_HOLD,
  inout         oFLASH_WP

);

`define FREE_VERSION

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

// DVI output
DVI_OUT
(
  .iPCLK(wVID_CLK),
  .iSCLK(wVID_CLKx5),

  .iRED(wDVI_RED),
  .iGRN(wDVI_GRN),
  .iBLU(wDVI_BLU),
  .iHS (wDVI_HS),
  .iVS (wDVI_VS),
  .iDE (wDVI_DE),

  .oDVI_DATA(oHDMI_TX),
  .oDVI_CLK(oHDMI_CLK),
  .iDVI_HPD(iHDMI_HPD)
);


wire fb_st_start,fb_st_dv,fb_st_ready;
wire [30:0] fb_st_data;

wire [23:0] mipi_st_data;
wire mipi_st_start,mipi_st_dv;

wire [23:0] mipi_ste_data;
wire mipi_ste_start,mipi_ste_dv;

wire [31:0] wSAM_PIO_IN;
wire [31:0] wSAM_PIO_OUT;
wire [31:0] wSAM_PIO_DIR;
wire [63:0] wSAM_PIO_MSEL;
wire [31:0] wWM_PIO_IN;
wire [31:0] wWM_PIO_OUT;
wire [31:0] wWM_PIO_DIR;
wire [63:0] wWM_PIO_MSEL;
wire [31:0] wPEX_PIO_IN;
wire [31:0] wPEX_PIO_OUT;
wire [31:0] wPEX_PIO_DIR;
wire [63:0] wPEX_PIO_MSEL;

wire wMIPI_SCL_O, wMIPI_SCL_EN;
wire wMIPI_SDA_O, wMIPI_SDA_EN;
wire wHDMI_SCL_O, wHDMI_SCL_EN;
wire wHDMI_SDA_O, wHDMI_SDA_EN;

reg [5:0] rRESETCNT;

always @(posedge wMEM_CLK)
begin
  if (!rRESETCNT[5])
  begin
  rRESETCNT<=rRESETCNT+1;
  end
end

assign bMIPI_SDA = !wMIPI_SDA_EN ? wMIPI_SDA_O : 1'bz;
assign bMIPI_SCL = !wMIPI_SCL_EN ? wMIPI_SCL_O : 1'bz;

`ifndef FREE_VERSION
MKRVIDOR4000_graphics_sys u0(
`else
MKRVIDOR4000_graphics_lite_sys u0(
`endif
		.clk_clk                (wMEM_CLK),               //      clk.clk
		.reset_reset_n          (rRESETCNT[5]), // reset.reset_n
		.vid_clk                (wVID_CLK),        //   vid.clk
		.fb_vport_red           (wDVI_RED),     //      .red
		.fb_vport_grn           (wDVI_GRN),     //      .grn
 		.fb_vport_blu           (wDVI_BLU),     // vport.blu
		.fb_vport_de            (wDVI_DE),      //      .de
		.fb_vport_hs            (wDVI_HS),      //      .hs
		.fb_vport_vs            (wDVI_VS),      //      .vs
		.mipi_rx_clk            (iMIPI_CLK),
		.mipi_rx_data           (iMIPI_D),

		.arb_fb_clk             (wVID_CLK),       //    arb_fb.clk
		.arb_fb_rdy             (fb_st_ready),       //          .rdy
		.arb_fb_data            (fb_st_data),      //          .data
		.arb_fb_dv              (fb_st_dv),        //          .dv
		.arb_fb_start           (fb_st_start),     //          .start

		.fb_st_start            (fb_st_start),      //     fb_st.start
		.fb_st_data             (fb_st_data),       //          .data
		.fb_st_dv               (fb_st_dv),         //          .dv
		.fb_st_ready            (fb_st_ready),       //          .ready

		.mipi_st_data           (mipi_st_data),     //   mipi_st.data
		.mipi_st_start          (mipi_st_start),    //          .start
		.mipi_st_dv             (mipi_st_dv),       //          .dv
		.arb_mipi_clk           (iMIPI_CLK),     //  arb_mipi.clk
		.arb_mipi_data          ({mipi_ste_data[23-:5],mipi_ste_data[15-:5],mipi_ste_data[7-:5]}),    //          .data
		.arb_mipi_start         (mipi_ste_start),    //          .data
		.arb_mipi_dv            (mipi_ste_dv)      //          .dv
	);

// MIPI input
assign bMIPI_GP[0]=1'b1;
assign bMIPI_GP[1]=1'bz;
endmodule
