
module MKRVIDOR4000_mipi (
	clk_clk,
	fb_st_start,
	fb_st_data,
	fb_st_dv,
	fb_st_ready,
	fb_vport_blu,
	fb_vport_de,
	fb_vport_grn,
	fb_vport_hs,
	fb_vport_vs,
	fb_vport_red,
	mipi_rx_clk,
	mipi_rx_data,
	mipi_st_data,
	mipi_st_start,
	mipi_st_dv,
	reset_reset_n,
	vid_clk,
	arb_mipi_clk,
	arb_mipi_data,
	arb_mipi_dv,
	arb_mipi_start,
	arb_fb_clk,
	arb_fb_rdy,
	arb_fb_data,
	arb_fb_dv,
	arb_fb_start,
	sdram_addr,
	sdram_ba,
	sdram_cas_n,
	sdram_cke,
	sdram_cs_n,
	sdram_dq,
	sdram_dqm,
	sdram_ras_n,
	sdram_we_n);	

	input		clk_clk;
	input		fb_st_start;
	input	[30:0]	fb_st_data;
	input		fb_st_dv;
	output		fb_st_ready;
	output	[7:0]	fb_vport_blu;
	output		fb_vport_de;
	output	[7:0]	fb_vport_grn;
	output		fb_vport_hs;
	output		fb_vport_vs;
	output	[7:0]	fb_vport_red;
	input		mipi_rx_clk;
	input	[1:0]	mipi_rx_data;
	output	[23:0]	mipi_st_data;
	output		mipi_st_start;
	output		mipi_st_dv;
	input		reset_reset_n;
	input		vid_clk;
	input		arb_mipi_clk;
	input	[14:0]	arb_mipi_data;
	input		arb_mipi_dv;
	input		arb_mipi_start;
	input		arb_fb_clk;
	input		arb_fb_rdy;
	output	[30:0]	arb_fb_data;
	output		arb_fb_dv;
	output		arb_fb_start;
	output	[11:0]	sdram_addr;
	output	[1:0]	sdram_ba;
	output		sdram_cas_n;
	output		sdram_cke;
	output		sdram_cs_n;
	inout	[15:0]	sdram_dq;
	output	[1:0]	sdram_dqm;
	output		sdram_ras_n;
	output		sdram_we_n;
endmodule
