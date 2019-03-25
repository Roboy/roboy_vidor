
module MKRVIDOR4000_graphics_lite_sys (
	arb_fb_clk,
	arb_fb_rdy,
	arb_fb_data,
	arb_fb_dv,
	arb_fb_start,
	arb_mipi_clk,
	arb_mipi_data,
	arb_mipi_dv,
	arb_mipi_start,
	clk_clk,
	clk_0_clk,
	csi_i2c_scl_i,
	csi_i2c_scl_o,
	csi_i2c_scl_en,
	csi_i2c_sda_i,
	csi_i2c_sda_o,
	csi_i2c_sda_en,
	encoder_encoder_a,
	encoder_encoder_b,
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
	flash_spi_MOSI,
	flash_spi_SCLK,
	flash_spi_MISO,
	flash_spi_CS,
	hdmi_i2c_scl_i,
	hdmi_i2c_scl_o,
	hdmi_i2c_scl_en,
	hdmi_i2c_sda_i,
	hdmi_i2c_sda_o,
	hdmi_i2c_sda_en,
	irq_in_port,
	irq_out_port,
	mb_ak,
	mb_rq,
	mipi_rx_clk,
	mipi_rx_data,
	mipi_st_data,
	mipi_st_start,
	mipi_st_dv,
	neopixel_data,
	neopixel_clock,
	nina_spi_MOSI,
	nina_spi_SCLK,
	nina_spi_MISO,
	nina_spi_CS,
	pex_pio_in,
	pex_pio_dir,
	pex_pio_out,
	pex_pio_msel,
	qr_vid_in_data,
	qr_vid_in_dv,
	qr_vid_in_start,
	qr_vid_in_reset,
	qr_vid_in_clk,
	qr_vid_out_data,
	qr_vid_out_dv,
	qr_vid_out_start,
	qspi_dclk,
	qspi_ncs,
	qspi_oe,
	qspi_dataout,
	qspi_dataoe,
	qspi_datain,
	reset_reset_n,
	reset_0_reset_n,
	sam_pio_in,
	sam_pio_dir,
	sam_pio_out,
	sam_pio_msel,
	sam_pwm_pwm,
	sdram_addr,
	sdram_ba,
	sdram_cas_n,
	sdram_cke,
	sdram_cs_n,
	sdram_dq,
	sdram_dqm,
	sdram_ras_n,
	sdram_we_n,
	vid_clk,
	wm_pio_in,
	wm_pio_dir,
	wm_pio_out,
	wm_pio_msel);	

	input		arb_fb_clk;
	input		arb_fb_rdy;
	output	[30:0]	arb_fb_data;
	output		arb_fb_dv;
	output		arb_fb_start;
	input		arb_mipi_clk;
	input	[14:0]	arb_mipi_data;
	input		arb_mipi_dv;
	input		arb_mipi_start;
	input		clk_clk;
	input		clk_0_clk;
	input		csi_i2c_scl_i;
	output		csi_i2c_scl_o;
	output		csi_i2c_scl_en;
	input		csi_i2c_sda_i;
	output		csi_i2c_sda_o;
	output		csi_i2c_sda_en;
	input	[10:0]	encoder_encoder_a;
	input	[10:0]	encoder_encoder_b;
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
	output		flash_spi_MOSI;
	output		flash_spi_SCLK;
	input		flash_spi_MISO;
	output		flash_spi_CS;
	input		hdmi_i2c_scl_i;
	output		hdmi_i2c_scl_o;
	output		hdmi_i2c_scl_en;
	input		hdmi_i2c_sda_i;
	output		hdmi_i2c_sda_o;
	output		hdmi_i2c_sda_en;
	input	[1:0]	irq_in_port;
	output	[1:0]	irq_out_port;
	output		mb_ak;
	input		mb_rq;
	input		mipi_rx_clk;
	input	[1:0]	mipi_rx_data;
	output	[23:0]	mipi_st_data;
	output		mipi_st_start;
	output		mipi_st_dv;
	output	[11:0]	neopixel_data;
	output		neopixel_clock;
	output		nina_spi_MOSI;
	output		nina_spi_SCLK;
	input		nina_spi_MISO;
	output		nina_spi_CS;
	input	[31:0]	pex_pio_in;
	output	[31:0]	pex_pio_dir;
	output	[31:0]	pex_pio_out;
	output	[63:0]	pex_pio_msel;
	input	[23:0]	qr_vid_in_data;
	input		qr_vid_in_dv;
	input		qr_vid_in_start;
	input		qr_vid_in_reset;
	input		qr_vid_in_clk;
	output	[23:0]	qr_vid_out_data;
	output		qr_vid_out_dv;
	output		qr_vid_out_start;
	output		qspi_dclk;
	output		qspi_ncs;
	output		qspi_oe;
	output	[3:0]	qspi_dataout;
	output	[3:0]	qspi_dataoe;
	input	[3:0]	qspi_datain;
	input		reset_reset_n;
	input		reset_0_reset_n;
	input	[31:0]	sam_pio_in;
	output	[31:0]	sam_pio_dir;
	output	[31:0]	sam_pio_out;
	output	[63:0]	sam_pio_msel;
	output	[23:0]	sam_pwm_pwm;
	output	[11:0]	sdram_addr;
	output	[1:0]	sdram_ba;
	output		sdram_cas_n;
	output		sdram_cke;
	output		sdram_cs_n;
	inout	[15:0]	sdram_dq;
	output	[1:0]	sdram_dqm;
	output		sdram_ras_n;
	output		sdram_we_n;
	input		vid_clk;
	input	[31:0]	wm_pio_in;
	output	[31:0]	wm_pio_dir;
	output	[31:0]	wm_pio_out;
	output	[63:0]	wm_pio_msel;
endmodule
