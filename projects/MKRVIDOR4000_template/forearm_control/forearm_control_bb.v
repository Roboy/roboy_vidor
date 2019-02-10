
module forearm_control (
	clk_clk,
	myocontrol_0_conduit_end_miso,
	myocontrol_0_conduit_end_mosi,
	myocontrol_0_conduit_end_sck,
	myocontrol_0_conduit_end_ss_n,
	myocontrol_0_conduit_end_mirrored_muscle_unit,
	myocontrol_0_conduit_end_power_sense_n,
	myocontrol_0_conduit_end_gpio_n,
	myocontrol_0_conduit_end_angle_miso,
	myocontrol_0_conduit_end_angle_mosi,
	myocontrol_0_conduit_end_angle_sck,
	myocontrol_0_conduit_end_angle_ss_n_o,
	reset_reset_n,
	myocontrol_0_avalon_slave_0_address,
	myocontrol_0_avalon_slave_0_write,
	myocontrol_0_avalon_slave_0_writedata,
	myocontrol_0_avalon_slave_0_read,
	myocontrol_0_avalon_slave_0_readdata,
	myocontrol_0_avalon_slave_0_waitrequest);	

	input		clk_clk;
	input		myocontrol_0_conduit_end_miso;
	output		myocontrol_0_conduit_end_mosi;
	output		myocontrol_0_conduit_end_sck;
	output	[2:0]	myocontrol_0_conduit_end_ss_n;
	input		myocontrol_0_conduit_end_mirrored_muscle_unit;
	input		myocontrol_0_conduit_end_power_sense_n;
	output		myocontrol_0_conduit_end_gpio_n;
	input		myocontrol_0_conduit_end_angle_miso;
	output		myocontrol_0_conduit_end_angle_mosi;
	output		myocontrol_0_conduit_end_angle_sck;
	output	[2:0]	myocontrol_0_conduit_end_angle_ss_n_o;
	input		reset_reset_n;
	input	[15:0]	myocontrol_0_avalon_slave_0_address;
	input		myocontrol_0_avalon_slave_0_write;
	input	[31:0]	myocontrol_0_avalon_slave_0_writedata;
	input		myocontrol_0_avalon_slave_0_read;
	output	[31:0]	myocontrol_0_avalon_slave_0_readdata;
	output		myocontrol_0_avalon_slave_0_waitrequest;
endmodule
