
module forearm_control (
	clk_clk,
	reset_reset_n,
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
	myocontrol_0_conduit_end_angle_ss_n_o);	

	input		clk_clk;
	input		reset_reset_n;
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
endmodule
