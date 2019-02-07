	forearm_control u0 (
		.clk_clk                                       (<connected-to-clk_clk>),                                       //                      clk.clk
		.reset_reset_n                                 (<connected-to-reset_reset_n>),                                 //                    reset.reset_n
		.myocontrol_0_conduit_end_miso                 (<connected-to-myocontrol_0_conduit_end_miso>),                 // myocontrol_0_conduit_end.miso
		.myocontrol_0_conduit_end_mosi                 (<connected-to-myocontrol_0_conduit_end_mosi>),                 //                         .mosi
		.myocontrol_0_conduit_end_sck                  (<connected-to-myocontrol_0_conduit_end_sck>),                  //                         .sck
		.myocontrol_0_conduit_end_ss_n                 (<connected-to-myocontrol_0_conduit_end_ss_n>),                 //                         .ss_n
		.myocontrol_0_conduit_end_mirrored_muscle_unit (<connected-to-myocontrol_0_conduit_end_mirrored_muscle_unit>), //                         .mirrored_muscle_unit
		.myocontrol_0_conduit_end_power_sense_n        (<connected-to-myocontrol_0_conduit_end_power_sense_n>),        //                         .power_sense_n
		.myocontrol_0_conduit_end_gpio_n               (<connected-to-myocontrol_0_conduit_end_gpio_n>),               //                         .gpio_n
		.myocontrol_0_conduit_end_angle_miso           (<connected-to-myocontrol_0_conduit_end_angle_miso>),           //                         .angle_miso
		.myocontrol_0_conduit_end_angle_mosi           (<connected-to-myocontrol_0_conduit_end_angle_mosi>),           //                         .angle_mosi
		.myocontrol_0_conduit_end_angle_sck            (<connected-to-myocontrol_0_conduit_end_angle_sck>),            //                         .angle_sck
		.myocontrol_0_conduit_end_angle_ss_n_o         (<connected-to-myocontrol_0_conduit_end_angle_ss_n_o>)          //                         .angle_ss_n_o
	);

