// (C) 2001-2018 Intel Corporation. All rights reserved.
// Your use of Intel Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files from any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Intel Program License Subscription 
// Agreement, Intel FPGA IP License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Intel and sold by 
// Intel or its authorized distributors.  Please refer to the applicable 
// agreement for further details.



// Your use of Altera Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License Subscription 
// Agreement, Altera MegaCore Function License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Altera and sold by 
// Altera or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


// $Id: //acds/rel/18.1std/ip/merlin/altera_merlin_router/altera_merlin_router.sv.terp#1 $
// $Revision: #1 $
// $Date: 2018/07/18 $
// $Author: psgswbuild $

// -------------------------------------------------------
// Merlin Router
//
// Asserts the appropriate one-hot encoded channel based on 
// either (a) the address or (b) the dest id. The DECODER_TYPE
// parameter controls this behaviour. 0 means address decoder,
// 1 means dest id decoder.
//
// In the case of (a), it also sets the destination id.
// -------------------------------------------------------

`timescale 1 ns / 1 ns

module MKRVIDOR4000_graphics_lite_sys_mm_interconnect_1_router_001_default_decode
  #(
     parameter DEFAULT_CHANNEL = 0,
               DEFAULT_WR_CHANNEL = -1,
               DEFAULT_RD_CHANNEL = -1,
               DEFAULT_DESTID = 3 
   )
  (output [110 - 106 : 0] default_destination_id,
   output [21-1 : 0] default_wr_channel,
   output [21-1 : 0] default_rd_channel,
   output [21-1 : 0] default_src_channel
  );

  assign default_destination_id = 
    DEFAULT_DESTID[110 - 106 : 0];

  generate
    if (DEFAULT_CHANNEL == -1) begin : no_default_channel_assignment
      assign default_src_channel = '0;
    end
    else begin : default_channel_assignment
      assign default_src_channel = 21'b1 << DEFAULT_CHANNEL;
    end
  endgenerate

  generate
    if (DEFAULT_RD_CHANNEL == -1) begin : no_default_rw_channel_assignment
      assign default_wr_channel = '0;
      assign default_rd_channel = '0;
    end
    else begin : default_rw_channel_assignment
      assign default_wr_channel = 21'b1 << DEFAULT_WR_CHANNEL;
      assign default_rd_channel = 21'b1 << DEFAULT_RD_CHANNEL;
    end
  endgenerate

endmodule


module MKRVIDOR4000_graphics_lite_sys_mm_interconnect_1_router_001
(
    // -------------------
    // Clock & Reset
    // -------------------
    input clk,
    input reset,

    // -------------------
    // Command Sink (Input)
    // -------------------
    input                       sink_valid,
    input  [124-1 : 0]    sink_data,
    input                       sink_startofpacket,
    input                       sink_endofpacket,
    output                      sink_ready,

    // -------------------
    // Command Source (Output)
    // -------------------
    output                          src_valid,
    output reg [124-1    : 0] src_data,
    output reg [21-1 : 0] src_channel,
    output                          src_startofpacket,
    output                          src_endofpacket,
    input                           src_ready
);

    // -------------------------------------------------------
    // Local parameters and variables
    // -------------------------------------------------------
    localparam PKT_ADDR_H = 67;
    localparam PKT_ADDR_L = 36;
    localparam PKT_DEST_ID_H = 110;
    localparam PKT_DEST_ID_L = 106;
    localparam PKT_PROTECTION_H = 114;
    localparam PKT_PROTECTION_L = 112;
    localparam ST_DATA_W = 124;
    localparam ST_CHANNEL_W = 21;
    localparam DECODER_TYPE = 0;

    localparam PKT_TRANS_WRITE = 70;
    localparam PKT_TRANS_READ  = 71;

    localparam PKT_ADDR_W = PKT_ADDR_H-PKT_ADDR_L + 1;
    localparam PKT_DEST_ID_W = PKT_DEST_ID_H-PKT_DEST_ID_L + 1;



    // -------------------------------------------------------
    // Figure out the number of bits to mask off for each slave span
    // during address decoding
    // -------------------------------------------------------
    localparam PAD0 = log2ceil(64'h800000 - 64'h0); 
    localparam PAD1 = log2ceil(64'ha00000 - 64'h800000); 
    localparam PAD2 = log2ceil(64'ha04000 - 64'ha00000); 
    localparam PAD3 = log2ceil(64'ha04d00 - 64'ha04cc0); 
    localparam PAD4 = log2ceil(64'ha05420 - 64'ha05400); 
    localparam PAD5 = log2ceil(64'ha05440 - 64'ha05420); 
    localparam PAD6 = log2ceil(64'ha05460 - 64'ha05440); 
    localparam PAD7 = log2ceil(64'ha05480 - 64'ha05460); 
    localparam PAD8 = log2ceil(64'ha05500 - 64'ha05480); 
    localparam PAD9 = log2ceil(64'ha05540 - 64'ha05500); 
    localparam PAD10 = log2ceil(64'ha05580 - 64'ha05540); 
    localparam PAD11 = log2ceil(64'ha055a0 - 64'ha05580); 
    localparam PAD12 = log2ceil(64'ha055d0 - 64'ha055c0); 
    localparam PAD13 = log2ceil(64'ha05820 - 64'ha05800); 
    localparam PAD14 = log2ceil(64'ha06020 - 64'ha06000); 
    localparam PAD15 = log2ceil(64'ha07020 - 64'ha07000); 
    localparam PAD16 = log2ceil(64'ha0a000 - 64'ha08000); 
    localparam PAD17 = log2ceil(64'ha0a800 - 64'ha0a000); 
    localparam PAD18 = log2ceil(64'ha0fff0 - 64'ha0ffe8); 
    localparam PAD19 = log2ceil(64'ha10800 - 64'ha10000); 
    // -------------------------------------------------------
    // Work out which address bits are significant based on the
    // address range of the slaves. If the required width is too
    // large or too small, we use the address field width instead.
    // -------------------------------------------------------
    localparam ADDR_RANGE = 64'ha10800;
    localparam RANGE_ADDR_WIDTH = log2ceil(ADDR_RANGE);
    localparam OPTIMIZED_ADDR_H = (RANGE_ADDR_WIDTH > PKT_ADDR_W) ||
                                  (RANGE_ADDR_WIDTH == 0) ?
                                        PKT_ADDR_H :
                                        PKT_ADDR_L + RANGE_ADDR_WIDTH - 1;

    localparam RG = RANGE_ADDR_WIDTH-1;
    localparam REAL_ADDRESS_RANGE = OPTIMIZED_ADDR_H - PKT_ADDR_L;

      reg [PKT_ADDR_W-1 : 0] address;
      always @* begin
        address = {PKT_ADDR_W{1'b0}};
        address [REAL_ADDRESS_RANGE:0] = sink_data[OPTIMIZED_ADDR_H : PKT_ADDR_L];
      end   

    // -------------------------------------------------------
    // Pass almost everything through, untouched
    // -------------------------------------------------------
    assign sink_ready        = src_ready;
    assign src_valid         = sink_valid;
    assign src_startofpacket = sink_startofpacket;
    assign src_endofpacket   = sink_endofpacket;
    wire [PKT_DEST_ID_W-1:0] default_destid;
    wire [21-1 : 0] default_src_channel;




    // -------------------------------------------------------
    // Write and read transaction signals
    // -------------------------------------------------------
    wire write_transaction;
    assign write_transaction = sink_data[PKT_TRANS_WRITE];
    wire read_transaction;
    assign read_transaction  = sink_data[PKT_TRANS_READ];


    MKRVIDOR4000_graphics_lite_sys_mm_interconnect_1_router_001_default_decode the_default_decode(
      .default_destination_id (default_destid),
      .default_wr_channel   (),
      .default_rd_channel   (),
      .default_src_channel  (default_src_channel)
    );

    always @* begin
        src_data    = sink_data;
        src_channel = default_src_channel;
        src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = default_destid;

        // --------------------------------------------------
        // Address Decoder
        // Sets the channel and destination ID based on the address
        // --------------------------------------------------

    // ( 0x0 .. 0x800000 )
    if ( {address[RG:PAD0],{PAD0{1'b0}}} == 24'h0   ) begin
            src_channel = 21'b00000000000000000001;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 3;
    end

    // ( 0x800000 .. 0xa00000 )
    if ( {address[RG:PAD1],{PAD1{1'b0}}} == 24'h800000   ) begin
            src_channel = 21'b00000000000001000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 15;
    end

    // ( 0xa00000 .. 0xa04000 )
    if ( {address[RG:PAD2],{PAD2{1'b0}}} == 24'ha00000   ) begin
            src_channel = 21'b00100000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 11;
    end

    // ( 0xa04cc0 .. 0xa04d00 )
    if ( {address[RG:PAD3],{PAD3{1'b0}}} == 24'ha04cc0   ) begin
            src_channel = 21'b00000000000000100000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 14;
    end

    // ( 0xa05400 .. 0xa05420 )
    if ( {address[RG:PAD4],{PAD4{1'b0}}} == 24'ha05400   ) begin
            src_channel = 21'b00000010000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 16;
    end

    // ( 0xa05420 .. 0xa05440 )
    if ( {address[RG:PAD5],{PAD5{1'b0}}} == 24'ha05420   ) begin
            src_channel = 21'b00000100000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 13;
    end

    // ( 0xa05440 .. 0xa05460 )
    if ( {address[RG:PAD6],{PAD6{1'b0}}} == 24'ha05440   ) begin
            src_channel = 21'b00001000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 20;
    end

    // ( 0xa05460 .. 0xa05480 )
    if ( {address[RG:PAD7],{PAD7{1'b0}}} == 24'ha05460   ) begin
            src_channel = 21'b00000000000000001000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 6;
    end

    // ( 0xa05480 .. 0xa05500 )
    if ( {address[RG:PAD8],{PAD8{1'b0}}} == 24'ha05480  && write_transaction  ) begin
            src_channel = 21'b00000000000000000100;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 17;
    end

    // ( 0xa05500 .. 0xa05540 )
    if ( {address[RG:PAD9],{PAD9{1'b0}}} == 24'ha05500  && read_transaction  ) begin
            src_channel = 21'b00000000000000010000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 2;
    end

    // ( 0xa05540 .. 0xa05580 )
    if ( {address[RG:PAD10],{PAD10{1'b0}}} == 24'ha05540   ) begin
            src_channel = 21'b00000000000100000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 0;
    end

    // ( 0xa05580 .. 0xa055a0 )
    if ( {address[RG:PAD11],{PAD11{1'b0}}} == 24'ha05580   ) begin
            src_channel = 21'b00000000010000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 9;
    end

    // ( 0xa055c0 .. 0xa055d0 )
    if ( {address[RG:PAD12],{PAD12{1'b0}}} == 24'ha055c0   ) begin
            src_channel = 21'b00010000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 7;
    end

    // ( 0xa05800 .. 0xa05820 )
    if ( {address[RG:PAD13],{PAD13{1'b0}}} == 24'ha05800   ) begin
            src_channel = 21'b01000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 19;
    end

    // ( 0xa06000 .. 0xa06020 )
    if ( {address[RG:PAD14],{PAD14{1'b0}}} == 24'ha06000   ) begin
            src_channel = 21'b00000000000000000010;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 4;
    end

    // ( 0xa07000 .. 0xa07020 )
    if ( {address[RG:PAD15],{PAD15{1'b0}}} == 24'ha07000   ) begin
            src_channel = 21'b00000000100000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 5;
    end

    // ( 0xa08000 .. 0xa0a000 )
    if ( {address[RG:PAD16],{PAD16{1'b0}}} == 24'ha08000   ) begin
            src_channel = 21'b00000000001000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 1;
    end

    // ( 0xa0a000 .. 0xa0a800 )
    if ( {address[RG:PAD17],{PAD17{1'b0}}} == 24'ha0a000   ) begin
            src_channel = 21'b10000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 8;
    end

    // ( 0xa0ffe8 .. 0xa0fff0 )
    if ( {address[RG:PAD18],{PAD18{1'b0}}} == 24'ha0ffe8  && read_transaction  ) begin
            src_channel = 21'b00000000000010000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 18;
    end

    // ( 0xa10000 .. 0xa10800 )
    if ( {address[RG:PAD19],{PAD19{1'b0}}} == 24'ha10000   ) begin
            src_channel = 21'b00000001000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 10;
    end

end


    // --------------------------------------------------
    // Ceil(log2()) function
    // --------------------------------------------------
    function integer log2ceil;
        input reg[65:0] val;
        reg [65:0] i;

        begin
            i = 1;
            log2ceil = 0;

            while (i < val) begin
                log2ceil = log2ceil + 1;
                i = i << 1;
            end
        end
    endfunction

endmodule


