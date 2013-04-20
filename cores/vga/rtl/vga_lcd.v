/*
 *  LCD controller for VGA
 *  Copyright (C) 2010  Zeus Gomez Marmolejo <zeus@aluzina.org>
 *  with modifications by Charley Picker <charleypicker@yahoo.com>
 *
 *  This file is part of the Zet processor. This processor is free
 *  hardware; you can redistribute it and/or modify it under the terms of
 *  the GNU General Public License as published by the Free Software
 *  Foundation; either version 3, or (at your option) any later version.
 *
 *  Zet is distrubuted in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 *  or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 *  License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Zet; see the file COPYING. If not, see
 *  <http://www.gnu.org/licenses/>.
 */

module vga_lcd (
    input clk,              // 100 Mhz clock
    input rst,

    input shift_reg1,       // if set: 320x200
    input graphics_alpha,   // if not set: 640x400 text mode

    // CSR slave interface for reading
    output [17:1] csr_adr_o,
    input  [15:0] csr_dat_i,
    output        csr_cyc_o,
    output        csr_stb_o,
    // input         csr_ack_i,  // not implemented yet

    // attribute_ctrl
    input  [3:0] pal_addr,
    input        pal_we,
    output [7:0] pal_read,
    input  [7:0] pal_write,

    // dac_regs
    input        dac_we,
    input  [1:0] dac_read_data_cycle,
    input  [7:0] dac_read_data_register,
    output [3:0] dac_read_data,
    input  [1:0] dac_write_data_cycle,
    input  [7:0] dac_write_data_register,
    input  [3:0] dac_write_data,

    // VGA pad signals
    output [3:0] vga_red_o,
    output [3:0] vga_green_o,
    output [3:0] vga_blue_o,
    output       horiz_sync,
    output       vert_sync,

    // CRTC
    input [5:0] cur_start,
    input [5:0] cur_end,
    input [4:0] vcursor,
    input [6:0] hcursor,

    input [6:0] horiz_total,
    input [6:0] end_horiz,
    input [6:0] st_hor_retr,
    input [4:0] end_hor_retr,
    input [9:0] vert_total,
    input [9:0] end_vert,
    input [9:0] st_ver_retr,
    input [3:0] end_ver_retr,

    input x_dotclockdiv2,

    // retrace signals
    output v_retrace,
    output vh_retrace
  );

  // Registers and nets
  // Hookup crtc output stage to sequencer input stage 
  wire [9:0] h_count;   // Horizontal pipeline delay is 2 cycles
  wire horiz_sync_i;
  wire [9:0] v_count;   // 0 to VER_SCAN_END
  wire vert_sync_crtc_o;
  wire video_on_h_i;
  wire video_on_v;
    
  // Hookup sequencer output stage to fifo input stage 
  wire [11:0] fb_dat_i;
  wire horiz_sync_seq_o;
  wire vert_sync_seq_o;
  wire video_on_h_seq_o;
  wire video_on_v_seq_o;
  wire [7:0] character_seq_o;
  
  // Hookup fifo output stage to pal_dac input stage
  wire [11:0] fb_dat_o;
  wire fb_horiz_sync_seq_o;
  wire fb_vert_sync_seq_o;
  wire fb_video_on_h_seq_o;
  wire fb_video_on_v_seq_o;
  wire [7:0] fb_character_seq_o;
  
  reg next_crtc_cyc;
  reg next_sequencer_cyc;
  
  // Memory controller signals
  wire mem_cyc_active;
  reg [3:0] mem_cyc_p;
  
  wire csr_adr_o_temp;
  wire csr_stb_o_temp;
    
  reg read_fifo;
  wire fill_fifo;
  wire fifo_low;
  wire [4:0] fb_data_fifo_nword;
    
  wire enable_crtc;
  wire enable_sequencer;
  wire enable_pal_dac;
  
  wire next_crtc_seq_cyc;
  reg next_pal_dac_cyc;
   
  reg [1:0] four_cyc_counter;
    
  // Module instances
  vga_crtc crtc (
    .clk (clk),              // 100 Mhz clock
    .rst (rst),
    
    .enable_crtc (enable_crtc),
    
    // CRTC configuration signals
    
    .cur_start (cur_start),
    .cur_end (cur_end),
    .vcursor (vcursor),
    .hcursor (hcursor),

    .horiz_total (horiz_total),
    .end_horiz (end_horiz),
    .st_hor_retr (st_hor_retr),
    .end_hor_retr (end_hor_retr),
    .vert_total (vert_total),
    .end_vert (end_vert),
    .st_ver_retr (st_ver_retr),
    .end_ver_retr (end_ver_retr),
    
    // CRTC output signals
    
    .h_count (h_count),
    .horiz_sync_i (horiz_sync_i),
    
    .v_count (v_count),
    .vert_sync (vert_sync_crtc_o),
    
    .video_on_h_i (video_on_h_i),
    .video_on_v (video_on_v)
    
  );
  
  vga_sequencer sequencer (
    .clk (clk),              // 100 Mhz clock
    .rst (rst),
    
    .enable_sequencer (enable_sequencer),
    
    // Sequencer input signals
    
    .h_count (h_count),
    .horiz_sync_i (horiz_sync_i),
    
    .v_count (v_count),
    .vert_sync (vert_sync_crtc_o),
    
    .video_on_h_i (video_on_h_i),
    .video_on_v (video_on_v),
    
    // Sequencer configuration signals
    
    .shift_reg1 (shift_reg1),       // if set: 320x200
    .graphics_alpha (graphics_alpha),   // if not set: 640x400 text mode

    // CSR slave interface for reading
    .csr_adr_o (csr_adr_o),
    .csr_dat_i (csr_dat_i),
    .csr_stb_o (csr_stb_o_temp),

    // CRTC
    .cur_start (cur_start),
    .cur_end (cur_end),
    .vcursor (vcursor),
    .hcursor (hcursor),

    .x_dotclockdiv2 (x_dotclockdiv2),
    
    // Sequencer output signals
    
    .horiz_sync_seq_o (horiz_sync_seq_o),
    .vert_sync_seq_o (vert_sync_seq_o),
    .video_on_h_seq_o (video_on_h_seq_o),
    .video_on_v_seq_o (video_on_v_seq_o),
    .character_seq_o (character_seq_o)  
    
  );
  
  // video-data buffer (temporary store data read from video memory)
  // We want to store at least one scan line (800 pixels x 12 bits per pixel) in the buffer
  vga_fifo #(4, 12) data_fifo (
    .clk    ( clk                ),
	.aclr   ( 1'b1               ),
	.sclr   ( rst                ),
	.d      ( fb_dat_i           ),
	.wreq   ( fill_fifo          ),
	.q      ( fb_dat_o           ),
	.rreq   ( read_fifo          ),
	.nword  ( fb_data_fifo_nword ),
	.empty  ( ),
	.full   ( ),
	.aempty ( ),
	.afull  ( )
  );
  
  vga_pal_dac pal_dac (
    .clk (clk),              // 100 Mhz clock
    .rst (rst),
    
    .enable_pal_dac (enable_pal_dac),
    
    // VGA PAL/DAC input signals
    
    .horiz_sync_pal_dac_i (fb_horiz_sync_seq_o),
    .vert_sync_pal_dac_i (fb_vert_sync_seq_o),
    .video_on_h_pal_dac_i (fb_video_on_h_seq_o),
    .video_on_v_pal_dac_i (fb_video_on_v_seq_o),
    .character_pal_dac_i (fb_character_seq_o),
    
    // VGA PAL/DAC configuration signals
    
    .shift_reg1 (shift_reg1),       // if set: 320x200
    .graphics_alpha (graphics_alpha),   // if not set: 640x400 text mode

    // attribute_ctrl
    .pal_addr (pal_addr),
    .pal_we (pal_we),
    .pal_read (pal_read),
    .pal_write (pal_write),

    // dac_regs
    .dac_we (dac_we),
    .dac_read_data_cycle (dac_read_data_cycle),
    .dac_read_data_register (dac_read_data_register),
    .dac_read_data (dac_read_data),
    .dac_write_data_cycle (dac_write_data_cycle),
    .dac_write_data_register (dac_write_data_register),
    .dac_write_data (dac_write_data),
    
    // VGA PAL/DAC output signals

    // VGA pad signals
    .vga_red_o (vga_red_o),
    .vga_green_o (vga_green_o),
    .vga_blue_o (vga_blue_o),
    .horiz_sync (horiz_sync),
    .vert_sync (vert_sync),

    // retrace signals
    .v_retrace (v_retrace),
    .vh_retrace (vh_retrace)
  );
  
  // Continuous assignments
  
  // Pack sequencer stage output into one wire group
  assign fb_dat_i = { horiz_sync_seq_o,
                      vert_sync_seq_o,
                      video_on_h_seq_o,
                      video_on_v_seq_o,
                      character_seq_o[7:0] };
  
  // Unpack fb_dat_o back into seperate wires
  assign fb_horiz_sync_seq_o = fb_dat_o [11];
  assign fb_vert_sync_seq_o = fb_dat_o [10];
  assign fb_video_on_h_seq_o = fb_dat_o [9];
  assign fb_video_on_v_seq_o = fb_dat_o [8];
  assign fb_character_seq_o = fb_dat_o [7:0];
  
  // Do not stall memory cycle during active or acknowledgement phases
  // assign csr_ack_i = 1'b1;  // csr_ack_i has not been implemented yet
  assign mem_cyc_active = mem_cyc_p[0] | mem_cyc_p[1] | mem_cyc_p[2] | mem_cyc_p[3]; // Is there an active memory cycle in progess request?
  
  // What level is the pixel fifo at?
  assign fifo_low = ~fb_data_fifo_nword[4] & ~fb_data_fifo_nword[3];
  
  // Determine next crtc/sequencer cycle
    
  // The next_crtc_seq_cyc should occur during mem cycle or when fifo needs filled
  assign next_crtc_seq_cyc = mem_cyc_active | fifo_low;
  
  // Disconnect csr_adr_o, csr_stb_o & csr_cyc_o from wishbone bus during a stall condition
  // assign csr_adr_o = next_crtc_seq_cyc ? csr_adr_o_temp : 1'b0;
  // wire csr_adr_o = csr_adr_o_temp;
  assign csr_stb_o = next_crtc_seq_cyc ? csr_stb_o_temp : 1'b0;
  // wire csr_stb_o = csr_stb_o_temp;
  // assign csr_cyc_o = next_crtc_seq_cyc ? mem_cyc_active : 1'b0;
    
  // These signals enable and control when the next crtc/sequencer cycle should occur
  assign enable_crtc = next_crtc_seq_cyc;
  assign enable_sequencer = next_crtc_seq_cyc;
  
  // When the next_crtc_seq_cyc occurs we should place another pixel in fifo
  assign fill_fifo = next_crtc_seq_cyc;
  
  // This signal enables and controls when the next pal_dac cycle should occure
  assign enable_pal_dac = next_pal_dac_cyc;  // 100 Mhz version
   
  // Behaviour
  // Provide counter for pal_dac stage
  always @(posedge clk)
  if (rst)
    begin
      four_cyc_counter <= 2'b00;
    end
  else
    begin
      if (four_cyc_counter == 2'b01)  // Toggle read_fifo
		  read_fifo <= 1'b1;
		else read_fifo <= 1'b0;
		
		if (four_cyc_counter == 2'b01)  // Toggle next_pal_dac_cyc
		  next_pal_dac_cyc <=1'b1;
		else next_pal_dac_cyc <= 1'b0;
		
		four_cyc_counter <= four_cyc_counter + 2'b01;  // Roll over every four cycles
		  
    end
    
    // mem_cyc_p pipe
  always @(posedge clk)
    if (rst)
      begin
        mem_cyc_p <= 4'b0;
      end
    else
      if (next_crtc_seq_cyc)
        begin
          mem_cyc_p <= { mem_cyc_p[2:0], csr_stb_o };
        end
  
endmodule
