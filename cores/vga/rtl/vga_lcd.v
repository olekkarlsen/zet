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
    input clk,              // 25 Mhz clock
    input rst,

    input shift_reg1,       // if set: 320x200
    input graphics_alpha,   // if not set: 640x400 text mode

    // CSR slave interface for reading
    output [17:1] csr_adr_o,
    input  [15:0] csr_dat_i,
    output        csr_stb_o,

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
  wire [9:0] h_count;   // Horizontal pipeline delay is 2 cycles
  wire horiz_sync_i;
  wire [9:0] v_count;   // 0 to VER_SCAN_END
  wire video_on_h_i;
  wire video_on_v;
  
  wire horiz_sync_seq_o;
  wire vert_sync_seq_o;
  wire video_on_h_seq_o;
  wire video_on_v_seq_o;
  wire [7:0] character_seq_o;
  
  wire enable_crtc;
  wire enable_sequencer;
  wire enable_pal_dac;
  
  
  // Module instances
  vga_crtc crtc (
    .clk (clk),              // 25 Mhz clock
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
    .vert_sync (vert_sync),
    
    .video_on_h_i (video_on_h_i),
    .video_on_v (video_on_v)
    
  );
  
  vga_sequencer sequencer (
    .clk (clk),              // 25 Mhz clock
    .rst (rst),
    
    .enable_sequencer (enable_sequencer),
    
    // Sequencer input signals
    
    .h_count (h_count),
    .horiz_sync_i (horiz_sync_i),
    
    .v_count (v_count),
    .vert_sync (vert_sync),
    
    .video_on_h_i (video_on_h_i),
    .video_on_v (video_on_v),
    
    // Sequencer configuration signals
    
    .shift_reg1 (shift_reg1),       // if set: 320x200
    .graphics_alpha (graphics_alpha),   // if not set: 640x400 text mode

    // CSR slave interface for reading
    .csr_adr_o (csr_adr_o),
    .csr_dat_i (csr_dat_i),
    .csr_stb_o (csr_stb_o),

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
  
  vga_pal_dac pal_dac (
    .clk (clk),              // 25 Mhz clock
    .rst (rst),
    
    .enable_pal_dac (enable_pal_dac),
    
    // VGA PAL/DAC input signals
    
    .horiz_sync_pal_dac_i (horiz_sync_seq_o),
    .vert_sync_pal_dac_i (vert_sync_seq_o),
    .video_on_h_pal_dac_i (video_on_h_seq_o),
    .video_on_v_pal_dac_i (video_on_v_seq_o),
    .character_pal_dac_i (character_seq_o),
    
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
  assign enable_crtc = 1'b1;
  assign enable_sequencer = 1'b1;
  assign enable_pal_dac = 1'b1;

endmodule
