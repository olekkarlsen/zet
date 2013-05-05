/*
 *  VGA top level file
 *  Copyright (C) 2010  Zeus Gomez Marmolejo <zeus@aluzina.org>
 *  VGA SDRAM support added by Charley Picker <charleypicker@yahoo.com>
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

module vga (
    // Wishbone signals
    input         wb_clk_i,     // 100 Mhz VDU clock
    input         wb_rst_i,
    input  [15:0] wb_dat_i,
    output [15:0] wb_dat_o,
    input  [16:1] wb_adr_i,
    input         wb_we_i,
    input         wb_tga_i,
    input  [ 1:0] wb_sel_i,
    input         wb_stb_i,
    input         wb_cyc_i,
    output        wb_ack_o,

    // VGA pad signals
    output [ 3:0] vga_red_o,
    output [ 3:0] vga_green_o,
    output [ 3:0] vga_blue_o,
    output        horiz_sync,
    output        vert_sync,
    
    // VGA FML SDRAM Master Interface
    output [19:0] vga_fml_adr_o,
	output        vga_fml_stb_o,
	output        vga_fml_we_o,
	input         vga_fml_ack_i,
	output [ 1:0] vga_fml_sel_o,
	input  [15:0] vga_fml_di,
	output [15:0] vga_fml_do
  );


  // Registers and nets
  //
  
  // Config wires
  wire [15:0] conf_wb_dat_o;
  wire        conf_wb_ack_o;

  // Mem wires
  wire [15:0] mem_wb_dat_o;
  wire        mem_wb_ack_o;
  
  wire [19:1] wbm_adr_o;
  wire [19:1] lcdm_adr_o;  

  // LCD wires
  wire [17:1] lcd_adr_o;
  wire [15:0] lcdm_dat_i;
  wire [ 1:0] lcdm_sel_o;
  wire        lcdm_cyc_o;
  wire        lcdm_stb_o;
  wire        lcdm_ack_i;	
  wire        v_retrace;
  wire        vh_retrace;
  wire        w_vert_sync;

  // VGA configuration registers
  wire        shift_reg1;
  wire        graphics_alpha;
  wire        memory_mapping1;
  wire [ 1:0] write_mode;
  wire [ 1:0] raster_op;
  wire        read_mode;
  wire [ 7:0] bitmask;
  wire [ 3:0] set_reset;
  wire [ 3:0] enable_set_reset;
  wire [ 3:0] map_mask;
  wire        x_dotclockdiv2;
  wire        chain_four;
  wire [ 1:0] read_map_select;
  wire [ 3:0] color_compare;
  wire [ 3:0] color_dont_care;

  // Wishbone master to SDRAM
  wire [17:1] wb_adr_o;
  wire [ 1:0] wbm_sel_o;
  wire        wbm_we_o;
  wire [15:0] wbm_dat_o;
  wire [15:0] wbm_dat_i;
  wire        wbm_stb_o;
  wire        wbm_ack_i;

  wire        stb;

  // CRT wires
  wire [ 5:0] cur_start;
  wire [ 5:0] cur_end;
  wire [15:0] start_addr;
  wire [ 4:0] vcursor;
  wire [ 6:0] hcursor;
  wire [ 6:0] horiz_total;
  wire [ 6:0] end_horiz;
  wire [ 6:0] st_hor_retr;
  wire [ 4:0] end_hor_retr;
  wire [ 9:0] vert_total;
  wire [ 9:0] end_vert;
  wire [ 9:0] st_ver_retr;
  wire [ 3:0] end_ver_retr;

  // attribute_ctrl wires
  wire [3:0] pal_addr;
  wire       pal_we;
  wire [7:0] pal_read;
  wire [7:0] pal_write;

  // dac_regs wires
  wire       dac_we;
  wire [1:0] dac_read_data_cycle;
  wire [7:0] dac_read_data_register;
  wire [3:0] dac_read_data;
  wire [1:0] dac_write_data_cycle;
  wire [7:0] dac_write_data_register;
  wire [3:0] dac_write_data;

  // Module instances
  //
  vga_config_iface config_iface (
    .wb_clk_i (wb_clk_i),
    .wb_rst_i (wb_rst_i),
    .wb_dat_i (wb_dat_i),
    .wb_dat_o (conf_wb_dat_o),
    .wb_adr_i (wb_adr_i[4:1]),
    .wb_we_i  (wb_we_i),
    .wb_sel_i (wb_sel_i),
    .wb_stb_i (stb & wb_tga_i),
    .wb_ack_o (conf_wb_ack_o),

    .shift_reg1       (shift_reg1),
    .graphics_alpha   (graphics_alpha),
    .memory_mapping1  (memory_mapping1),
    .write_mode       (write_mode),
    .raster_op        (raster_op),
    .read_mode        (read_mode),
    .bitmask          (bitmask),
    .set_reset        (set_reset),
    .enable_set_reset (enable_set_reset),
    .map_mask         (map_mask),
    .x_dotclockdiv2   (x_dotclockdiv2),
    .chain_four       (chain_four),
    .read_map_select  (read_map_select),
    .color_compare    (color_compare),
    .color_dont_care  (color_dont_care),

    .pal_addr  (pal_addr),
    .pal_we    (pal_we),
    .pal_read  (pal_read),
    .pal_write (pal_write),

    .dac_we                  (dac_we),
    .dac_read_data_cycle     (dac_read_data_cycle),
    .dac_read_data_register  (dac_read_data_register),
    .dac_read_data           (dac_read_data),
    .dac_write_data_cycle    (dac_write_data_cycle),
    .dac_write_data_register (dac_write_data_register),
    .dac_write_data          (dac_write_data),

    .cur_start  (cur_start),
    .cur_end    (cur_end),
    .start_addr (start_addr),
    .vcursor    (vcursor),
    .hcursor    (hcursor),

    .horiz_total  (horiz_total),
    .end_horiz    (end_horiz),
    .st_hor_retr  (st_hor_retr),
    .end_hor_retr (end_hor_retr),
    .vert_total   (vert_total),
    .end_vert     (end_vert),
    .st_ver_retr  (st_ver_retr),
    .end_ver_retr (end_ver_retr),

    .v_retrace  (v_retrace),
    .vh_retrace (vh_retrace)
  );

  vga_lcd lcd (
    .clk (wb_clk_i),
    .rst (wb_rst_i),

    .shift_reg1     (shift_reg1),
    .graphics_alpha (graphics_alpha),

    .pal_addr  (pal_addr),
    .pal_we    (pal_we),
    .pal_read  (pal_read),
    .pal_write (pal_write),

    .dac_we                  (dac_we),
    .dac_read_data_cycle     (dac_read_data_cycle),
    .dac_read_data_register  (dac_read_data_register),
    .dac_read_data           (dac_read_data),
    .dac_write_data_cycle    (dac_write_data_cycle),
    .dac_write_data_register (dac_write_data_register),
    .dac_write_data          (dac_write_data),

    .lcd_adr_o(lcd_adr_o),
	.lcd_dat_i(lcdm_dat_i),
	.lcd_sel_o(lcdm_sel_o),
	.lcd_cyc_o(lcdm_cyc_o),
	.lcd_stb_o(lcdm_stb_o),
	.lcd_ack_i(lcdm_ack_i),
	
    .vga_red_o   (vga_red_o),
    .vga_green_o (vga_green_o),
    .vga_blue_o  (vga_blue_o),
    .horiz_sync  (horiz_sync),
    .vert_sync   (w_vert_sync),

    .cur_start  (cur_start),
    .cur_end    (cur_end),
    .vcursor    (vcursor),
    .hcursor    (hcursor),

    .horiz_total  (horiz_total),
    .end_horiz    (end_horiz),
    .st_hor_retr  (st_hor_retr),
    .end_hor_retr (end_hor_retr),
    .vert_total   (vert_total),
    .end_vert     (end_vert),
    .st_ver_retr  (st_ver_retr),
    .end_ver_retr (end_ver_retr),

    .x_dotclockdiv2 (x_dotclockdiv2),

    .v_retrace  (v_retrace),
    .vh_retrace (vh_retrace)
  );

  vga_cpu_mem_iface cpu_mem_iface (
    .wb_clk_i (wb_clk_i),
    .wb_rst_i (wb_rst_i),

    .wbs_adr_i (wb_adr_i),
    .wbs_sel_i (wb_sel_i),
    .wbs_we_i  (wb_we_i),
    .wbs_dat_i (wb_dat_i),
    .wbs_dat_o (mem_wb_dat_o),
    .wbs_stb_i (stb & !wb_tga_i),
    .wbs_ack_o (mem_wb_ack_o),

    .wbm_adr_o (wb_adr_o),
    .wbm_sel_o (wbm_sel_o),
    .wbm_we_o  (wbm_we_o),
    .wbm_dat_o (wbm_dat_o),
    .wbm_dat_i (wbm_dat_i),
    .wbm_stb_o (wbm_stb_o),
    .wbm_ack_i (wbm_ack_i),

    .chain_four       (chain_four),
    .memory_mapping1  (memory_mapping1),
    .write_mode       (write_mode),
    .raster_op        (raster_op),
    .read_mode        (read_mode),
    .bitmask          (bitmask),
    .set_reset        (set_reset),
    .enable_set_reset (enable_set_reset),
    .map_mask         (map_mask),
    .read_map_select  (read_map_select),
    .color_compare    (color_compare),
    .color_dont_care  (color_dont_care)
  );

  vga_mem_arbitrer #(
    .fml_depth(20)  // 1024KB Memory size
  ) mem_arbitrer (
    .clk_i (wb_clk_i),
    .rst_i (wb_rst_i),

    .cpu_adr_i (wbm_adr_o),
    .cpu_sel_i (wbm_sel_o),
    .cpu_we_i  (wbm_we_o),
    .cpu_dat_i (wbm_dat_o),
    .cpu_dat_o (wbm_dat_i),
    .cpu_stb_i (wbm_stb_o),
    .cpu_ack_o (wbm_ack_i),
    
    .lcd_adr_i (lcdm_adr_o),
	.lcd_dat_o (lcdm_dat_i),
	.lcd_sel_i (lcdm_sel_o),
	.lcd_cyc_i (lcdm_cyc_o),
	.lcd_stb_i (lcdm_stb_o),
	.lcd_ack_o (lcdm_ack_i),
	
	.fml_adr_o (vga_fml_adr_o),
	.fml_stb_o (vga_fml_stb_o),
	.fml_we_o  (vga_fml_we_o),
	.fml_ack_i (vga_fml_ack_i),
	.fml_sel_o (vga_fml_sel_o),
	.fml_di    (vga_fml_di),
	.fml_do    (vga_fml_do)
  );

  // Continous assignments
  assign wb_dat_o  = wb_tga_i ? conf_wb_dat_o : mem_wb_dat_o;
  assign wb_ack_o  = wb_tga_i ? conf_wb_ack_o : mem_wb_ack_o;
  assign stb       = wb_stb_i & wb_cyc_i;
  assign vert_sync = ~graphics_alpha ^ w_vert_sync;
  
  // Translate video memory address to full 1024KB address space
  // Add additional bits for wb memory access  
  assign wbm_adr_o = {2'b0, wb_adr_o};
  // Add video configuration lcd base address to lcd offset
  assign lcdm_adr_o = lcd_adr_o + start_addr[15:1];
  
endmodule
