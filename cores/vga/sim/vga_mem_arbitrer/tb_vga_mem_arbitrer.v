/*
 * Milkymist VJ SoC
 * Copyright (C) 2007, 2008, 2009 Sebastien Bourdeauducq
 * adjusted to single line cache by Charley Picker <charleypicker@yahoo.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

`timescale 1ns / 1ps

`define ENABLE_VCD

module tb_vga_mem_arbitrer;

reg clk;
initial clk = 1'b0;
always #5 clk = ~clk;

reg rst;

// 1024KB Memory address range
reg [19:1] wb_adr_i;
reg [15:0] wb_dat_i;
wire [15:0] wb_dat_o;
reg [1:0] wb_sel_i;
reg wb_cyc_i;
reg wb_stb_i;
reg wb_we_i;
wire wb_ack_o;

reg [19:1] lcd_adr_i;
wire [15:0] lcd_dat_o;
reg [1:0] lcd_sel_i;
reg lcd_cyc_i;
reg lcd_stb_i;
wire lcd_ack_o;

wire [19:0] fml_adr;
wire fml_stb;
wire fml_we;
reg fml_ack;
wire [1:0] fml_sel;
wire [15:0] fml_dw;
reg [15:0] fml_dr;

/* Process FML requests */
reg [2:0] fml_wcount;
reg [2:0] fml_rcount;
initial begin
	fml_ack = 1'b0;
	fml_wcount = 0;
	fml_rcount = 0;
end

always @(posedge clk) begin
	if(fml_stb & (fml_wcount == 0) & (fml_rcount == 0)) begin
		fml_ack <= 1'b1;
		if(fml_we) begin
			$display("%t FML W addr %x data %x", $time, fml_adr, fml_dw);
			fml_wcount <= 7;
		end else begin
			fml_dr = 16'hbeef;
			$display("%t FML R addr %x data %x", $time, fml_adr, fml_dr);
			fml_rcount <= 7;
		end
	end else
		fml_ack <= 1'b0;
	if(fml_wcount != 0) begin
		#1 $display("%t FML W continuing %x / %d", $time, fml_dw, fml_wcount);
		fml_wcount <= fml_wcount - 1;
	end
	if(fml_rcount != 0) begin
		fml_dr = #1 {13'h1eba, fml_rcount};
		$display("%t FML R continuing %x / %d", $time, fml_dr, fml_rcount);
		fml_rcount <= fml_rcount - 1;
	end
end

vga_mem_arbitrer #(
  .fml_depth(20)
) dut (
    .clk_i(clk),
	.rst_i(rst),
	
	.cpu_adr_i(wb_adr_i),
	.cpu_dat_i(wb_dat_i),
	.cpu_dat_o(wb_dat_o),
	.cpu_sel_i(wb_sel_i),
	.cpu_cyc_i(wb_cyc_i),
	.cpu_stb_i(wb_stb_i),
	.cpu_we_i(wb_we_i),
	.cpu_ack_o(wb_ack_o),
	
	.lcd_adr_i(lcd_adr_i),
	.lcd_dat_o(lcd_dat_o),
	.lcd_sel_i(lcd_sel_i),
	.lcd_cyc_i(lcd_cyc_i),
	.lcd_stb_i(lcd_stb_i),
	.lcd_ack_o(lcd_ack_o),
	
	.fml_adr_o(fml_adr),
	.fml_stb_o(fml_stb),
	.fml_we_o(fml_we),
	.fml_ack_i(fml_ack),
	.fml_sel_o(fml_sel),
	.fml_di(fml_dr),
	.fml_do(fml_dw)	
);

task waitclock;
begin
	@(posedge clk);
	#1;
end
endtask

task wbwrite;
input [19:1] address;
input [15:0] data;
input [1:0] byte_sel;
integer i;
begin
	wb_adr_i = address;
	wb_dat_i = data;
	wb_sel_i = byte_sel;
	//wb_sel_i = 2'b11;
	wb_sel_i = byte_sel;
	wb_cyc_i = 1'b1;
	wb_stb_i = 1'b1;
	wb_we_i = 1'b1;
	i = 0;
	waitclock;
	i = i+1;
	while(~wb_ack_o) begin
	    i = i+1;
		waitclock;
	end
	$display("WB Write: address=%h data=%h sel=%b acked in %d clocks", address, data, wb_sel_i, i);
	wb_adr_i = 20'hx;
	wb_cyc_i = 1'b0;
	wb_stb_i = 1'b0;
	wb_we_i = 1'b0;
end
endtask

task wbread;
input [19:1] address;
input [1:0] byte_sel;
integer i;
begin
	wb_adr_i = address;
	wb_sel_i = byte_sel;
	wb_cyc_i = 1'b1;
	wb_stb_i = 1'b1;
	wb_we_i = 1'b0;
	i = 0;
	waitclock;
	i = i+1;
	while(~wb_ack_o) begin
	    i = i+1;
		waitclock;
	end
	$display("WB Read : address=%h data=%h sel=%b acked in %d clocks", address, wb_dat_o, wb_sel_i, i);
	wb_adr_i = 20'hx;
	wb_cyc_i = 1'b0;
	wb_stb_i = 1'b0;
	wb_we_i = 1'b0;
end
endtask

task lcdread;
input [19:1] address;
input [1:0] byte_sel;
integer i;
begin
	lcd_adr_i = address;
	lcd_sel_i = byte_sel;
	lcd_cyc_i = 1'b1;
	lcd_stb_i = 1'b1;
	
	i = 0;
	waitclock;
	i = i+1;
	while(~lcd_ack_o) begin
	    i = i+1;
		waitclock;
	end
	$display("LCD Read : address=%h data=%h sel=%b acked in %d clocks", address, lcd_dat_o, lcd_sel_i, i);
	lcd_adr_i = 20'hx;
	lcd_cyc_i = 1'b0;
	lcd_stb_i = 1'b0;	
end
endtask

always begin
// `ifdef ENABLE_VCD
//	$dumpfile("vga_mem_arbitrer.vcd");
//	$dumpvars(0, dut);
// `endif
	rst = 1'b1;
	
	wb_adr_i = 20'd0;
	wb_dat_i = 16'h0;
	wb_sel_i = 2'b00;
	wb_cyc_i = 1'b0;
	wb_stb_i = 1'b0;
	wb_we_i = 1'b0;
	
	waitclock;
	
	rst = 1'b0;
	
	waitclock;
	
	$display("Testing: read miss");
	wbread(20'h0, 2'b01);
	
	$display("Testing: write hit");
	wbwrite(20'h0, 16'h0123, 2'b11);
	wbwrite(20'h1, 16'h1234, 2'b01);
	wbwrite(20'h2, 16'h2345, 2'b10);
	wbwrite(20'h3, 16'h3456, 2'b11);
	wbwrite(20'h4, 16'h4567, 2'b11);
	wbwrite(20'h5, 16'h5678, 2'b11);
	wbwrite(20'h6, 16'h6789, 2'b11);
	wbwrite(20'h7, 16'h789a, 2'b11);
	
	$display("Reading back values written through WB interface");
	
	wbread(20'h3, 2'b11);
	wbread(20'h0, 2'b11);	
	wbread(20'h1, 2'b11);
	wbread(20'h2, 2'b11);
	wbread(20'h5, 2'b11);
	wbread(20'h6, 2'b11);
	wbread(20'h7, 2'b11);
	wbread(20'h4, 2'b11);
	
	$display("Reading back values written through LCD interface");
	lcdread(20'h3, 2'b11);
	lcdread(20'h0, 2'b11);	
	lcdread(20'h1, 2'b11);
	lcdread(20'h2, 2'b11);
	lcdread(20'h5, 2'b11);
	lcdread(20'h6, 2'b11);
	lcdread(20'h7, 2'b11);
	lcdread(20'h4, 2'b11);	
	
	$display("Testing WB write beyond 16 byte boundry");
	wbwrite(20'h8, 16'h789a, 2'b11);
	
	$display("Testing LCD read beyond 16 byte boundry");
	lcdread(20'h8, 2'b11);
	
	$display("Testing LCD read beyond 16 byte boundry");
	lcdread(20'h0, 2'b11);
	
	$display("Testing: read miss on a dirty line");
	wbread(20'h01000, 2'b11);
	
	$display("Testing: read hit");
	wbread(20'h01004, 2'b11);
	
	$display("Testing: write miss");
	wbwrite(20'h0, 16'hface, 2'b11);
	wbread(20'h0, 2'b11);
	wbread(20'h4, 2'b11);
	
	$stop;
	
end


endmodule
