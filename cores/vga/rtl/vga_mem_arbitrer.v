/*
 *  VGA SDRAM Memory arbitrer for VGA
 *  Copyright (C) 2013  Charley Picker <charleypicker@yahoo.com>
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

module vga_mem_arbitrer #(
    parameter fml_depth = 20 // 1024KB Memory address range
) (
	input clk_i,
	input rst_i,	

    // CPU Video Memory Access wishbone slave interface(Read/Write)
    input      [fml_depth-1:1] cpu_adr_i,
    input      [ 1:0] cpu_sel_i,
    input             cpu_cyc_i,
    input             cpu_we_i,
    input      [15:0] cpu_dat_i,
    output reg [15:0] cpu_dat_o,
    input             cpu_stb_i,
    output reg        cpu_ack_o,
    
    // LCD Video Memory Access wishbone slave interface(Read Only)
    input      [fml_depth-1:1] lcd_adr_i,
    input      [ 1:0] lcd_sel_i,
    input             lcd_cyc_i,
    output reg [15:0] lcd_dat_o,
    input             lcd_stb_i,
    output reg        lcd_ack_o,
        
    // FML master interface to sdram
	output reg [fml_depth-1:0] fml_adr_o,
	output reg        fml_stb_o,
	output reg        fml_we_o,
	input             fml_ack_i,
	output reg [ 1:0] fml_sel_o,
	input      [15:0] fml_di,
	output reg [15:0] fml_do    
  );
  
  // CPU Registers and nets
  wire cpu_data_op;
  wire [fml_depth-3-1:4] cpu_req_address;  
  wire [ 2:0] cpu_req_offset;
  
  reg [fml_depth-3-1:4] cpu_data_address;
  wire req_cpu_address_matched;
  wire req_new_cpu_address;
  
  reg [7:0] cpu_data_ram0 [0:7];
  reg [7:0] cpu_data_ram1 [0:7];  
  reg cpu_data_valid, cpu_data_dirty; 
  
  wire cpu_read_cpu_data;
  wire cpu_write_cpu_data;
  
  // LCD Registers and nets
  wire lcd_data_op;
  wire [fml_depth-3-1:4] lcd_req_address;  
  wire [ 2:0] lcd_req_offset;
  
  reg [fml_depth-3-1:4] lcd_data_address;
  wire req_lcd_address_matched;
  wire req_new_lcd_address;
  
  reg [7:0] lcd_data_ram0 [0:7];
  reg [7:0] lcd_data_ram1 [0:7];  
  reg lcd_data_valid; 
  
  wire lcd_read_lcd_data;
  
  // LCD <- CPU Read Registers and nets
  wire req_lcd_address_matched_cpu;    
  wire lcd_read_cpu_data;
  
  // FML Arbitrer Registers and nets
  wire req_cpu_lcd_address_matched;
  wire wait_cpu_read_write_fml;
  
  wire cpu_read_write_fml;
  wire lcd_read_fml;
  
  // CPU Continuous assignments
  assign cpu_data_op = cpu_stb_i & cpu_cyc_i;
  assign cpu_req_address = cpu_adr_i[fml_depth-3-1:4];
  assign cpu_req_offset  = cpu_adr_i[ 3:1];
  
  assign req_cpu_address_matched = (cpu_data_address == cpu_req_address);
  assign req_new_cpu_address = cpu_data_op & (!cpu_data_valid | !req_cpu_address_matched);
  
  assign cpu_read_cpu_data = cpu_data_op & cpu_data_valid & !cpu_we_i & req_cpu_address_matched;
  assign cpu_write_cpu_data = cpu_data_op & cpu_data_valid & cpu_we_i & req_cpu_address_matched;
  
  // LCD Continuous assignments
  assign lcd_data_op = lcd_stb_i & lcd_cyc_i;
  assign lcd_req_address = lcd_adr_i[fml_depth-3-1:4];
  assign lcd_req_offset  = lcd_adr_i[ 3:1];
  
  assign req_lcd_address_matched = (lcd_data_address == lcd_req_address);
  assign req_new_lcd_address = lcd_data_op & (!lcd_data_valid | !req_lcd_address_matched);
  
  assign lcd_read_lcd_data = lcd_data_op & lcd_data_valid & req_lcd_address_matched;
  
  // LCD <- CPU Read Continuous assignments
  assign req_lcd_address_matched_cpu = (lcd_req_address == cpu_data_address);    
  assign lcd_read_cpu_data = lcd_data_op & cpu_data_valid & req_lcd_address_matched_cpu;
  
  // FML Arbritrer Continuous assignments
  assign req_cpu_lcd_address_matched = (cpu_req_address == lcd_req_address);
  assign wait_cpu_read_write_fml = cpu_data_op & req_cpu_lcd_address_matched;
  
  assign cpu_read_write_fml = req_new_cpu_address;
  assign lcd_read_fml = req_new_lcd_address & !lcd_read_cpu_data & !wait_cpu_read_write_fml;  
    
  // CPU Read/Write Behaviour
  // CPU Read cpu_data
  always @(posedge clk_i)
    if (rst_i)
      cpu_dat_o <= 16'b0;
    else
      if (cpu_read_cpu_data)
        begin
          if (cpu_sel_i[0])
            cpu_dat_o[7:0] <= cpu_data_ram0[cpu_req_offset];            
          if (cpu_sel_i[1])
            cpu_dat_o[15:8] <= cpu_data_ram1[cpu_req_offset];    
        end    
  
  // CPU Write cpu_data
  always @(posedge clk_i)
    if (rst_i)
      cpu_data_dirty <= 1'b0;           
    else
      begin
        if (cpu_write_cpu_data)
          begin
            if (cpu_sel_i[0])
              cpu_data_ram0[cpu_req_offset] <= cpu_dat_i[7:0];              
            if (cpu_sel_i[1])
              cpu_data_ram1[cpu_req_offset] <= cpu_dat_i[15:8];             
            cpu_data_dirty <= 1'b1;    
          end        
      end
  
  // CPU Acknowledge cpu_data Read/Write
  always @(posedge clk_i)
    if (rst_i)
      cpu_ack_o <= 1'b0;
    else
      if (cpu_read_cpu_data | cpu_write_cpu_data)
        cpu_ack_o <= 1'b1;
      else
        cpu_ack_o <= 1'b0;
      
  // LCD Read Behaviour
  // LCD Read lcd_data
  always @(posedge clk_i)
    if (rst_i)
      lcd_dat_o <= 16'b0;
    else
      if (lcd_read_cpu_data)
        begin
          if (lcd_sel_i[0])
            lcd_dat_o[7:0] <= cpu_data_ram0[lcd_req_offset];            
          if (lcd_sel_i[1])
            lcd_dat_o[15:8] <= cpu_data_ram1[lcd_req_offset];    
        end
      else
        if (lcd_read_lcd_data)
          begin
            if (lcd_sel_i[0])
              lcd_dat_o[7:0] <= lcd_data_ram0[lcd_req_offset];            
            if (lcd_sel_i[1])
              lcd_dat_o[15:8] <= lcd_data_ram1[lcd_req_offset];
          end
  
  // LCD Acknowledge lcd_data Read
  always @(posedge clk_i)
    if (rst_i)
      lcd_ack_o <= 1'b0;
    else
      if (lcd_read_cpu_data | lcd_read_lcd_data)
        lcd_ack_o <= 1'b1;
      else
        lcd_ack_o <= 1'b0;      
    
  // FML Behaviour
  // FML Read/Write FSM
  reg [4:0] state;
  reg [4:0] next_state;

  parameter IDLE         = 5'd0;
  
  parameter CPU_EVICT    = 5'd1;
  parameter CPU_EVICT2   = 5'd2;
  parameter CPU_EVICT3   = 5'd3;
  parameter CPU_EVICT4   = 5'd4;
  parameter CPU_EVICT5   = 5'd5;
  parameter CPU_EVICT6   = 5'd6;
  parameter CPU_EVICT7   = 5'd7;
  parameter CPU_EVICT8   = 5'd8;

  parameter CPU_REFILL   = 5'd9;
  parameter CPU_REFILL2  = 5'd10;
  parameter CPU_REFILL3  = 5'd11;
  parameter CPU_REFILL4  = 5'd12;
  parameter CPU_REFILL5  = 5'd13;
  parameter CPU_REFILL6  = 5'd14;
  parameter CPU_REFILL7  = 5'd15;
  parameter CPU_REFILL8  = 5'd16;
  
  parameter LCD_REFILL   = 5'd17;
  parameter LCD_REFILL2  = 5'd18;
  parameter LCD_REFILL3  = 5'd19;
  parameter LCD_REFILL4  = 5'd20;
  parameter LCD_REFILL5  = 5'd21;
  parameter LCD_REFILL6  = 5'd22;
  parameter LCD_REFILL7  = 5'd23;
  parameter LCD_REFILL8  = 5'd24;

always @(posedge clk_i) begin
	if(rst_i) begin
	  fml_adr_o <= 20'b0;
	  fml_stb_o <= 1'b0;
	  fml_we_o <= 1'b0;
	  fml_do <= 16'b0;
	  
	  cpu_data_address <= 17'b0;
	  cpu_data_valid <= 1'b0;
	  
	  lcd_data_address <= 17'b0;
	  lcd_data_valid <= 1'b0;
	  	  	  
	  state <= IDLE;	    
	end		
	else begin
		// $display("state: %d -> %d", state, next_state);
		state <= next_state;
	end
end

always @(*) begin
	fml_stb_o = 1'b0;
	fml_we_o = 1'b0;
	
	next_state = state;
	
	case(state)
		IDLE: begin
		  if (lcd_read_fml)
		    begin
		      // Prevent data change during lcd refill phase
		      lcd_data_valid = 1'b0;
		      
		      next_state = LCD_REFILL; 
		    end
		  else
		    if(cpu_read_write_fml)
		      begin
		        // Prevent data change during cpu evict/refill phase
		        cpu_data_valid = 1'b0;
		        	    
		        if(cpu_data_dirty)
		          next_state = CPU_EVICT;
		        else
		          next_state = CPU_REFILL;
		      end
		end
		
		CPU_EVICT: begin
		  fml_adr_o = { cpu_data_address, 3'b0};
		  fml_do = {cpu_data_ram1[0], cpu_data_ram0[0]};
		  fml_stb_o = 1'b1;
		  fml_we_o = 1'b1;
		  if(fml_ack_i)
		      next_state = CPU_EVICT2;
		end
		CPU_EVICT2: begin
		  fml_do = {cpu_data_ram1[1], cpu_data_ram0[1]};
		  next_state = CPU_EVICT3;
		end
		CPU_EVICT3: begin
		  fml_do = {cpu_data_ram1[2], cpu_data_ram0[2]};
		  next_state = CPU_EVICT4;
		end
		CPU_EVICT4: begin
		  fml_do = {cpu_data_ram1[3], cpu_data_ram0[3]};
		  next_state = CPU_EVICT5;
		end
		CPU_EVICT5: begin
		  fml_do = {cpu_data_ram1[4], cpu_data_ram0[4]};
		  next_state = CPU_EVICT6;
		end
		CPU_EVICT6: begin
		  fml_do = {cpu_data_ram1[5], cpu_data_ram0[5]};
		  next_state = CPU_EVICT7;
		end
		CPU_EVICT7: begin
		  fml_do = {cpu_data_ram1[6], cpu_data_ram0[6]};
		  next_state = CPU_EVICT8;
		end
		CPU_EVICT8: begin
		  fml_do = {cpu_data_ram1[7], cpu_data_ram0[7]};
		  next_state = CPU_REFILL;
		end
		
		CPU_REFILL: begin
		  // Write the tag first		  
		  if(cpu_we_i)
		    cpu_data_dirty = 1'b1;
		  else
		    cpu_data_dirty = 1'b0;
		    
		  cpu_data_address = cpu_req_address;
		    
		  fml_adr_o = { cpu_req_address, 3'b0 };
		  cpu_data_ram0[0] = fml_di[7:0];
		  cpu_data_ram1[0] = fml_di[15:8];
		  fml_stb_o = 1'b1;
		  if(fml_ack_i)
		    next_state = CPU_REFILL2;		  
		end
		CPU_REFILL2: begin
		  cpu_data_ram0[1] = fml_di[7:0];
		  cpu_data_ram1[1] = fml_di[15:8];
		  next_state = CPU_REFILL3;
		end
		CPU_REFILL3: begin
		  cpu_data_ram0[2] = fml_di[7:0];
		  cpu_data_ram1[2] = fml_di[15:8];
		  next_state = CPU_REFILL4;
		end
		CPU_REFILL4: begin
		  cpu_data_ram0[3] = fml_di[7:0];
		  cpu_data_ram1[3] = fml_di[15:8];
		  next_state = CPU_REFILL5;
		end
		CPU_REFILL5: begin
		  cpu_data_ram0[4] = fml_di[7:0];
		  cpu_data_ram1[4] = fml_di[15:8];
		  next_state = CPU_REFILL6;
		end
		CPU_REFILL6: begin
		  cpu_data_ram0[5] = fml_di[7:0];
		  cpu_data_ram1[5] = fml_di[15:8];
		  next_state = CPU_REFILL7;
		end
		CPU_REFILL7: begin
		  cpu_data_ram0[6] = fml_di[7:0];
		  cpu_data_ram1[6] = fml_di[15:8];
		  next_state = CPU_REFILL8;
		end
		CPU_REFILL8: begin
		  cpu_data_ram0[7] = fml_di[7:0];
		  cpu_data_ram1[7] = fml_di[15:8];
		  
		  // We can now read/write the new loaded data
		  cpu_data_valid = 1'b1;
		  next_state = IDLE;
		end
		
		LCD_REFILL: begin
		  lcd_data_address = lcd_req_address;
		    
		  fml_adr_o = { lcd_req_address, 3'b0 };
		  lcd_data_ram0[0] = fml_di[7:0];
		  lcd_data_ram1[0] = fml_di[15:8];
		  fml_stb_o = 1'b1;
		  if(fml_ack_i)
		    next_state = LCD_REFILL2;		  
		end
		LCD_REFILL2: begin
		  lcd_data_ram0[1] = fml_di[7:0];
		  lcd_data_ram1[1] = fml_di[15:8];
		  next_state = LCD_REFILL3;
		end
		LCD_REFILL3: begin
		  lcd_data_ram0[2] = fml_di[7:0];
		  lcd_data_ram1[2] = fml_di[15:8];
		  next_state = LCD_REFILL4;
		end
		LCD_REFILL4: begin
		  lcd_data_ram0[3] = fml_di[7:0];
		  lcd_data_ram1[3] = fml_di[15:8];
		  next_state = LCD_REFILL5;
		end
		LCD_REFILL5: begin
		  lcd_data_ram0[4] = fml_di[7:0];
		  lcd_data_ram1[4] = fml_di[15:8];
		  next_state = LCD_REFILL6;
		end
		LCD_REFILL6: begin
		  lcd_data_ram0[5] = fml_di[7:0];
		  lcd_data_ram1[5] = fml_di[15:8];
		  next_state = LCD_REFILL7;
		end
		LCD_REFILL7: begin
		  lcd_data_ram0[6] = fml_di[7:0];
		  lcd_data_ram1[6] = fml_di[15:8];
		  next_state = LCD_REFILL8;
		end
		LCD_REFILL8: begin
		  lcd_data_ram0[7] = fml_di[7:0];
		  lcd_data_ram1[7] = fml_di[15:8];
		  
		  // We can now read the new loaded data
		  lcd_data_valid = 1'b1;
		  next_state = IDLE;
		end
	endcase
end

endmodule
