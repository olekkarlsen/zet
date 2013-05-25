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
    output     [15:0] cpu_dat_o,
    input             cpu_stb_i,
    output reg        cpu_ack_o,
    
    // LCD Video Memory Access wishbone slave interface(Read Only)
    input      [fml_depth-1:1] lcd_adr_i,
    input      [ 1:0] lcd_sel_i,
    input             lcd_cyc_i,
    output     [15:0] lcd_dat_o,
    input             lcd_stb_i,
    output reg        lcd_ack_o,
        
    // FML master interface to sdram
	output reg [fml_depth-1:0] fml_adr_o,
	output reg        fml_stb_o,
	output reg        fml_we_o,
	input             fml_ack_i,
	output     [ 1:0] fml_sel_o,
	input      [15:0] fml_di,
	output     [15:0] fml_do    
  );
  
  // CPU Registers and nets
  wire cpu_data_op;
  wire [fml_depth-3-1:4] cpu_req_address;  
  wire [ 2:0] cpu_req_offset;
  
  reg [fml_depth-3-1:4] cpu_data_address;
  reg cpu_datamem_valid, cpu_datamem_dirty;
  wire cpu_set_cpu_datamem_dirty;
  
  wire req_cpu_address_matched_cpu;
  
  wire req_new_cpu_address;
  
  wire cpu_datamem_wb_ack;
  
  wire [2:0]  cpu_datamem_a;
  wire [1:0]  cpu_datamem_we;
  wire [15:0] cpu_datamem_di;
  wire [15:0] cpu_datamem_do;
  
  wire [2:0]  cpu_datamem_a2;
  wire [15:0] cpu_datamem_do2;  
    
  // LCD Registers and nets
  wire lcd_data_op;
  wire [fml_depth-3-1:4] lcd_req_address;  
  wire [ 2:0] lcd_req_offset;
  
  reg [fml_depth-3-1:4] lcd_data_address;
  reg lcd_datamem_valid;
  
  wire req_lcd_address_matched_lcd;
  wire req_lcd_address_matched_cpu;
  wire req_lcd_cpu_address_matched;
       
  wire req_new_lcd_address;
  
  wire lcd_datamem_wb_ack;
  
  wire [2:0]  lcd_datamem_a;
  wire [1:0]  lcd_datamem_we;
  wire [15:0] lcd_datamem_di;
  wire [15:0] lcd_datamem_do;
  
  wire [2:0]  lcd_datamem_a2;
  wire [15:0] lcd_datamem_do2;
    
  // FML Arbitrer Registers and nets
  wire [15:0] cpu_dat_do_mux;
  wire [15:0] lcd_dat_op_mux;
  wire [15:0] cpu_dat2_op_mux;
  
  wire req_cpu_lcd_address_matched;
    
  reg fml_load_req_cpu_data_address;
  reg fml_set_cpu_datamem_valid;
  reg fml_rst_cpu_datamem_valid;
  
  reg fml_rst_cpu_datamem_dirty;
  
  reg fml_load_req_lcd_data_address;
  reg fml_set_lcd_datamem_valid;
  reg fml_rst_lcd_datamem_valid;
  
  reg [2:0] fml_burst_count;
  reg fml_rst_burst_count;
  reg fml_next_burst_count;
  
  reg fml_write_cpu_datamem;
  reg fml_write_lcd_datamem;
  
  // CPU Continuous assignments
  assign cpu_data_op = cpu_stb_i & cpu_cyc_i;
  assign cpu_req_address = cpu_adr_i[fml_depth-3-1:4];
  assign cpu_req_offset  = cpu_adr_i[ 3:1];
  
  assign req_cpu_address_matched_cpu = (cpu_req_address == cpu_data_address);
  
  assign cpu_set_cpu_datamem_dirty = cpu_data_op & cpu_datamem_valid & cpu_we_i;
  
  assign req_new_cpu_address = cpu_data_op & !(cpu_datamem_valid & req_cpu_address_matched_cpu);
  
  assign cpu_datamem_wb_ack = cpu_data_op & cpu_datamem_valid & req_cpu_address_matched_cpu;  
  
  // LCD Continuous assignments
  assign lcd_data_op = lcd_stb_i & lcd_cyc_i;
  assign lcd_req_address = lcd_adr_i[fml_depth-3-1:4];
  assign lcd_req_offset  = lcd_adr_i[ 3:1];
  
  assign req_lcd_address_matched_lcd = (lcd_req_address == lcd_data_address);
  assign req_lcd_address_matched_cpu = (lcd_req_address == cpu_data_address);
  assign req_lcd_cpu_address_matched = (lcd_req_address == cpu_req_address);  
  
  assign lcd_datamem_wb_ack = lcd_data_op & ( (lcd_datamem_valid & req_lcd_address_matched_lcd) |
                                              (cpu_datamem_valid & req_lcd_address_matched_cpu)   );
  
  assign req_new_lcd_address = lcd_data_op & !( (lcd_datamem_valid & req_lcd_address_matched_lcd) |
                                                (cpu_datamem_valid & req_lcd_address_matched_cpu) |
                                                (cpu_data_op & req_lcd_cpu_address_matched)        );
                               
    
  // FML Arbritrer Continuous assignments
      
  // Only the cpu data will ever be written to sdram
  assign fml_do = cpu_datamem_valid ? 16'b0 : cpu_datamem_do;
    
  assign fml_sel_o = 2'b11;  
  
  // cpu_datamem Continuous assignments
  assign cpu_datamem_a  = cpu_datamem_wb_ack ? (cpu_data_op ? cpu_req_offset : 3'b0)
                                             : fml_burst_count;
  assign cpu_datamem_we = cpu_datamem_wb_ack ? ( cpu_data_op ? (cpu_we_i ? cpu_sel_i : 2'b0)
                                                             : 2'b0 )
                                             : (fml_write_cpu_datamem ? fml_sel_o : 2'b0);
  assign cpu_datamem_di = cpu_datamem_wb_ack ? (cpu_data_op ? cpu_dat_i : 16'b0)
                                             : fml_di;
                                            
  assign cpu_dat_do_mux = cpu_datamem_wb_ack ? { (cpu_sel_i[1] ? cpu_datamem_do[15:8] : 8'b0),
                                                 (cpu_sel_i[0] ? cpu_datamem_do[7:0] : 8'b0) }
                                             : 16'b0;
  // assign cpu_dat_o      = cpu_datamem_valid ? cpu_datamem_do : 16'b0;
  assign cpu_dat_o      = cpu_datamem_wb_ack ? cpu_dat_do_mux : 16'b0;
                                              
  assign cpu_datamem_a2 = lcd_datamem_wb_ack ? (lcd_data_op ? lcd_req_offset : 3'b0)
                                             : 3'b0;  
  
  vga_arb_datamem #(
	.depth(3)
  ) cpu_datamem (
	  .sys_clk(clk_i),
	
	  .a(cpu_datamem_a),
	  .we(cpu_datamem_we),
	  .di(cpu_datamem_di),
	  .dout(cpu_datamem_do),

	  .a2(cpu_datamem_a2),
	  .do2(cpu_datamem_do2)
  );

  
  // lcd_datamem Continuous assignments
  assign lcd_datamem_a  = lcd_datamem_wb_ack ? (lcd_data_op ? lcd_req_offset : 3'b0 )
                                             : fml_burst_count;
  assign lcd_datamem_we = lcd_datamem_wb_ack ? 2'b0 : (fml_write_lcd_datamem ? fml_sel_o : 2'b0); 
  assign lcd_datamem_di = lcd_datamem_wb_ack ? 16'b0 : fml_di;
  
  assign lcd_dat_op_mux = lcd_datamem_wb_ack ? { (lcd_sel_i[1] ? lcd_datamem_do[15:8] : 8'b0),
                                                 (lcd_sel_i[0] ? lcd_datamem_do[7:0] : 8'b0) }
                                             : 16'b0;
  assign cpu_dat2_op_mux = lcd_datamem_wb_ack ? { (lcd_sel_i[1] ? cpu_datamem_do2[15:8] : 8'b0),
                                                  (lcd_sel_i[0] ? cpu_datamem_do2[7:0] : 8'b0) }
                                              : 16'b0;
  assign lcd_dat_o       = req_lcd_address_matched_cpu ? cpu_dat2_op_mux : lcd_dat_op_mux;  
  
  vga_arb_datamem #(
	  .depth(3)
  ) lcd_datamem (
	  .sys_clk(clk_i),
	
	  .a(lcd_datamem_a),
	  .we(lcd_datamem_we),
	  .di(lcd_datamem_di),
	  .dout(lcd_datamem_do),

	  // .a2(lcd_datamem_a2),
	  // .do2(lcd_datamem_do2)
	  .a2(3'b0),
	  .do2()
  );    
    
  // CPU WB Read/Write Behaviour
  // CPU Reset/Set cpu_datamem_dirty
  always @(posedge clk_i)
    if (rst_i)
      cpu_datamem_dirty <= 1'b0;
    else    
      if (fml_rst_cpu_datamem_dirty)
        cpu_datamem_dirty <= 1'b0;  // Reset has priority!!
      else
        if (cpu_set_cpu_datamem_dirty)
          cpu_datamem_dirty <= 1'b1;   
    
  // CPU Acknowledge cpu_data Read/Write
  always @(posedge clk_i)
    if (rst_i)
      cpu_ack_o <= 1'b0;
    else
      if (cpu_datamem_wb_ack)
        cpu_ack_o <= cpu_datamem_wb_ack & ~cpu_ack_o; 
      
  // LCD Read Behaviour  
  // LCD Acknowledge lcd_data Read
  always @(posedge clk_i)
    if (rst_i)
      lcd_ack_o <= 1'b0;
    else
      if (lcd_datamem_wb_ack)
        lcd_ack_o <= lcd_datamem_wb_ack & ~lcd_ack_o;
          
  // FML Behaviour
  // FML Burst Count
  always @(posedge clk_i)
    if (rst_i)
      fml_burst_count <= 3'b0;
    else
      if (fml_rst_burst_count)
        fml_burst_count <= 3'b0;
      else
        if (fml_next_burst_count)
          fml_burst_count <= fml_burst_count + 1'b1;
  
  // Reset/Set CPU Data valid bit
  always @(posedge clk_i)
    if (rst_i)
      cpu_datamem_valid <= 1'b0;
    else
      if (fml_rst_cpu_datamem_valid)
        cpu_datamem_valid <= 1'b0;
      else
        if (fml_set_cpu_datamem_valid)
          cpu_datamem_valid <= 1'b1;
  
  // Reset/Set LCD Data valid bit
  always @(posedge clk_i)
    if (rst_i)
      lcd_datamem_valid <= 1'b0;
    else
      if (fml_rst_lcd_datamem_valid)
        lcd_datamem_valid <= 1'b0;
      else
        if (fml_set_lcd_datamem_valid)
          lcd_datamem_valid <= 1'b1;
          
  // Load cpu and lcd address
  always @(posedge clk_i)
    if (rst_i)
      begin
          cpu_data_address <= 20-6'b0;
          lcd_data_address <= 20-6'b0;
      end
    else
      if (fml_load_req_cpu_data_address)
        cpu_data_address <= cpu_req_address;
      else
        if (fml_load_req_lcd_data_address)
          lcd_data_address <= lcd_req_address;    
  
  // FML Read/Write FSM
  reg [4:0] state;
  reg [4:0] next_state;

  localparam IDLE         = 5'd0;
  
  localparam CPU_EVICT    = 5'd1;
  localparam CPU_EVICT2   = 5'd2;
  localparam CPU_EVICT3   = 5'd3;
  localparam CPU_EVICT4   = 5'd4;
  localparam CPU_EVICT5   = 5'd5;
  localparam CPU_EVICT6   = 5'd6;
  localparam CPU_EVICT7   = 5'd7;
  localparam CPU_EVICT8   = 5'd8;

  localparam CPU_REFILL   = 5'd9;
  localparam CPU_REFILL2  = 5'd10;
  localparam CPU_REFILL3  = 5'd11;
  localparam CPU_REFILL4  = 5'd12;
  localparam CPU_REFILL5  = 5'd13;
  localparam CPU_REFILL6  = 5'd14;
  localparam CPU_REFILL7  = 5'd15;
  localparam CPU_REFILL8  = 5'd16;
  
  localparam LCD_REFILL   = 5'd17;
  localparam LCD_REFILL2  = 5'd18;
  localparam LCD_REFILL3  = 5'd19;
  localparam LCD_REFILL4  = 5'd20;
  localparam LCD_REFILL5  = 5'd21;
  localparam LCD_REFILL6  = 5'd22;
  localparam LCD_REFILL7  = 5'd23;
  localparam LCD_REFILL8  = 5'd24;

always @(posedge clk_i) begin
	if(rst_i) begin
	    state <= IDLE;	    
	end		
	else begin
		// $display("state: %d -> %d", state, next_state);
		state <= next_state;
	end
end

always @(*) begin

    fml_set_cpu_datamem_valid = 1'b0;
    fml_rst_cpu_datamem_valid = 1'b0;
  
    fml_rst_cpu_datamem_dirty = 1'b0;
  
    fml_set_lcd_datamem_valid = 1'b0;
    fml_rst_lcd_datamem_valid = 1'b0;
  
    fml_load_req_cpu_data_address = 1'b0;
    fml_load_req_lcd_data_address = 1'b0;
    
    fml_rst_burst_count = 1'b0;
    fml_next_burst_count = 1'b0;
    
    fml_write_cpu_datamem = 1'b0;
    fml_write_lcd_datamem = 1'b0;
	
	fml_adr_o = 20'b0;
	fml_stb_o = 1'b0;
	fml_we_o  = 1'b0;
	
	next_state = state;
	
	case(state)
		IDLE: begin
		  if (req_new_lcd_address)
		    begin
		      // Prevent data change during lcd refill phase
		      fml_rst_lcd_datamem_valid = 1'b1;		      
		      fml_load_req_lcd_data_address = 1'b1;		      		      
		      next_state = LCD_REFILL; 
		    end
		  else
		    if(req_new_cpu_address)
		      begin
		        // Prevent data change during cpu evict/refill phase
		        fml_rst_cpu_datamem_valid = 1'b1;		        		        
		        if(cpu_datamem_dirty) begin		        
		          fml_next_burst_count = 1'b1;
		          next_state = CPU_EVICT;
		        end
		        else
		          begin
		            $display("fml_burst_count = %d", fml_burst_count);
		            //fml_next_burst_count = 1'b1; // not sure
		            //fml_write_cpu_datamem = 1'b1;		            
		            fml_load_req_cpu_data_address = 1'b1;
		            next_state = CPU_REFILL;		                
		          end		          
		      end
		end
		
		CPU_EVICT: begin
		  // First write back active data to sdram
		  fml_adr_o = { cpu_data_address, 3'b0};
		  fml_we_o = 1'b1;		  
		  fml_stb_o = 1'b1;
		  if(fml_ack_i)
		    begin
		      fml_next_burst_count = 1'b1;
		      next_state = CPU_EVICT2;    
		    end		      
		end
		CPU_EVICT2: begin
		  fml_next_burst_count = 1'b1;
		  next_state = CPU_EVICT3;
		end
		CPU_EVICT3: begin
		  fml_next_burst_count = 1'b1;
		  next_state = CPU_EVICT4;
		end
		CPU_EVICT4: begin
		  fml_next_burst_count = 1'b1;
		  next_state = CPU_EVICT5;
		end
		CPU_EVICT5: begin
		  fml_next_burst_count = 1'b1;
		  next_state = CPU_EVICT6;
		end
		CPU_EVICT6: begin
		  fml_next_burst_count = 1'b1;
		  next_state = CPU_EVICT7;
		end
		CPU_EVICT7: begin
		  fml_next_burst_count = 1'b1;
		  next_state = CPU_EVICT8;
		end
		CPU_EVICT8: begin
		  fml_load_req_cpu_data_address = 1'b1;
		  fml_rst_burst_count = 1'b1;		  
		  next_state = CPU_REFILL;
		end
		
		CPU_REFILL: begin		
		  // Write the tag first		  
		  if(cpu_we_i)
		    fml_rst_cpu_datamem_dirty = 1'b0;		    
		  else
		    fml_rst_cpu_datamem_dirty = 1'b1;		    		    
		  // Now we can read new data from sdram
		  fml_adr_o = { cpu_req_address, 3'b0 };
		  fml_write_cpu_datamem = 1'b1;
		  fml_stb_o = 1'b1;
		  if(fml_ack_i)
		    begin
		      fml_write_cpu_datamem = 1'b1;
		      next_state = CPU_REFILL2;    
		    end		    		  
		end
		CPU_REFILL2: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_cpu_datamem = 1'b1;
		  next_state = CPU_REFILL3;
		end
		CPU_REFILL3: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_cpu_datamem = 1'b1;
		  next_state = CPU_REFILL4;
		end
		CPU_REFILL4: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_cpu_datamem = 1'b1;
		  next_state = CPU_REFILL5;
		end
		CPU_REFILL5: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_cpu_datamem = 1'b1;
		  next_state = CPU_REFILL6;
		end
		CPU_REFILL6: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_cpu_datamem = 1'b1;
		  next_state = CPU_REFILL7;
		end
		CPU_REFILL7: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_cpu_datamem = 1'b1;
		  next_state = CPU_REFILL8;
		end
		CPU_REFILL8: begin		  		  
		  // We can now read/write the new loaded data
		  fml_set_cpu_datamem_valid = 1'b1;
		  fml_write_cpu_datamem = 1'b1;
		  fml_rst_burst_count = 1'b1; // We will always assume counter starts at zero!!
		  next_state = IDLE;
		end
		
		LCD_REFILL: begin
		  // Load new requested lcd data from sdram
		  fml_adr_o = { lcd_req_address, 3'b0 };
		  fml_write_lcd_datamem = 1'b1;
		  fml_stb_o = 1'b1;
		  if(fml_ack_i)
		    begin
		      fml_next_burst_count = 1'b1;
		      fml_write_lcd_datamem = 1'b1;
		      next_state = LCD_REFILL2;    
		    end		    		  
		end
		LCD_REFILL2: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_lcd_datamem = 1'b1;
		  next_state = LCD_REFILL3;
		end
		LCD_REFILL3: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_lcd_datamem = 1'b1;
		  next_state = LCD_REFILL4;
		end
		LCD_REFILL4: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_lcd_datamem = 1'b1;
		  next_state = LCD_REFILL5;
		end
		LCD_REFILL5: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_lcd_datamem = 1'b1;
		  next_state = LCD_REFILL6;
		end
		LCD_REFILL6: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_lcd_datamem = 1'b1;
		  next_state = LCD_REFILL7;
		end
		LCD_REFILL7: begin
		  fml_next_burst_count = 1'b1;
		  fml_write_lcd_datamem = 1'b1;
		  next_state = LCD_REFILL8;
		end
		LCD_REFILL8: begin		  		  
		  // We can now read the new loaded data
		  fml_set_lcd_datamem_valid = 1'b1;
		  fml_write_lcd_datamem = 1'b1;
		  fml_rst_burst_count = 1'b1; // We will always assume counter starts at zero!!
		  next_state = IDLE;
		end
	endcase
end

endmodule
