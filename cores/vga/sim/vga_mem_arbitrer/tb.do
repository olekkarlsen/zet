quit -sim

if {[file exists work]} {
  vdel -all -lib work
}

vlib work
vlog -work work -lint ../../rtl/vga_arb_datamem.v
vlog -work work -lint tb_vga_mem_arbitrer.v
vlog -work work -lint ../../rtl/vga_mem_arbitrer.v

vsim -novopt -t ps work.tb_vga_mem_arbitrer

add wave -divider test
add wave -hex *

add wave -divider vga_mem_arbitrer
add wave -hex dut/*

run 3us
