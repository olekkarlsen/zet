SOURCES=tb_vga_mem_arbitrer.v $(wildcard ../../rtl/*.v)

all: vga_mem_arbitrer

vga_mem_arbitrer: $(SOURCES)
	cver $(SOURCES)

clean:
	rm -f verilog.log

.PHONY: clean vga_mem_arbitrer
