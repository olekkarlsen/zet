/*
 *  Copyright (c) 2009  Zeus Gomez Marmolejo <zeus@opencores.org>
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

module hex_display (
    input  [31:0] num,
    input         en,

    output [6:0] hex0,
    output [6:0] hex1,
    output [6:0] hex2,
    output [6:0] hex3,
	  output [6:0] hex4,
    output [6:0] hex5,
    output [6:0] hex6,
    output [6:0] hex7
  );

  // Module instantiations
  seg_7 hex_group0 (
    .num (num[3:0]),
    .en  (en),
    .seg (hex0)
  );

  seg_7 hex_group1 (
    .num (num[7:4]),
    .en  (en),
    .seg (hex1)
  );

  seg_7 hex_group2 (
    .num (num[11:8]),
    .en  (en),
    .seg (hex2)
  );

  seg_7 hex_group3 (
    .num (num[15:12]),
    .en  (en),
    .seg (hex3)
  );

  seg_7 hex_group4 (
    .num (num[19:16]),
    .en  (en),
    .seg (hex4)
  );

  seg_7 hex_group5 (
    .num (num[23:20]),
    .en  (en),
    .seg (hex5)
  );
    
  seg_7 hex_group6 (
    .num (num[27:24]),
    .en  (en),
    .seg (hex6)
  );
    
  seg_7 hex_group7 (
    .num (num[31:28]),
    .en  (en),
    .seg (hex7)
  );
endmodule
