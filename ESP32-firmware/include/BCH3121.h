#include <Arduino.h>

/*
 *   Copyright (C) 2018 by Andy Uribe CA6JAU
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#if !defined(BCH3121_H)
#define  BCH3121_H

class CBCH3121 {
public:
  CBCH3121();

  void encode(uint32_t& data);
  bool decode(uint32_t& data, uint16_t& errors);

private:
  void     calc_syndrome(uint32_t data);
  bool     calc_parity(uint32_t data);
  uint16_t check_parity(uint32_t& data);

  int8_t  m_S1;
  int8_t  m_S3;

};

#endif