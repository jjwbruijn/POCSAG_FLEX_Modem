#include "BCH3121.h"

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

/*
 *   This is a BCH(31,21) implementation, designed for POCSAG
 *   Based on double-error-correcting method for binary BCH codes
 *
 *   More info (math base):
 *   https://web.stanford.edu/class/ee387/handouts/notes16.pdf
 *   https://www.ece.jhu.edu/~cooper/ERROR_CONTROL_CODING/06dec.pdf
*/

// Polynomial form of alpha^i, GF(2^5), using irreducible polynomial: p(X) = X^5 + X^2 + 1
// See http://pnrsolution.org/Datacenter/Vol3/Issue2/250.pdf (Table I)
const int8_t alpha_to[] = {1, 2, 4, 8, 16, 5, 10, 20, 13, 26,
                           17, 7, 14, 28, 29, 31, 27, 19, 3, 6,
                           12, 24, 21, 15, 30, 25, 23, 11, 22, 9,
                           18, 0};

// Index of alpha^i:
const int8_t index_of[] = {-1, 0, 1, 18, 2, 5, 19, 11, 3, 29,
                           6, 27, 20, 8, 12, 23, 4, 10, 30, 17,
                           7, 22, 28, 26, 21, 25, 9, 16, 13, 14,
                           24, 15};

#define POCSAG_DATA_MASK 0xFFFFF800;
#define POCSAG_BCH_N 31
#define POCSAG_BCH_K 21

// Polynomial generator g(X) = X^10 + X^9 + X^8 + X^6 + X^5 + X3 + 1
#define POCSAG_BCH_GENPOLY 0x00000769U

CBCH3121::CBCH3121() : m_S1(0U),
                       m_S3(0U) {
}

void CBCH3121::encode(uint32_t& data) {
    // Just use data part of the codeword
    data &= POCSAG_DATA_MASK;

    uint32_t tmp = data;
    uint32_t gen_poly = POCSAG_BCH_GENPOLY << POCSAG_BCH_K;

    // Calculate BCH check bits
    for (unsigned int i = 0U; i < POCSAG_BCH_K; i++, tmp <<= 1) {
        if (tmp & 0x80000000U)
            tmp ^= gen_poly;
    }

    // Add BCH check bits
    data |= (tmp >> POCSAG_BCH_K);

    // Add parity bit
    if (calc_parity(data))
        data |= 0x01U;
}

bool CBCH3121::decode(uint32_t& data, uint16_t& errors) {
    int8_t S1_3, tmp, X1, X2;
    uint8_t cnt = 0U;
    uint8_t Q = 0U;
    uint8_t locator[5U];
    uint32_t copy = data;

    // Calculate syndrome
    calc_syndrome(data);

    if (m_S1 == -1 && m_S3 == -1) {
        // return if no errors
        errors += check_parity(data);
        return true;
    } else {
        // Calculate S1^3 in GF(2^5):
        S1_3 = (m_S1 * 3) % POCSAG_BCH_N;

        // Check for single error
        if (S1_3 == m_S3) {
            // Correct single error
            data ^= (0x01U << (m_S1 + 1U));
            errors += check_parity(data) + 1U;
            return true;
        } else {  // More than 1 errors

            // Calculate in GF(2^5):
            // X1 = S1
            // X2 = (S3 + S1^3) / S1

            X1 = m_S1;

            if (m_S3 != -1)
                tmp = alpha_to[S1_3] ^ alpha_to[m_S3];
            else
                tmp = alpha_to[S1_3];

            X2 = (index_of[tmp] - m_S1 + POCSAG_BCH_N) % POCSAG_BCH_N;

            // Chien search
            for (uint8_t i = 1U; i <= POCSAG_BCH_N; i++) {
                Q = 1U;

                if (X1 != -1) {
                    X1 = (X1 + 1) % POCSAG_BCH_N;
                    Q ^= alpha_to[X1];
                }

                if (X2 != -1) {
                    X2 = (X2 + 2) % POCSAG_BCH_N;
                    Q ^= alpha_to[X2];
                }

                if (!Q) {
                    locator[cnt] = i % POCSAG_BCH_N;
                    cnt++;
                }
            }

            if (cnt != 2U) {  // Decoding failed, too many errors
                errors += check_parity(data) + 3U;
                data = copy;
                return false;
            }

            // Correct double error:
            data ^= (0x80000000U >> (locator[0U] - 1U));
            data ^= (0x80000000U >> (locator[1U] - 1U));

            errors += check_parity(data) + 2U;

            return true;
        }
    }
    data = copy;
    return false;
}

void CBCH3121::calc_syndrome(uint32_t data) {
    // Reset syndromes
    m_S1 = 0;
    m_S3 = 0;

    // Drop parity bit
    data >>= 1;

    // Calculate just syndromes S1 and S3
    for (uint8_t i = 0; i < POCSAG_BCH_N; i++, data >>= 1) {
        if (data & 0x01U) {
            m_S1 ^= alpha_to[i % POCSAG_BCH_N];
            m_S3 ^= alpha_to[(3 * i) % POCSAG_BCH_N];
        }
    }

    // Transform polynomial form to index form
    m_S1 = index_of[m_S1];
    m_S3 = index_of[m_S3];
}

bool CBCH3121::calc_parity(uint32_t data) {
    data ^= data >> 16;
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;

    return data & 0x01U;
}

uint16_t CBCH3121::check_parity(uint32_t& data) {
    // Check for error in parity bit
    if (calc_parity(data)) {
        data ^= 0x01;
        return 1U;
    } else
        return 0U;
}