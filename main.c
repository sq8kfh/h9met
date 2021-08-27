/*
 * h9met
 * Weather station
 *
 * Created by SQ8KFH on 2021-08-04.
 *
 * Copyright (C) 2021 Kamil Palkowski. All rights reserved.
 */

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "can/can.h"

#define SUB_TIMER_PRESCALER 1200
uint16_t sub_timer = SUB_TIMER_PRESCALER;

#define SPACE_SIZE 21
#define MARK_SIZE 39

#define HDLC_FLAG ((uint8_t)0x7e)
#define PREAMBLE_LENGTH 11

uint8_t space_idx = 0;
uint8_t mark_idx = 0;

uint8_t mark[MARK_SIZE] = {127, 147, 167, 186, 203, 218, 231, 242, 249, 253, 254, 252, 246, 237, 226, 212, 195, 177, 158, 138, 117, 97, 78, 60, 43, 29, 17, 9, 3, 0, 1, 5, 12, 22, 35, 50, 67, 86, 105};
//uint8_t mark[MARK_SIZE] = {127, 140, 152, 164, 175, 185, 193, 199, 204, 206, 207, 205, 202, 196, 189, 180, 170, 159, 147, 134, 121, 108, 96, 85, 74, 65, 58, 52, 49, 47, 48, 50, 54, 61, 69, 78, 89, 101, 113};
uint8_t space[SPACE_SIZE] = {127, 164, 198, 225, 244, 253, 252, 239, 217, 186, 151, 114, 78, 46, 21, 5, 0, 6, 22, 47, 79};
uint8_t mark2space[MARK_SIZE] = {0,0,0,1,2,2,3,4,4,4,5,6,6,7,7,8,8,9,9,10,11,11,12,12,13,13,14,14,15,16,16,17,17,18,18,19,19,20,20};
uint8_t space2mark[SPACE_SIZE] = {0,2,3,5,7,8,10,12,14,16,18,20,22,24,26,27,29,31,33,34,36};

volatile uint8_t mark_space = 0;

uint16_t hdlc_crc = 0xFFFF;

uint8_t data_idx = 0;
uint8_t bit_idx = 0;
#define DATA_LENGHT 57
uint8_t data_buf[] = {'A' << 1, 'P' << 1, 'R' << 1, 'S' << 1, ' ' << 1, ' ' << 1, 0b01110000 << 1, 'S' << 1, 'Q' << 1, '8' << 1, 'K' << 1, 'F' << 1, 'H' << 1, 0b00110000 << 1, 'W' << 1, 'I' << 1, 'D' << 1, 'E' << 1, '2' << 1, ' ' << 1, '1' << 1 | 0x01, 0x03, 0xF0, '/', '2', '1', '0', '0', '4', '8', 'h', '4', '9', '1', '6', '.', '5', '4', 'N', '/', '0', '1', '8', '1', '4', '.', '5', '8', 'E', 'O', 'T', 'T', '7', 'F', 'h', 'a', 'b'};

uint8_t bit1counter = 0;

uint8_t transmiter_state = 1; // 0 - stop
                              // 1 - preamble
                              // 2 - data
                              // 3 - crc
                              // 4 - flag

#define lo8(x) ((x)&0xff)
#define hi8(x) ((x)>>8)
uint16_t crc_ccitt_update(uint16_t crc, uint8_t data) {
    data ^= lo8 (crc);
    data ^= data << 4;

    return ((((uint16_t)data << 8) | hi8 (crc)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

ISR(TIMER0_COMP_vect) {
    if (!transmiter_state) { //brak transmisji
        OCR0A = 127;
        return;
    }
    if (mark_space) {
        ++mark_idx;
        if (mark_idx >= MARK_SIZE) mark_idx=0;
        OCR0A = mark[mark_idx];
    }
    else {
        ++space_idx;
        if (space_idx >= SPACE_SIZE) space_idx=0;
        OCR0A = space[space_idx];
    }
}

ISR(TIMER1_COMPA_vect) {
    //SUB_TIMER
    if (--sub_timer == 0) {
        PORTE |= (1 << PE7); //wind speed ICP
        sub_timer = SUB_TIMER_PRESCALER;
    }


    if (transmiter_state == 2) {   //data
        if (!((data_buf[data_idx] >> bit_idx) & 0x01) || (bit1counter == 5) ) { //bit: 0
            if (mark_space) {
                space_idx = mark2space[mark_idx];
                mark_space=0;
            }
            else {
                mark_idx = space2mark[space_idx];
                mark_space = 1;
            }
            bit1counter = 0;
        }
        else {
            ++bit1counter;
        }

        if(bit1counter != 5) {
            ++bit_idx;
            if (bit_idx >= 8) {
                bit_idx = 0;

                hdlc_crc = crc_ccitt_update(hdlc_crc, data_buf[data_idx]);

                ++data_idx;
                if (data_idx >= DATA_LENGHT) {
                    bit_idx = 0;
                    data_idx = 0;

                    hdlc_crc = ~hdlc_crc;  //flip the bits
                    transmiter_state = 3;
                }
            }
        }
    }
    else if (transmiter_state == 3) { //crc
        if (!((hdlc_crc >> bit_idx) & 0x01) || (bit1counter == 5) ) { //bit: 0
            if (mark_space) {
                space_idx = mark2space[mark_idx];
                mark_space=0;
            }
            else {
                mark_idx = space2mark[space_idx];
                mark_space = 1;
            }
            bit1counter = 0;
        }
        else {
            ++bit1counter;
        }

        if(bit1counter != 5) {
            ++bit_idx;
            if (bit_idx >= 16) {
                bit_idx = 0;
                transmiter_state = 4;
            }
        }
    }
    else if (transmiter_state == 1 || transmiter_state == 4) {
        if (!((HDLC_FLAG >> bit_idx) & 0x01)) { //bit: 0
            if (mark_space) {
                space_idx = mark2space[mark_idx];
                mark_space=0;
            }
            else {
                mark_idx = space2mark[space_idx];
                mark_space = 1;
            }
        }
        ++bit_idx;
        if (bit_idx >= 8) {
            bit_idx = 0;
            ++data_idx;

            if (transmiter_state == 4 && data_idx >= 3) {
                bit_idx = 0;
                data_idx = 0;
                bit1counter = 0;
                space_idx = SPACE_SIZE;
                mark_idx = MARK_SIZE;
                transmiter_state = 0;
            }
            else if (data_idx >= PREAMBLE_LENGTH) {
                bit_idx = 0;
                data_idx = 0;
                bit1counter = 0;
                hdlc_crc = 0xFFFF;
                transmiter_state = 2;
            }
        }
    }
}

int main(void) {
	DDRB = 0xff;
	DDRC = 0xff;
	DDRD = 0xff;
	DDRE = 0xff;
	
	CAN_init();

    ADCSRA = (1<<ADEN) //ADEN=1 włączenie przetwornika ADC)
             |(1<<ADPS0) // ustawienie preskalera na 128
             |(1<<ADPS1)
             |(1<<ADPS2);

    ADMUX  =    (1<<REFS1) | (1<<REFS0); // REFS1:0: wybór napięcia odniesienia ADC
                                         //na wewnętrzne źródło 2,56V
                                         //z zewnętrznym kondensatorem na pinie AREF

    //timer
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11); //CTC, clk/8
    OCR1A = 1250; //overflow, 1200Hz
    TIMSK1 = (1 << OCIE1A);

    //wind speed
    DDRE &= ~(1 << PE6); //T3
    PORTE |= (1 << PE6);
    DDRE |= (1 << PE7); //ICP
    PORTE &= ~(1 << PE7);
    TCCR3A = 0;
    TCCR3B = (1 << CS32) | (1 << CS31) | (1 << ICES3); //T3 pin falling edge, Input capture rising edge
    TIFR3 = (1 << ICF3);

    TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (1 << CS00);
    OCR0A = space[space_idx];
    TIMSK0 = (1 << OCIE0A);
    sei();

    CAN_send_turned_on_broadcast();
    uint32_t adc_timer = 0;

    while (1) {
		h9msg_t cm;
		int can_get_ret = CAN_get_msg(&cm);
		if (can_get_ret == 1) {
		    if (cm.type == H9MSG_TYPE_GET_REG) {
				h9msg_t cm_res;
				CAN_init_response_msg(&cm, &cm_res);
				cm_res.data[0] = cm.data[0];
				switch (cm.data[0]) {
					case 11:
                        cm_res.dlc = 3;
                        CAN_put_msg(&cm_res);
						break;
                    default:
                        cm_res.type = H9MSG_TYPE_ERROR;
                        cm_res.data[0] = H9MSG_ERROR_INVALID_REGISTER;
                        cm_res.dlc = 1;
                        CAN_put_msg(&cm_res);
				}
			}
			else if (cm.type == H9MSG_TYPE_SET_REG) {
				h9msg_t cm_res;
				CAN_init_response_msg(&cm, &cm_res);
				cm_res.data[0] = cm.data[0];
				switch (cm.data[0]) {
					case 11:
                        cm_res.dlc = 3;
                        CAN_put_msg(&cm_res);
						break;
                    default:
                        cm_res.type = H9MSG_TYPE_ERROR;
                        cm_res.data[0] = H9MSG_ERROR_INVALID_REGISTER;
                        cm_res.dlc = 1;
                        CAN_put_msg(&cm_res);
				}
			}
            else {
                h9msg_t cm_res;
                CAN_init_response_msg(&cm, &cm_res);
                cm_res.type = H9MSG_TYPE_ERROR;
                cm_res.data[0] = H9MSG_ERROR_INVALID_MSG;
                cm_res.dlc = 1;
                CAN_put_msg(&cm_res);
            }
		}

		////
		if (TIFR3 & (1 << ICF3)) { //1Hz
		    if (transmiter_state == 0) transmiter_state = 1;

		    uint16_t ws = ICR3;
		    PORTE &= ~(1 << PE7);
		    TIFR3 = (1 << ICF3);

		//if (adc_timer == 0) {
		    ADMUX = (1<<REFS0); //ADC0 ref=5V
            ADCSRA |= (1 << ADSC); //ADSC: uruchomienie pojedynczej konwersji
            while (ADCSRA & (1 << ADSC)); //czeka na zakończenie konwersji
            ADCSRA |= (1 << ADSC); //ADSC: uruchomienie pojedynczej konwersji
            while (ADCSRA & (1 << ADSC)); //czeka na zakończenie konwersji
            uint16_t mpx4115 = ADC;

            ADMUX = (1<<REFS0) | (1<<MUX0); //ADC1 ref=5V
            ADCSRA |= (1 << ADSC); //ADSC: uruchomienie pojedynczej konwersji
            while (ADCSRA & (1 << ADSC)); //czeka na zakończenie konwersji
            ADCSRA |= (1 << ADSC); //ADSC: uruchomienie pojedynczej konwersji
            while (ADCSRA & (1 << ADSC)); //czeka na zakończenie konwersji
            uint16_t w200p = ADC;

            ADMUX = (1<<REFS1) | (1<<REFS0) | (1<<MUX1); //ADC2
            ADCSRA |= (1 << ADSC); //ADSC: uruchomienie pojedynczej konwersji
            while (ADCSRA & (1 << ADSC)); //czeka na zakończenie konwersji
            ADCSRA |= (1 << ADSC); //ADSC: uruchomienie pojedynczej konwersji
            while (ADCSRA & (1 << ADSC)); //czeka na zakończenie konwersji
            uint16_t cmp3 = ADC;

            h9msg_t cm_adc;
            CAN_init_new_msg(&cm_adc);
            cm_adc.type = H9MSG_TYPE_REG_VALUE_BROADCAST;
            cm_adc.destination_id = H9MSG_BROADCAST_ID;
            cm_adc.dlc = 7;
            cm_adc.data[0] = 10;
            uint16_t tmp = ADC;
            cm_adc.data[1] = (mpx4115 >> 8) & 0xff;
            cm_adc.data[2] = (mpx4115) & 0xff;
            cm_adc.data[3] = (w200p >> 8) & 0xff;
            cm_adc.data[4] = (w200p) & 0xff;
            cm_adc.data[5] = (cmp3 >> 8) & 0xff;
            cm_adc.data[6] = (cmp3) & 0xff;

            cm_adc.data[5] = (ws >> 8) & 0xff;
            cm_adc.data[6] = (ws) & 0xff;
            CAN_put_msg(&cm_adc);

            adc_timer = 200000;
        }
		--adc_timer;
	}
}
