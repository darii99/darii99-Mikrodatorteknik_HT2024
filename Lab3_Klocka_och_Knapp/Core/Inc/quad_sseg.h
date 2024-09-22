/*
 * quad_sseg.h
 *
 * For this module to work the pins SEG_DIO and SEG_CLK
 * needs to be defined (done with user labels in CubeMX).
 *
 */

#ifndef SRC_QUAD_SSEG_H_
#define SRC_QUAD_SSEG_H_

// To be used as values for qs_put_digits().
// Look in 'seg_array' for more details.
#define QS_BLANK 16
#define QS_MINUS 17



/************************************************************
 * Put your own bitmask at digit at a certain location.
 *
 * The number inside of the squares in this image is the
 * valid location numbers.
 * To set the colon, have the MSB be set to 1 when writing
 * to location 1.
 *
 *          #####  #####   #####  #####
 *          #   #  #   # o #   #  #   #
 *          # 0 #  # 1 #   # 2 #  # 3 #
 *          #   #  #   # o #   #  #   #
 *          #####  #####   #####  #####
 *
 ************************************************************/
void qs_put_bitmask_at(
		uint8_t bitmask,
		uint8_t loc);


/************************************************************
 * To set the colon sign make sure MSB(bm1) == 1.
 * See comment for qs_put_bitmask_at().
 *
 ************************************************************/
void qs_put_bitmasks(
		uint8_t bm0,
		uint8_t bm1,
		uint8_t bm2,
		uint8_t bm3);



/************************************************************
 * Put four digits on display and set the colon.
 *
 * Accepts 0x00 to 0x0F as values. The constants
 * QS_BLANK and QS_MINUS also works.
 *
 * Examples:
 *
 * 		qs_put_digits(1, 2, 3, 4, 1)
 * 					====> "12:34"
 *
 * 		qs_put_digits(2, 3, 5, 9, 0)
 * 					====> "23:59"
 *
 * 		qs_put_digits(0xB, 0xA, 0xB, 0xE, 0)
 * 					====> "bA bE"
 *
 * 		qs_put_digits(QS_MINUS, QS_BLANK, 2, 0, 0)
 * 					====> "-  20"
 *
 * 		qs_put_digits(QS_BLANK, QS_MINUS, QS_MINUS QS_BLANK, 1)
 * 					====> " -:- "
 *
 ************************************************************/
void qs_put_digits(
		uint8_t d0,
		uint8_t d1,
		uint8_t d2,
		uint8_t d3,
		uint8_t colon);

/************************************************************
 *	Displays a big number, with leading blanks.
 ************************************************************/
void qs_put_big_num(uint16_t big_num);


#endif /* SRC_QUAD_SSEG_H_ */
