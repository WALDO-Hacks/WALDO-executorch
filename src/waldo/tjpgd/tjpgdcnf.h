/*----------------------------------------------*/
/* TJpgDec System Configurations R0.03          */
/* (ChaN's TJpgDec, configured for WALDO)       */
/*----------------------------------------------*/

#define	JD_SZBUF		512	/* Size of stream input buffer */
#define JD_FORMAT		0	/* Output pixel format 0:RGB888 */
#define	JD_USE_SCALE	0	/* Switches output descaling feature */
#define JD_TBLCLIP		1	/* Use table conversion for saturation arithmetic */
#define JD_FASTDECODE	2	/* 2: + table conversion for huffman decoding (needs 32-bit MCU, ~6 KB more work area) */
