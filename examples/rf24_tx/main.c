#include <qm_common.h>
#include <clk.h>
#include <stdint.h>
#include "rf24/rf24.h"

/* ------------------------------------------------------------------------- */
int main()
{
	uint8_t payload[3];

	QM_PRINTF("Simple nRF24L01 transmit example\r\n");

#if defined(RF24_SPI_MULTIBYTE)
	QM_PRINTF("Using multibyte SPI transfers\r\n");
#else
	QM_PRINTF("Using single byte SPI transfers\r\n");
#endif

	if (rf24_init()) {
		QM_PRINTF("Failed to initialize nRF24L01\r\n");
	} else {
		QM_PRINTF("Initialized nRF24L01 radio. ");
		if (rf24_is_plus()) {
			QM_PRINTF("Detected nRF24L01+\r\n");
		} else {
			QM_PRINTF("Detected nRF24L01\r\n");
		}
	}


    rf24_set_retries(15,15);

    rf24_open_writing_pipe_uint64(0xAA55AA55AA);

    rf24_stop_listening();

    clk_sys_udelay(1000);

    uint8_t counter = 0;
    while(1) {
    	payload[0]= counter;
    	payload[1]= counter + 1;
    	payload[2]= counter + 2;

    	QM_PRINTF("Now sending ... %d,%d,%d\r\n",payload[0],payload[1],payload[2]);

    	rf24_write(payload,3);

    	clk_sys_udelay(1000000);

    	if (counter < 10) {
    		counter++;
    	} else {
    		counter = 0;
    	}

#if !defined (RF24_MINIMAL)
    	rf24_print_details();
#endif

    }

    return 0;
}
