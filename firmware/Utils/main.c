
#include <16f688.h>
#use delay(clock=8000000)


#fuses INTRC_IO, NOPROTECT, NOBROWNOUT, NOMCLR, NOWDT, PUT, 
int16 TA,VADC;
int32 TB;


void main(void) {
	setup_oscillator( OSC_8MHZ );
	setup_comparator(NC_NC_NC_NC);
	setup_adc(ADC_CLOCK_INTERNAL );
	
 	output_low(pin_c4);
 	output_low(pin_c3);

	for (;;){
		set_adc_channel(6);
		delay_us(10);
		VADC = read_adc();
		if (VADC<26) VADC=26;
		//VADC=255;
		TB=255750/VADC;
		TA=(int16)TB;
		


	//	TA=1000;
		output_high(pin_c3); // 1
		delay_us(TA);
		output_low(pin_c3); 
		delay_us(3*TA);
	 	output_high(pin_c4); 
		delay_us(TA);	
		output_low(pin_c4);
	
		delay_us(5*TA); // 2
		delay_us(5*TA); // 3
		delay_us(5*TA); // 4
		delay_us(5*TA); // 5
		delay_us(5*TA); // 6
		delay_us(5*TA); // 7
	//	delay_us(3*TA); // 8

    }

}