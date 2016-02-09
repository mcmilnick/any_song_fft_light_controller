#include <stdio.h>
#include <avr/io.h>
#include <math.h>
#include <util/delay.h>

#define SAMPLING_RATE 128000  //dependent on ADC prescaler
#define BINS 8
#define BLOCK_SIZE 100

volatile int ADC_flag;
volatile int sig_in_10bit;
int freq_pins[BINS] = {0,1,2,3,4,5,6,7};
float Q0, Q1, Q2;

void setup()
{
  for(uint8_t i = 0; i < BINS; i++)
  {
    pinMode(freq_pins[i], OUTPUT);
  }
  InitADC();
  loop();
}

void loop()
{
  float *coeff_array; //don't need to return the sin, cos for the optimized filter
  uint8_t sig_in_count = 0;
  float mag_square[BINS];
  int ADC_data[BLOCK_SIZE];

  coeff_array = calc_coeffs();

  while (1)
  {
    if (ADC_flag == 1) //ADC flag
    {
      ADC_flag = 0;
      //All ADC data is read in as a percentage of GND to VCC. EX: 5V supply - 5V = 1, 2.5V = .5.
      ADC_data[sig_in_count] = sig_in_10bit;
      sig_in_count++;
    }
    //Timing of whenever is noticable to a human.
    if (sig_in_count == 100)
    {
      sig_in_count = 0; //new_block
      for (uint8_t i = 0; i < BINS; i++) //loops for each center frequency
      {
        Q2 = 0;
        Q1 = 0; //reset vars on block begin
        for (uint8_t j = 0; j < BLOCK_SIZE; j++) //loops for each sample in a block
        {
          get_filter_coefficient(ADC_data[j], coeff_array[i]);
        }
        //Find mag after going through block. Deal with squared for less processing.
        mag_square[i] = GetMagnitudeSquared(coeff_array[i]);
        
        //interpet magnitude of each bin
        (mag_square[i] > 4000000) ? digitalWrite(freq_pins[i], HIGH):
                                    digitalWrite(freq_pins[i], LOW);
      }
    }
  }
}


/*-------------------------------------------------------------------------------------
   Functions pertaining to ADC function
  ------------------------------------------------------------------------------------*/
void InitADC()
{
  //ADLAR shift bit is default right. This is fine for 10 bit
  //ADMUX bottom 4 not set gives A0
  ADCSRA = ADCSRB = 0; //clear regs
  ADMUX |= (1 << REFS0); //set ref to AVcc
  ADCSRA |= (1 << ADATE); //auto trigger bit
  ADCSRA |= (1 << ADIE); //enable interrupt
  ADCSRB &= (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0); //clear bottom 3 for free running mode
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN); //set prescaller to 128kHz and en ADC
  sei(); //global interrupts
  ADC_flag = 0;
  ADCSRA |= (1 << ADSC); //start conversion
}

ISR(ADC_vect)
{
  ADC_flag = 1;
  //Read low register. Locks ADC update until ADCH is read
  sig_in_10bit = ADCL | (ADCH << 8); //10 bit grab. Otherwise no H and shift left.
}


/*-------------------------------------------------------------------------------------
   Functions pertaining to Goertzel filtering
  ------------------------------------------------------------------------------------*/

//Initial calculation of coefficients. There is one for every bin's center freq.
float * calc_coeffs(void)
{
  //the frequency range of interest is 20-5kHz. With 12 bins, we have 416Hz wide bins. We will center on each bin.
  float target_freqs[] = {208, 624, 1040, 1456, 1872, 2288,
                          2704, 3120, 3536, 3952, 4368, 4784
                         };
  float w[12], twok[12], sin_array[12], cos_array[12];
  static float coeff_array[12];

  for (uint8_t bins_num = 0; bins_num < BINS; bins_num++)
  {
    twok[bins_num] = 1 + (2 * ((BINS * target_freqs[bins_num]) / SAMPLING_RATE));
    w[bins_num] = (PI * twok[bins_num]) / BINS;
    sin_array[bins_num] = sin(w[bins_num]);
    cos_array[bins_num] = cos(w[bins_num]);
    coeff_array[bins_num] = 2 * cos_array[bins_num];
  }
  return coeff_array;
}

//Need new coefficient calc for every sample in a block
void get_filter_coefficient(int ADC_data, float coeff)
{
  Q0 = ADC_data + ((float)coeff * Q1) - Q2;
  Q2 = Q1;
  Q1 = Q0;
}

//Optimized algorithm cuts phase info.
float GetMagnitudeSquared(float coeff)
{
  float result = (Q1 * Q1) + (Q2 * Q2) - (Q1 * Q2 * coeff);
  return result;
}


//TBD: get rid of floats. They take to long.
