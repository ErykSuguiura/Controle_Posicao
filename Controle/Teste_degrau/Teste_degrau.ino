//#include <limits.h>

#define PWM1 9
#define PWM2 10
#define POTENC A0
#define WAIT 100

/*
 * TIMER_0 -> PWMM 5 e 6
 * TIMER_1 -> PWM 9 e 10
 * TIMER 2 -> PWM 11 e 3
 */


double t = 0;
int k = 0;

//Resposta em frequencia
double f_sin = 0.1; //Hz
double out_sin = 0;

//Amplitude da resposta em frequencia
double A_degrau = 100;
#define NUM_CICLOS_AMOSTRA 20
#define NUM_CICLOS_ACOMODACAO 5
#define FMAX 40
#define FINCREMENT 0.1

//Amostragem de Amplitude
int media_max, media_min, A_out;
int ciclos_absolutos = 0;

//Amostragem de Fase
float potenciometro_data_last=0;
int index_fase=0; 


int index_limite = 0;
int potenciometro_data;
bool ampl_impr = false, fase_impr = false;

#define N_AMOS_FFT 10000

short white_noise_vec[N_AMOS_FFT];
//short out_vec[N_AMOS_FFT];

void setup()
{
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(POTENC, INPUT);
  Serial.begin(9600);
  
  delay(1000);
  Serial.println("kTs,input,out");

  cli();//stop interrupts

//set timer0 interrupt at 2kHz
 TCCR0A = 0;// set entire TCCR0A register to 0
 TCCR0B = 0;// same for TCCR0B
 TCNT0  = 0;//initialize counter value to 0
 // set compare match register for 2khz increments
 OCR0A = 249;// = (16*10^6) / (2000*64) - 1 (must be <256)
 // turn on CTC mode
 TCCR0A |= (1 << WGM01);
 // Set CS01 and CS00 bits for 64 prescaler
 TCCR0B |= (1 << CS01) | (1 << CS00);   
 // enable timer compare interrupt
 TIMSK0 |= (1 << OCIE0A);


  sei();//allow interrupts
}


void entrada_motor(double input)
{
  if(input > 0.)
  {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, abs(input));
  }
  else
  {
    analogWrite(PWM1,  abs(input));
    analogWrite(PWM2, 0); 
  }
      
}

int ciclo_absoluto =0;

int out_potenc;

int pos_last=0;
int input_quad = 80;


ISR(TIMER0_COMPA_vect)
{//timer2 interrupt 8kHz toggles pin 9

  entrada_motor(input_quad);
  out_potenc = analogRead(POTENC);


  k = (k<N_AMOS_FFT)? k+1: 0;

  Serial.print(double(k)/1000., 3);
  Serial.print(",");
  Serial.print(input_quad);
  Serial.print(",");
  //Serial.println((out_potenc-pos_last)*1000);
  Serial.println((out_potenc-pos_last));

  if(k%0 ==0) input_quad*=-1;

  pos_last=out_potenc;
}
bool print_ams = false;


void loop()
{
}
