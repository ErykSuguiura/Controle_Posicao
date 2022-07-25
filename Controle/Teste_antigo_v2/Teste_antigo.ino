#include <limits.h>

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
double k = 0;

//Resposta em frequencia
double f_sin = 4.25; //Hz
double out_sin = 0;

//Amplitude da resposta em frequencia
double A_sin = 80;
#define NUM_CICLOS_AMOSTRA 100
#define NUM_CICLOS_ACOMODACAO 20
#define FMAX 30
#define FINCREMENT 10

//Amostragem de Amplitude
int maximos[NUM_CICLOS_AMOSTRA]; 
int minimos[NUM_CICLOS_AMOSTRA]; 
int media_max, media_min;
int ciclos_absolutos = 0;

float A_out;

//Amostragem de Fase
float potenciometro_data_last=0;
int fases[NUM_CICLOS_AMOSTRA];
int index_fase=0; 


int index_limite = 0;
int potenciometro_data;
bool ampl_impr = false, fase_impr = false;

void setup()
{
  for(int i = 0; i<NUM_CICLOS_AMOSTRA; i++)
  {
     maximos[i] = 0; 
     minimos[i] = INT_MAX;
     fases[i]   = 0;
  }
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(POTENC, INPUT);
  Serial.begin(9600);
  
  delay(1000);

  cli();//stop interrupts

////set timer0 interrupt at 2kHz
//  TCCR0A = 0;// set entire TCCR0A register to 0
//  TCCR0B = 0;// same for TCCR0B
//  TCNT0  = 0;//initialize counter value to 0
//  // set compare match register for 2khz increments
//  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
//  // turn on CTC mode
//  TCCR0A |= (1 << WGM01);
//  // Set CS01 and CS00 bits for 64 prescaler
//  TCCR0B |= (1 << CS01) | (1 << CS00);   
//  // enable timer compare interrupt
//  TIMSK0 |= (1 << OCIE0A);
//
////set timer1 interrupt at 1Hz
//  TCCR1A = 0;// set entire TCCR1A register to 0
//  TCCR1B = 0;// same for TCCR1B
//  TCNT1  = 0;//initialize counter value to 0
//  // set compare match register for 1hz increments
//  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
//  // turn on CTC mode
//  TCCR1B |= (1 << WGM12);
//  // Set CS10 and CS12 bits for 1024 prescaler
//  TCCR1B |= (1 << CS12) | (1 << CS10);  
//  // enable timer compare interrupt
//  TIMSK1 |= (1 << OCIE1A);

//set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);


  sei();//allow interrupts
  
  
}


void entrada_motor(int input)
{
//  Serial.println(analogRead(POTENC));
  if(input > 0)
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


ISR(TIMER2_COMPA_vect)
{//timer2 interrupt 8kHz toggles pin 9
  out_sin = A_sin*sin(2.*PI* f_sin*k*(1./8000.));
  
  entrada_motor(out_sin); //sen[2*pi*k*Ts] Ts = 1/8k sen[2*pi*0*Ts] = sen[2*pi*(k*Ts)
//  Serial.println(out_sin);
  if(k>=8000/f_sin && index_limite < NUM_CICLOS_AMOSTRA && ciclos_absolutos > NUM_CICLOS_ACOMODACAO) 
  {
    index_limite++;
    index_fase++;
  }

  potenciometro_data =analogRead(POTENC);
  if(index_limite<NUM_CICLOS_AMOSTRA && ciclos_absolutos > NUM_CICLOS_ACOMODACAO)
  {
    
    if(potenciometro_data > maximos[index_limite])
    {

     maximos[index_limite] = potenciometro_data;
      fases[index_fase] = (k-8000/(4*f_sin))*360./(8000/f_sin);
       
    }
    if(potenciometro_data < minimos[index_limite]) minimos[index_limite] = potenciometro_data; 
  }  
  k = (k<8000/f_sin)? k+1 : 0; 

  
    
  // //if(index_fase < NUM_CICLOS_AMOSTRA && float(potenciometro_data_last) < float(media_max)-A_out && float(potenciometro_data) >= float(media_max)-A_out && ampl_impr)//borda de subida
  // {
  //   fases[index_fase] = k*360./(8000/f_sin);
  //   index_fase++;
  // }
  // //Serial.println(index_fase);


  ciclos_absolutos = (ciclos_absolutos<INT_MAX-1)? ciclos_absolutos+1 : INT_MAX;

  potenciometro_data_last = potenciometro_data;
//  Serial.println(k*(1./8000.));
}

//Formatacao da Impressao: f_sin, A_out
void imprimir_ampl()
{
  for(int i = 0; i<NUM_CICLOS_AMOSTRA; i++ )
  {
    media_max += maximos[i];
    media_min += minimos[i];  
  }
  media_max /=NUM_CICLOS_AMOSTRA;
  media_min /= NUM_CICLOS_AMOSTRA;

  A_out = float(media_max - media_min)/(2.);
  Serial.print(f_sin);
  Serial.print(",");
  Serial.print(A_out/A_sin,4);
  ampl_impr = true;

}
void imprimir_fase()
{
  float fase_media;
  for(int i = 0; i<NUM_CICLOS_AMOSTRA; i++ )
    fase_media += fases[i];
    
  fase_media /=NUM_CICLOS_AMOSTRA;
  while(fase_media >180) fase_media=fase_media-360;
  Serial.print(",");
  Serial.println(fase_media);
  fase_impr = true;
  
}

void loop()
{
  if(index_limite == NUM_CICLOS_AMOSTRA && !ampl_impr)  
  {
    imprimir_ampl();
  }

  if(index_fase== NUM_CICLOS_AMOSTRA && ampl_impr && !fase_impr)
  {
    imprimir_fase();
  }
}
