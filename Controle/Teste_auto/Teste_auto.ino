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


//double t = 0;
double k = 0;

//Resposta em frequencia
double f_sin = 35.25; //Hz
double out_sin = 0;

//Amplitude da resposta em frequencia
double A_sin = 210;
#define NUM_CICLOS_AMOSTRA 100
#define NUM_CICLOS_ACOMODACAO 20
#define FMAX 42
#define FINCREMENT 0.25

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

  ciclos_absolutos = (ciclos_absolutos<INT_MAX-1)? ciclos_absolutos+1 : INT_MAX;

  potenciometro_data_last = potenciometro_data;
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

float fase_media;
void imprimir_fase()
{
  for(int i = 0; i<NUM_CICLOS_AMOSTRA; i++ )
    fase_media += fases[i];
    
  fase_media /=NUM_CICLOS_AMOSTRA;
  while(fase_media >180) fase_media -= 360;
  Serial.print(",");
  Serial.println(fase_media);
  fase_impr = true;
}

void reset_amostra()
{
  for(int i = 0; i<NUM_CICLOS_AMOSTRA; i++)
  {
     maximos[i] = 0; 
     minimos[i] = INT_MAX;  
     fases[i] = 0;       
  }
  
  ciclos_absolutos = 0;
  index_limite = 0;
  ampl_impr = false;
  
  index_fase  = 0; 
  fase_impr = false;

  fase_media = 0;
  media_max = 0;
  media_min = 0;
}

void loop()
{
  if(f_sin<=FMAX)
  {
    if(index_limite == NUM_CICLOS_AMOSTRA && !ampl_impr)  
    {
      //cli();
      imprimir_ampl();
    }

    if(index_fase== NUM_CICLOS_AMOSTRA && ampl_impr && !fase_impr)
    {
      imprimir_fase();

      reset_amostra();
      f_sin+=FINCREMENT;
      //sei();
    }
  }
}
