/*
Controle de velocidade de motor DC com PWM
Autores: Pedro Bertoleti e FilipeFlop
*/
 
#define PINO_PWM                      5    //pino do Arduino que terá a ligação para o driver de motor (ponte H) L298N)
#define TEMPO_NA_MESMA_VELOCIDADE     300  //tempo (ms) em que o motor ficara na mesma velocidade 
  
void setup()
{ 
    //configura como saída pino terá a ligação para o driver de motor (ponte H) L298N)
    pinMode(PINO_PWM, OUTPUT);
}
  
void loop()
{   
    int valor_pwm = 255;   //variavel que armazena o valor do PWM (0..255 -> 0%..100% da rotação do motor) 
    analogWrite(PINO_PWM, valor_pwm);
}
