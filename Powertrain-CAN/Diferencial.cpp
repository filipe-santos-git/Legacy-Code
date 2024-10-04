#include "mbed.h"
#include "math.h"
#define PI 3.14159265359
#define BLOQUEIO_DIRECAO 20
#define ANGULO_VOLANTE 160/2 // 160° -> 80°Esquerda e 80° Direita
//Definindo entrada
#include <cstdint>

#define N_LEITURAS 10
#define SENSOR_MIN_OFFSET_5V 6650  // Para 5 volts utilizando uma fonte debancada
#define SENSOR_MAX_OFFSET_5V 65535
#define SENSOR_MAX_OFFSET_5V_87 58500

int leituras[N_LEITURAS];
int index_leituras;
int total;

int sensor_apps();
void CalibrarADC();

AnalogIn sensor_acelerador(PA_3);
AnalogIn sensor_volante(PA_5);

int PWM();
float getDelta();
float getAngle();
void Diferencial();
float tensao_sensor;
float angulo_sensor, angulo_volante;
float delta, delta_aux;
float A = 1.525;
float d = 1;
float w_ref = 50;
float tan_delta;
float dif_w;
float w_out, w_in;
float pwm_aux;

uint16_t pwm = 0;
uint16_t pwm_ajustado = 0;

void lerTensao_angulo();


// main() runs in its own thread in the OS
int main()
{
    CalibrarADC();
    sensor_acelerador.set_reference_voltage(3.31);
    sensor_volante.set_reference_voltage(3.31);
  
         
    while (true) {

        pwm = sensor_apps();
        Diferencial();
        
        
        printf("T: %.4f, Ang_Vol: %.2f, Ang_Roda: %.2f W_in: %.2f, W_out: %.2f, PWM: %d  \n", tensao_sensor, delta_aux,delta, w_in, w_out,pwm);

    }
}

float getAngle(){
      tensao_sensor = sensor_volante.read_u16()*(3.31/65535.0);
        angulo_sensor = ((tensao_sensor-0.340f)/(2.92f))*360;
        angulo_volante = (angulo_sensor - 180);

        if(angulo_volante < -80){
            return  -80*PI/180;
        }
        else if(angulo_volante > 80){
            return  80*PI/180;
        } else{
            return angulo_volante*PI/180;
        }
}

float getDelta(){
     // 160° de Percurso de Volante e 20° de Bloqueio de Direção
     if(getAngle()>=0.04 || getAngle()<=-0.04){
        return getAngle()/((ANGULO_VOLANTE)/BLOQUEIO_DIRECAO);
     }else{
        return 0;
     }
}

void Diferencial(){

        tan_delta = tan(getDelta());
        dif_w = ((d*tan_delta)/A)*pwm;

        w_out = pwm + abs((dif_w/2));
        w_in = pwm - abs((dif_w/2));

        delta_aux = getAngle();
        delta = getDelta();
}

void CalibrarADC(){
     // Calibração do ADC1
    ADC1->CR &= ~ADC_CR_ADEN;
    ADC1 ->CR |= ADC_CR_ADCAL;

    while((ADC1->CR & ADC_CR_ADCAL) != 0){

    }


    //Habilitar ADC1 
    ADC1->CR |= ADC_CR_ADEN;
    while((ADC1->ISR & ADC_ISR_ADRDY)==0){

    }
}

int sensor_apps() {
  
  int media;

  total = total - leituras[index_leituras];
  // read the sensor:
  leituras[index_leituras] = sensor_acelerador.read_u16();
  // add value to total:
  total = total + leituras[index_leituras];
  // handle index
  index_leituras +=  1;
  if (index_leituras >= N_LEITURAS) {
    index_leituras = 0;
  }
  // calculate the average:
  media = total / N_LEITURAS;



if(media>= (SENSOR_MIN_OFFSET_5V+200) ){
    if(getAngle()<=-0.10 || getAngle()>=0.10){
    return  ((media-SENSOR_MIN_OFFSET_5V)*( SENSOR_MAX_OFFSET_5V_87+SENSOR_MIN_OFFSET_5V )/SENSOR_MAX_OFFSET_5V_87);
    }else{
        return  ((media-SENSOR_MIN_OFFSET_5V)*( SENSOR_MAX_OFFSET_5V+SENSOR_MIN_OFFSET_5V )/SENSOR_MAX_OFFSET_5V);
    }
}else{
    return 0;
}
}
