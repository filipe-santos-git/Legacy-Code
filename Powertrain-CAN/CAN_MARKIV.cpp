#include "mbed.h"
#include "CAN.h"

#define INVERSOR_TX_ID 0x100
#define INVERSOR_RX_ID 0x101

#define INVERSOR_TX_ID_2 0x200
#define INVERSOR_RX_ID_2 0x201

#define MAXRPM 9000
#define MAXPWM 65535
#define MAX_CAN_DATA_SIZE 8

#define N_LEITURAS 10
#define SENSOR_MIN_OFFSET_5V 6650  // Para 5 volts utilizando uma fonte debancada
#define SENSOR_MAX_OFFSET_5V 65535
// Calculo Médias -------
int leituras[N_LEITURAS];
int index_leituras;
int total;
// ---------------------

AnalogIn sensor_acelerador(PA_5);
int sensor_apps();
//CAN

CAN can1(PA_11, PA_12, 1000000);

bool is_can_active();
void reset_can();
bool baud_test();
//Função para enviar as mensagem CAN para o inversor
void send_to_inverter();
void send_to_inverter_2();
// Função para receber dados do inversor
void receive_from_inverter();
void receive_from_inverter_2();
// Fim de CAN

void CalibrarADC();



// Variáveis do TX 

char counter = 0;
uint16_t rpm = 9000;
uint16_t pwm = 0; 
float pwm_percent = 0.0;
float pwm_aux = 0.0;
uint16_t corrente = 200;

char counter2 = 0;
uint16_t rpm2 = 9000;
uint16_t pwm2 = 0;
float pwm_percent2 = 0.0;
float pwm_aux2 = 0.0;
uint16_t corrente2 = 220;



// Variáveis de STATUS - RX
float rx_pwm_percent = 0.0, rx_pwm_percent2 =0.0;
uint16_t rx_rpm, rx_rpm2;
uint16_t rpm_aux, rpm_aux2;
uint8_t rpm_high,rpm_high2;
uint8_t rpm_low,rpm_low2;
uint8_t tensao_high,tensao_high2;
uint16_t tensao_aux,tensao_aux2;
uint16_t tensao, tensao2;
uint8_t contador, contador2;
uint8_t temp_controlador,temp_controlador2;
uint8_t temp_motor,temp_motor2;
uint8_t rx_pwm,rx_pwm2;
uint8_t rx_corrente,rx_corrente2;
float tensao_float,tensao_float2;



// main() runs in its own thread in the OS




int main()
{
    can1.mode(CAN::Normal);
    can1.filter(0, 0, CANStandard);
    CalibrarADC();
    
    while (true) 

    {
        //Buttons(); // Chamamento da função dos botões
        send_to_inverter(); // Chamamento da função pra enviar ao inversor 1
        send_to_inverter_2(); // Chamamento da função para enviar ao inversor 2;
        
        receive_from_inverter(); // Chamento da Função para receber do Inversor;
        receive_from_inverter_2();// Chamamento da Função para receber do Inversor;

        wait_us(1e5); // aguarda 100ms
        
    }
}
//end of main




void receive_from_inverter() {
    //Definir mensagem CAN a ser recebida
    CANMessage inverter_rx_msg;
    inverter_rx_msg.len = 8;


if(is_can_active()) {

    if(can1.read(inverter_rx_msg)){
    // Aguardar a recepção da mensagem do inversor

         if(inverter_rx_msg.id == INVERSOR_RX_ID){
         contador = inverter_rx_msg.data[0] & 0xF;
         tensao_high = inverter_rx_msg.data[0] >> 4;
        // 011000000000
        // 000001101000
        // 011001101000


         tensao_aux = tensao_high << 8;
         tensao = tensao_aux | inverter_rx_msg.data[1];
         temp_controlador = inverter_rx_msg.data[2]-100; // Varia de 0-255, de -100ºC à 155ºC
         temp_motor = inverter_rx_msg.data[3]-100;// Varia de 0-255, de -100ºC à 155ºC
         rpm_low = inverter_rx_msg.data[4]; //
         rpm_high = inverter_rx_msg.data[5];
         rx_pwm = inverter_rx_msg.data[6]; // Varia de 0-255, 0 a 100%
         rx_corrente = inverter_rx_msg.data[7];
         rpm_aux = rpm_high << 8;
         rx_rpm = rpm_low | rpm_aux; 
         tensao_float = tensao/10.0f;
         rx_pwm_percent = (rx_pwm/255.0f)*100;

        printf("\r\n\t RX ID: 0x%x cont = %d V = %.1fV Temp_Inv = %d°C Temp_Mot = %d°C RPM = %d PWM = %d, %.2f%%, I = %dA)",inverter_rx_msg.id,contador,tensao_float,temp_controlador,temp_motor,rx_rpm,rx_pwm,rx_pwm_percent, rx_corrente);
         }
    }
 } else {
            printf("Conexão CAN inativa. Tentando reinicializar...\n");
            reset_can(); 
        }
   }
    
   void receive_from_inverter_2() {
    //Definir mensagem CAN a ser recebida
    CANMessage inverter_rx_msg2;
    inverter_rx_msg2.len = 8;


if(is_can_active()) {

    if(can1.read(inverter_rx_msg2)){
    // Aguardar a recepção da mensagem do inversor

         if(inverter_rx_msg2.id == INVERSOR_RX_ID_2){
         contador2 = inverter_rx_msg2.data[0] & 0xF;
         tensao_high2 = inverter_rx_msg2.data[0] >> 4; 
         tensao_aux2 = tensao_high2 << 8; //                  0000101000000000;
         tensao2 = tensao_aux2 | inverter_rx_msg2.data[1];//+ 0000000010100010 = 0000101010100010;
         temp_controlador2 = inverter_rx_msg2.data[2]-100; // Varia de 0-255, de -100ºC à 155ºC
         temp_motor2 = inverter_rx_msg2.data[3]-100;// Varia de 0-255, de -100ºC à 155ºC
         rpm_low2 = inverter_rx_msg2.data[4]; //
         rpm_high2 = inverter_rx_msg2.data[5];
         rx_pwm2 = inverter_rx_msg2.data[6]; // Varia de 0-255, 0 a 100%
         rx_corrente2 = inverter_rx_msg2.data[7];
         rpm_aux2 = rpm_high2 << 8;
         rx_rpm2 = rpm_low2 | rpm_aux2; 
         tensao_float2 = tensao2/10.0f;
         rx_pwm_percent2 = (rx_pwm2/255.0f)*100;


        printf("\r\n\t RX ID: 0x%x cont = %d V = %.1fV Temp_Inv = %d°C Temp_Mot = %d°C RPM = %d PWM = %d, %.2f%%, I = %dA)",INVERSOR_RX_ID_2,contador2,tensao_float2,temp_controlador2,temp_motor2,rx_rpm2,rx_pwm2,rx_pwm_percent2, rx_corrente2);
         }
    }
 } else {
            printf("Conexão CAN inativa. Tentando reinicializar...\n");
            reset_can(); 
        }
   }


void send_to_inverter(){
    
    CANMessage inverter_tx_msg;
    inverter_tx_msg.id = INVERSOR_TX_ID;
    inverter_tx_msg.len = 8; // Define o tamanho da msg, max = 8 Bytes
 


    inverter_tx_msg.data[0] = rpm & 0xFF; //  Byte menos significativos do RPM
    inverter_tx_msg.data[1] = rpm >> 8; // Byte mais significativos do RPM
    inverter_tx_msg.data[2] = 15; // Numero de pares de polos do motor
    inverter_tx_msg.data[3] =sensor_apps() & 0xFF; // Byte menos significativo do PWM
    inverter_tx_msg.data[4] = sensor_apps() >> 8; // Byte mais significativo do PWM
    inverter_tx_msg.data[5] = corrente & 0xFF; // Byte menos significativo corrente;
    inverter_tx_msg.data[6] = corrente >> 8; // Byte mais significativo da corrente;
    inverter_tx_msg.data[7] = 0b00000000; 
    

    //can1.write(inverter_tx_msg);
    //Enviar a mensagem CAN para o inversor
if(baud_test()){
    if(can1.write(inverter_tx_msg)){

    pwm_aux = pwm;
    pwm_percent = (pwm_aux/MAXPWM)*100;

    printf("\r\n\tTX:ID0x%x RPM HB = %d  RPM LB= %d RPM MAX = %d PP = %d PWM LB = %d PWM HB = %d PWM1 = %d %.2f%% I LB = %d I HB = %d I Máx = %dA ",inverter_tx_msg.id, inverter_tx_msg.data[0], inverter_tx_msg.data[1],rpm,inverter_tx_msg.data[2], inverter_tx_msg.data[3], inverter_tx_msg.data[4],pwm,pwm_percent, inverter_tx_msg.data[5], inverter_tx_msg.data[6],corrente);
    }
} else{
            printf("Mesagem não enviada...\n");
            reset_can(); 
}

}

void send_to_inverter_2(){
    
    CANMessage inverter_tx_msg2;
    inverter_tx_msg2.id = INVERSOR_TX_ID_2;
    inverter_tx_msg2.len = 8; // Define o tamanho da msg, max = 8 bits
 

    inverter_tx_msg2.data[0] = rpm2 & 0xFF; //  Byte menos significativos do RPM
    inverter_tx_msg2.data[1] = rpm2 >> 8; // Byte mais significativos do RPM
    inverter_tx_msg2.data[2] = 15; // Numero de pares de polos do motor
    inverter_tx_msg2.data[3] = sensor_apps() & 0xFF; // Byte menos significativo do PWM
    inverter_tx_msg2.data[4] = sensor_apps() >> 8; // Byte mais significativo do PWM
    inverter_tx_msg2.data[5] = corrente2 & 0xFF; // Byte menos significativo corrente;
    inverter_tx_msg2.data[6] = corrente2 >> 8; // Byte mais significativo da corrente;
    inverter_tx_msg2.data[7] = 0b00000000; 

    //can1.write(inverter_tx_msg);
    //Enviar a mensagem CAN para o inversor
if(baud_test()){
    if(can1.write(inverter_tx_msg2)){

    pwm_aux2 = pwm2;
    pwm_percent2 = (pwm_aux2/MAXPWM)*100;

    printf("\r\n\tTX:ID0x%x RPM HB = %d  RPM LB= %d RPM MAX = %d PP = %d PWM LB = %d PWM HB = %d PWM1 = %d %.2f%% I LB = %d I HB = %d I Máx = %dA ",inverter_tx_msg2.id, inverter_tx_msg2.data[0], inverter_tx_msg2.data[1],rpm,inverter_tx_msg2.data[2], inverter_tx_msg2.data[3], inverter_tx_msg2.data[4],pwm,pwm_percent, inverter_tx_msg2.data[5], inverter_tx_msg2.data[6],corrente);
    }
} else{
            printf("Mesagem não enviada...\n");
            reset_can(); 
}

}


void reset_can() {
    
    can1.reset(); // redefinir a interface CAN
    can1.mode(CAN::Normal);
    can1.filter(0, 0, CANStandard); // reconfigurar o filtro para receber todas as mensagens
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
    return  (media-SENSOR_MIN_OFFSET_5V)*( 65535+SENSOR_MIN_OFFSET_5V )/SENSOR_MAX_OFFSET_5V;
} else{
    return 0;
}
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

bool is_can_active(){
    CANMessage msg;
    return can1.read(msg); // tente ler uma mensagem
}

bool baud_test(){
    CANMessage msg;
    return can1.write(msg);
}
