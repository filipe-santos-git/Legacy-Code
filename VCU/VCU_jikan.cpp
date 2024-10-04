#include "mbed.h"
#include "MPU9250.h"
#include "time.h"
#include "CAN.h"
#include <cstdint>

// Definições de Portas de Entrada
AnalogIn Volante_in(PC_2); // PF_10 (originalmente)
AnalogIn BSE_in(PA_0);    // BSE
AnalogIn APPS1_in(PF_4);  //PA_0
AnalogIn APPS2_in(PF_4);    //PF_4
AnalogOut APPS_out(PA_5);     // APPS (via CAN)
InterruptIn Velocidade_in(PA_4, PullNone); //(PA_4, PullNone)
CAN can1(PD_0,PD_1,500e3);     // Velocidade 1

// Variáveis Globais
float Volante = 0;
float BSE = 0;
float APPS1 = 0;
float APPS2 = 0;    
uint64_t Delta_T1 = 0; 
uint64_t ultimo_t1 = 0;
uint64_t Delta_T2 = 0;
uint64_t ultimo_t2 = 0;
float Veloc_1 = 0;
float Veloc_2 = 0;



//Parametros de Configuração Volante
float Vol_min = 0.425;      //Leitura minima do sensor em Volts
float Vol_max = 2.825;      //Leitura maxima do sensor em Volts
float Vol_ang_min = -180;   //Angulo minimo do sensor para mapeamento
float Vol_ang_max = 180;    //Angulo maximo do sensor para mapeamento

//Parametros de Configuração BSE
float BSE_min = 0.3;        //Leitura minima do sensor em Volts
float BSE_max = 3.0;        //Leitura maxima do sensor em Volts
float BSE_ang_min = 0;      //Angulo minimo do sensor para mapeamento
float BSE_ang_max = 120;    //Angulo maximo do sensor para mapeamento

//Parametros de Configuração APPS
float APPS1_min = 0.425;    //Leitura minima do sensor em Volts
float APPS1_max = 3.021;    //Leitura maxima do sensor em Volts
float APPS2_min = 0.4;      //Leitura minima do sensor em Volts
float APPS2_max = 2.9;      //Leitura maxima do sensor em Volts
float APPS_ang_min = 0;     //Angulo minimo do sensor para mapeamento
float APPS_ang_max = 120;   //Angulo maximo do sensor para mapeamento

//Parametros de Configuração Velocidade
float R = 0.27; //Raio da roda em metros
int   N_furos = 3; //Número de furos no disco de freio

unsigned long int prevTime = 0; //váriaveis para contagem de tempo
unsigned long int prevTime1 = 0; 
const double erro_min = 4500; //limite inferior do valor_entrada para que não seja erro
const double erro_max = 48000; //limite superior do valor_entrada para que não seja erro
bool erro = false; //varável que guarda a informação caso ocorra algum erro


//APPS
int cont = 0;                      // Variável para denotar o início ou fim da contagem de tempo
unsigned long tempo = 0.0;         // Variável para guardar o número de milissegundos ---> Jikan = "tempo" em japonês                 // Variável para guardar o torque
int erro_apps = 0;                 // Variável para denotar a existência de erro



MPU9250 mpu9250;

Timer t;




///////////////////////////////FUNÇÕES AUXILIARES/////////////////////////////////

//Mesma função map do arduino
float map(long x, float in_min, float in_max, float out_min, float out_max) {
    float result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return result;
}

//Função millis() do arduino para contagem de tempo ()ver se não vai dar problema
double millis(){
    using namespace std::chrono;
    auto now_ms = time_point_cast<microseconds>(Kernel::Clock::now());
    long micros = now_ms.time_since_epoch().count();
    return micros / 1000.0;
}

//Verificar se houve um curto ou se foi aberto o circuito do sensor
void Verifica_Erro(int entrada){
  if(entrada < erro_min or entrada > erro_max){
    if(millis() - prevTime >= 100){
       prevTime =  millis();
       erro = true;
    } 
  }
  else{
    prevTime = millis();
  }
}

//APPS
// Função para verificar se ocorreu erros nos torques (Se os torques 
// se tiverem uma diferença de 10% durando 100 milissegundos, será erro)
int VerificaErroTorque(float t1, float t2) {
  if (abs(t1 - t2) > (0.1 * max(t1, t2))){
    cont += 1;
    if(cont == 1) {
      tempo = millis(); 
    }
    if(millis() - tempo > 100) {
      return 0;
    }
    return 1;
  } else{
      cont = 0;
      return 1;
  }
}

//Interrupção Velocidade 1
void contadorFrequenciaISR1(){                                  // Quantidade de pulsos que aconteceram
    Delta_T1 = t.elapsed_time().count() - ultimo_t1; // Tamanho do pulso em s
    ultimo_t1 = t.elapsed_time().count();                  // Renicializa a contagem do tempo     
}

//Interrupção Velocidade 2
void contadorFrequenciaISR2(){                                     // Quantidade de pulsos que aconteceram
    Delta_T2 = t.elapsed_time().count() - ultimo_t2; // Tamanho do pulso em s
    ultimo_t2 = t.elapsed_time().count();                  // Renicializa a contagem do tempo     
}

void enviar(){
    CANMessage SENSORES1, SENSORES2, SENSORES3;
    SENSORES1.id = 1;
    SENSORES1.len =8;
    SENSORES1.data[0]=  ((wchar_t)(int) (Volante*10))%256;
    SENSORES1.data[1]=  ((wchar_t)(int) (Volante*10))/256;
    SENSORES1.data[2]=  ((wchar_t)(int) (BSE*10))%256;
    SENSORES1.data[3]=  ((wchar_t)(int) (BSE*10))/256;
    SENSORES1.data[4]=  ((wchar_t)(int) (APPS1*10))%256;
    SENSORES1.data[5]=  ((wchar_t)(int) (APPS1*10))/256;
    SENSORES1.data[6]=  ((wchar_t)(int) (APPS2*10))%256;
    SENSORES1.data[7]=  ((wchar_t)(int) (APPS2*10))/256;

    SENSORES2.id = 2;
    SENSORES2.len =8;
    SENSORES2.data[0]=  (wchar_t)(int) Veloc_1;
    SENSORES2.data[1]=  (wchar_t)(int) Veloc_2;
    SENSORES2.data[2]=  ((wchar_t)(int) (ax*100))%256;
    SENSORES2.data[3]=  ((wchar_t)(int) (ax*100))/256;
    SENSORES2.data[4]=  ((wchar_t)(int) (ay*100))%256;
    SENSORES2.data[5]=  ((wchar_t)(int) (ay*100))/256;
    SENSORES2.data[6]=  ((wchar_t)(int) (az*100))%256;
    SENSORES2.data[7]=  ((wchar_t)(int) (az*100))/256;

    SENSORES3.id = 3;
    SENSORES3.len =8;
    SENSORES3.data[0]=  ((wchar_t)(int) (temperature*100))%256;
    SENSORES3.data[1]=  ((wchar_t)(int) (temperature*100))/256;
    SENSORES3.data[2]=  ((wchar_t)(int) (gx*100))%256;
    SENSORES3.data[3]=  ((wchar_t)(int) (gx*100))/256;
    SENSORES3.data[4]=  ((wchar_t)(int) (gy*100))%256;
    SENSORES3.data[5]=  ((wchar_t)(int) (gy*100))/256;
    SENSORES3.data[6]=  ((wchar_t)(int) (gz*100))%256;
    SENSORES3.data[7]=  ((wchar_t)(int) (gz*100))/256;   
    

    if (can1.write(SENSORES1)){
     //   printf("Mensagem SENSORES1 foi Enviada");
     
    }
    else{
        can1.reset();
    }

    if (can1.write(SENSORES2)){
       // printf("Mensagem SENSORES2 foi Enviada");
    }
    else{
        can1.reset();
    }

    if (can1.write(SENSORES3)){
        //printf("Mensagem SENSORES3 foi Enviada");
    }
    else{
        can1.reset();
    }
}

//////////////////////////////////////SENSORES//////////////////////////////////////////

// Função para ler o sensor de volante
void LerSensorVolante() {
  //Mapea a leitura ADC (0-65535) em angulos (-180° a 180 °) considerando a banda morta do sensor.
    Volante = map(Volante_in.read_u16(), (Vol_min/3.3)*65535, (Vol_max/3.3)*65535,Vol_ang_min, Vol_ang_max);
}

// Função para ler o sensor BSE
void LerSensorBSE() {

        Verifica_Erro( BSE_in.read_u16());

        BSE = map(BSE_in.read_u16(), (BSE_min/3.3)*65535, (BSE_max/3.3)*65535, BSE_ang_min, BSE_ang_max);

        if(erro){
        //    printf("ERRO\n");
        }
        else{
            if(millis() - prevTime1 >= 200){
            //    printf("Valor entrada: %.2f\n", BSE);
                prevTime1 = millis();
            }
        }
}

// Função para ler sensores de torque e controlar a saída
void LerSensoresTorque() {
    //APPS
        // Determina a tensão que será enviada para o inversor (read_u16 devolve valor entre 0 e 65535)
        APPS1 = map(APPS1_in.read_u16(), (APPS1_min/3.3)*65535, (APPS1_max/3.3)*65535, APPS_ang_min, APPS_ang_max);
        APPS2 = map(APPS2_in.read_u16(), (APPS2_min/3.3)*65535, (APPS2_max/3.3)*65535, APPS_ang_min, APPS_ang_max);
        
        float maxi = max(APPS1,APPS2);

        // Verifica Erros e Envia o Torque, se acontecerem os erros, deve mandar torque 0 para o inversor
        if (VerificaErroTorque(APPS1, APPS2) == 0){
            // Aconteceu erro (torques com mais de 10% de diferença por 100 ms ou mais)
            APPS_out.write_u16(0);
        } else{
            APPS_out.write_u16(maxi);
        }

}


// Função para ler sensores de velocidade
void LerSensoresVelocidade() {    
    //Velocidade 1
    //    Velocidade_in.fall(NULL); // Desativa o interrupt
    float f1 = 0;
    float f2 = 0;

        if (Delta_T1 != 0){
            f1 = 1000000.0/(N_furos*Delta_T1); // Calcula a frequência em Hz (Rotações por segundo)    
        }
        Veloc_1 = 2*f1*3.1415*R;

    //Velocidade 2
        if (Delta_T2 != 0){
            f2 = 1000000.0/(N_furos*Delta_T2); // Calcula a frequência em Hz (Rotações por segundo)
        }
        Veloc_2 = 2*f2*3.1415*R;
        
            //RPM = f*60
            //Velocidade Angular = 2*pi*f
            //Velocidade Linear  = 2*pi*f*R
}


// Função para leitura dos sensores MPU9250
void LerSensoresMPU(MPU9250& mpu9250) {

      // Se o pino for para HIGh, todo os registradores recebem novos dados
        if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // Na interrupção, checa se a data está prota para interromper
            
            mpu9250.readAccelData(accelCount);  // Lê os valores x/y/z do adc   
            // Calcula a aceleração em g's
           ax = (float)accelCount[0] * aRes - accelBias[0];
            ay = (float)accelCount[1] * aRes - accelBias[1];   
            az = (float)accelCount[2] * aRes - accelBias[2];  
    
            mpu9250.readGyroData(gyroCount);  // Lê os valores x/y/z do adc  
            // Calcula a velocidade angular em graus por segundo
            gx = (float)gyroCount[0] * gRes - gyroBias[0];
            gy = (float)gyroCount[1] * gRes - gyroBias[1];  
            gz = (float)gyroCount[2] * gRes - gyroBias[2];   
        }

        // Normalização e conversão dos valores obtidos
        mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, 0, 0, 0);

}


void LerSensorTemperatura(){
        // Lê o valores adc de temperatura em Kelvin, converte para graus Celsius e printa
        tempCount = mpu9250.readTempData();
        temperature = ((float) tempCount) / 333.87f + 21.0f;
        
}

//////////////////////////////////////FUNÇÃO PRINCIPAL//////////////////////////////////////////
int main() {
    i2c.frequency(400000);

    mpu9250.resetMPU9250();
    mpu9250.MPU9250SelfTest(SelfTest);
    mpu9250.initMPU9250();
    mpu9250.getAres();
    mpu9250.getGres();

    can1.mode(CAN::Normal);
    can1.filter(0,0,CANStandard);

   t.start();

    while (true) {
        LerSensoresMPU(mpu9250);
        LerSensorVolante();
        LerSensorBSE();
        LerSensoresTorque();
        LerSensoresVelocidade();
        LerSensorTemperatura();

    //     Reinicialização dos parâmetro do pulso                         
        Delta_T1 = 0;   //[DESCOBRIR SE ISSO É NECESSARIO PARA VELOCIDADE ZERAR]               
        ultimo_t1 = t.elapsed_time().count();        
        Velocidade_in.fall(&contadorFrequenciaISR1); // Ativa o interrupt        

//////////////////////////////////////DEBUG//////////////////////////////////////////        
        printf("Velocidade 1 %.2f m/s,Velocidade 2 %.2f m/s \n",Veloc_1, Veloc_2); // Converte a velocidade para RPM
        printf("Volante: %.2f°, APPS1: %.2f°, APPS2: %.2f°, BSE: %.2f°\n", Volante,APPS1,APPS2,BSE);
        printf("ax = %f, ay = %f, az = %f  m/s²\n", ax * 9.81 - 0.15, ay * 9.81 - 0.1, az * 9.81 + 0.12); //Printa os dados de aceleração convertendo para m/s² e fazendo a conversão
        printf("gx = %f, gy = %f, gz = %f  rad/s\n", gx, gy, gz); // Printa os dados de velocidade angular
        printf(" temperatura = %f  C\n\r", temperature);
        enviar();
        wait_us(500000);
        
    }
}
 // 
