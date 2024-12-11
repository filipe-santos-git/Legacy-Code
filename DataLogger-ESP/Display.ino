void setup() {
  
  Serial.begin(115200);       //Inicializa a comunicação serial com um baud-rate de 115200 bps
  picture(0);                 //Envia o background(imagem) base para o display
  delay(1000);   
}

  // Função Principal
void loop() {
  
  int analog = analogRead(36);  //Faz a leitura analogica do canal 36 do esp32 (up)
  bateria(analog/41);           
  velocidade(analog/29);
  temperatura((analog/41)-5,(analog/41),(analog/41)-10);

  status();
  delay(200);
  }

//Função que envia uma determinada imagem (00-FF) para o display  (pronto)
void picture(int id) {
  
  byte Imagem[] = {0xAA,0x70,0x00,id,0xCC,0x33,0xC3,0x3C};
  Serial.write(Imagem,sizeof(Imagem));
  }

//Função que posiciona e rotaciona o ponteiro  (pronto)
void ponteiro(int ang){
  
  int angH = ang/256;
  int angL = ang%256;
  byte Cobertura[] = {0xAA,0x9C,0x00,0x04,0x00,0x75,0x00,0x0B,0x02,0xC5,0x02,0x61,0x00,0x75,0x00,0x0B,0xCC,0x33,0xC3,0x3C};
  byte Ponteiro[] = {0xAA,0x9E,0x00,0x00,0x03,0x00,0xE5,0x00,0x65,0x00,0xFD,0x00,0xEE,0x00,0xF1,0x00,0x82,angH,angL,0x00,0xED,0x00,0x85,0xCC,0x33,0xC3,0x3C};
  Serial.write(Cobertura,sizeof(Cobertura)); // Cobre a posição anterior do ponteiro com uma imagem 
  Serial.write(Ponteiro,sizeof(Ponteiro));  // Posiciona o ponteiro;
}

//Função que envia a velocidade de maneira digital e analogica (pronto)
void velocidade (int speed){
  
  int c = speed/100;
  int d = speed/10 - 10*c;
  int u = speed%10;
  byte Velocidade[] = {0xAA,0x98,0x01,0x09,0x00,0x92,0x00,0x82,0x05,0xFF,0xFF,0x00,0x1F,c+48,d+48,u+48,0xCC,0x33,0xC3,0x3C};
  if (speed <= 10){
    ponteiro(0);}
  else if (speed > 10 && speed <140){
    ponteiro((4*speed)-40);} //conversão para que o angulo seja aproximadamente equivalente a velocidade no velocimetro
  else {
    ponteiro(522);}
  Serial.write(Velocidade,sizeof(Velocidade)); // Envia a velocidade digital para o display
}

// Função que imprime no display a porcentagem da bateria e a controla de maneira gráfica  (pronto)
void bateria (int bat){
  
  byte Palette1[] = {0xAA,0x40,0x30,0x30,0x30,0xFF,0xFF,0xFF,0xCC,0x33,0xC3,0x3C};
  Serial.write(Palette1,sizeof(Palette1));  // Define o FG como cinza e BG como branco
  
  int c = bat/100;
  int d = bat/10 - 10*c;
  int u = bat%10;
  byte Cobertura[] = {0xAA,0x5B,0x00,0x12,0x00,0x13,0x00,0x75,0x00,0x31,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0x00,0x16,0x00,0xAE,0x00,0x5B,0x00,0xD0,0x00,0x16,0x00,0x88,0x00,0x5B,0x00,0xAA,0x00,0x16,0x00,0x62,0x00,0x5B,0x00,0x84,0xCC,0x33,0xC3,0x3C};
  Serial.write(Cobertura,sizeof(Cobertura));      // Limpa a porcetagem e o valor gráfico da bateria
  byte Porcentagem[] = {0xAA,0x98,0x00,0x10,0x00,0x10,0x00,0x82,0x04,0xFF,0xFF,0x00,0x1F,c+48,d+48,u+48,0x20,0x25,0xCC,0x33,0xC3,0x3C};
  Serial.write(Porcentagem,sizeof(Porcentagem));      // Envia a porcentagem da bateria ao display

  byte Palette2[] = {0xAA,0x40,0x9B,0xCA,0x41,0xE6,0x4D,0x3D,0xCC,0x33,0xC3,0x3C};
  Serial.write(Palette2,sizeof(Palette2));  //Define o FG como verde e o BG como vermelho
  
  if (bat > 80 && bat <= 100){              //Posiciona os quadrados da bateria
    byte Quadrados[] = {0xAA,0x5B,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0x00,0x16,0x00,0xAE,0x00,0x5B,0x00,0xD0,0x00,0x16,0x00,0x88,0x00,0x5B,0x00,0xAA,0x00,0x16,0x00,0x62,0x00,0x5B,0x00,0x84,0xCC,0x33,0xC3,0x3C};
    Serial.write(Quadrados,sizeof(Quadrados));}
  else if (bat > 60 && bat <= 80){
    byte Quadrados[] = {0xAA,0x5B,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0x00,0x16,0x00,0xAE,0x00,0x5B,0x00,0xD0,0x00,0x16,0x00,0x88,0x00,0x5B,0x00,0xAA,0xCC,0x33,0xC3,0x3C};
    Serial.write(Quadrados,sizeof(Quadrados));}
  else if (bat > 40 && bat <= 60){
    byte Quadrados[] = {0xAA,0x5B,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0x00,0x16,0x00,0xAE,0x00,0x5B,0x00,0xD0,0xCC,0x33,0xC3,0x3C};
    Serial.write(Quadrados,sizeof(Quadrados));}
  else if (bat > 20 && bat <= 40){
    byte Quadrados[] = {0xAA,0x5B,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0xCC,0x33,0xC3,0x3C};
    Serial.write(Quadrados,sizeof(Quadrados));}
  else if (bat > 0 && bat <= 20){
    byte Quadrados[] = {0xAA,0x5A,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0xCC,0x33,0xC3,0x3C};
    Serial.write(Quadrados,sizeof(Quadrados));}
}

    // Função que imprime no display as temperaturas do acumulador e dos motores (pronto)
void temperatura (int temp_acum, int temp_motor1, int temp_motor2){
  
  byte Palette1[] = {0xAA,0x40,0x30,0x30,0x30,0xFF,0xFF,0xFF,0xCC,0x33,0xC3,0x3C};
  Serial.write(Palette1,sizeof(Palette1)); // Define o FG como cinza e BG como branco
  byte Cobertura[] = {0xAA,0x5B,0x01,0xA9,0x00,0x05,0x01,0xC6,0x00,0x6A,0xCC,0x33,0xC3,0x3C};
  Serial.write(Cobertura,sizeof(Cobertura));    // Limpa os valores de temperatura
  
  int da = temp_acum/10;
  int ua = temp_acum%10;
  byte Acumulador[] = {0xAA,0x98,0x01,0xAC,0x00,0x0C,0x00,0x82,0x02,0xFF,0xFF,0x00,0x1F,da+48,ua+48,0xCC,0x33,0xC3,0x3C};
  Serial.write(Acumulador,sizeof(Acumulador));  // Envia a temperatura do acumulador
  int d1 = temp_motor1/10;
  int u1 = temp_motor1%10;
  byte Motor1[] = {0xAA,0x98,0x01,0xAC,0x00,0x2C,0x00,0x82,0x02,0xFF,0xFF,0x00,0x1F,d1+48,u1+48,0xCC,0x33,0xC3,0x3C};
  Serial.write(Motor1,sizeof(Motor1));          //Envia a temperatura do Motor1
  int d2 = temp_motor2/10;
  int u2 = temp_motor2%10;
  byte Motor2[] = {0xAA,0x98,0x01,0xAC,0x00,0x50,0x00,0x82,0x02,0xFF,0xFF,0x00,0x1F,d2+48,u2+48,0xCC,0x33,0xC3,0x3C};
  Serial.write(Motor2,sizeof(Motor2));         //Envia a temperatura do Motor2
}


void status (){
   
  byte Cobertura[] = {0xAA,0x5B,0x01,0x35,0x00,0xDC,0x01,0x96,0x00,0xFC,0xCC,0x33,0xC3,0x3C};
  Serial.write(Cobertura,sizeof(Cobertura));  //Limpa o status 
  byte Status[] = {0xAA,0x98,0x01,0x35,0x00,0xDC,0x00,0x82,0x03,0xFF,0xFF,0x00,0x1F,0x50,0x72,0x6F,0x6E,0x74,0x6F,0xCC,0x33,0xC3,0x3C};
  Serial.write(Status,sizeof(Status));      //Envia o status: Pronto
  }

//Descrição do comandos:
  //  byte para escrever um texto:
  //  byte vetor[] = {0xAA,0x98,Xh,Xl,Yh,Yl,Lib_ID,C_Mode,C_dots,Fcolor,Fcolor,Bcolor,Bcolor,String,0xCC,0x33,0x3C,0xC3};
