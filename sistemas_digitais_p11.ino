#define F_CPU 16000000   
#define BAUD 9600        
#define MYUBRR F_CPU/16/BAUD-1 

//////////Definição de constantes///////////

//Primeiro, é definido sequências de textos para 
//auxiliar na visualização da simulação.
//Todos esses textos serão salvos na memória Flash,
//usando PROGMEM.

const char txtAndar[] PROGMEM = "\nAndar velocidade: \0";
const char txtVel1[] PROGMEM = "25% \n\0";
const char txtVel2[] PROGMEM = "50% \n\0";
const char txtVel3[] PROGMEM = "75% \n\0";
const char txtVel4[] PROGMEM = "100% \n\0";
const char txtLuz[] PROGMEM = "\nSensor Luz: \0";
const char txtTemp[] PROGMEM = "\nSensor Temp: \0";
const char txtGas[] PROGMEM = "\nSensor Gas: \0";
const char txtMov1[] PROGMEM = "\nSensor Mov: \0";
const char txtMov2[] PROGMEM = "\nTem mov \0";
const char txtMov3[] PROGMEM = "Nao tem mov \0";
const char txtServ1[] PROGMEM = "\nColeta 0 graus\0";
const char txtServ2[] PROGMEM = "\nColeta +90 graus\0";
const char txtServ3[] PROGMEM = "\nColeta -90 graus\0";
const char txtGiro1[] PROGMEM = "\nGiro para lado: \0";
const char txtGiro2[] PROGMEM = "Esquerdo \0";
const char txtGiro3[] PROGMEM = "Direito \0";
const char txtInicio[] PROGMEM = "\n\nInicio \0";
const char txtFim1[] PROGMEM = "\nIniciando processo de encerramento \0";
const char txtFim2[] PROGMEM = "\nProcesso Encerrado \0";


//Segundo, definiu-se variáveis voláteis que serão
//usadas na função de interrupção

volatile uint8_t cont = 0;  //contador para cada overflow
volatile uint8_t seg = 0;  //contador de segundos
volatile bool atualizar = 0; //sinalizador de que houve acréscimo de segundo
//Esta última variável "printar" será usada para controlar o 
//envio de informações para a MCU.


//Declaração de variáveis receptoras dos valores dos sensores
uint8_t sensorLuz = 0;
uint8_t sensorTemp = 0;
uint8_t sensorGas = 0;
uint8_t sensorMov = 0;

//Variável usada para ligar o robô
bool state = 0;

//Variáveis que recebem a posição atual do robô.
//Serão medidas em decímetros
uint8_t posX = 0;
uint8_t posY = 0;

//instanciar velocidade linear
uint8_t velocLinear = 0;

//Variável que determinar o sentido de locomoção do robô
uint8_t direcao = 0; //0-> +y, 1-> +x, 2-> -y, 3-> -x


//velocidade Linear Máxima determinada
//a partir do seguinte cálculo:
//VelocLinMax = 2pi*r*rps
//sendo r = raio das rodas e rps = rotações por segundo das rodas
//rps é determinado pela divisão do rpm por 60
//Para fins de projeto, fora desejado uma velocidade
//linear máxima que não necessite ser float ao dividir 
//a velocidade em 4. Portanto, fora projetado parâmetros
//de raio e rpm da roda usando os seguintes valores:
//Raio = 0.057010 metros (5.7 centímetros)
//RPM máximo = 67 -> RPS Máximo = 1.11
//Aplicando esses valores na fórmula da velocidade
//máxima, resultou em um valor de aproximadamente 0,4 m/s
const uint8_t velocLinMax = 4;


//////////////Funções utilizadas/////////

///////Protocolo USART///////

//Primeiro, a configuração para utilizar o USART
//Consiste em setar os bits necessários para operá-lo
void USART_Config(unsigned int ubrr){
 UBRR0H = (unsigned char) (ubrr >>8); //define os 8 bits HIGH do ubrr
 UBRR0L = (unsigned char) ubrr;      //define os 8 bits LOW do ubrr
 UCSR0B = 0b00011000; //Ativa RXEN0 e TXEN0 para receber e enviar dados
 UCSR0C = 0b00000110; //Define UCSZ00 e 01 para o USART trabalhar com
 //uma janela de 8bits
}


//Transmissão de dados
void USART_Transmit(unsigned char dado){
  while(!(UCSR0A & 0b00100000)); //Flag UDRE0 indicará
  //que o buffer está vazio e ele poderá receber o dado.
  //Portanto, é usado um while para esperar a flag setar
  UDR0 = dado; //atribui o valor de dado no registrador USART I/O
}

//Transmitir os textos da memória flash
//Irá ler byte a byte do texto selecionado e irá transmitir-los
//até encontrar o fim do texto (0).
void USART_Escreve_Flash(const char *c){
 for(;pgm_read_byte(&(*c)) != 0; c++)
   USART_Transmit(pgm_read_byte(c));
}


///////Ligar o robô/////////

//Com o state em 0, irá ficar no laço while até ocorrer uma das duas ações:
//1)receber qualquer caractere pelo usuário
//2)Switch conectado no pino PD2 ser acionado
void ligar(){
  while(!state){
    if(UCSR0A & 0b10000000){//Se a flag RXC0 acionar, quer dizer que tem 
    //informações no buffer de entrada  
    	state = UDR0; //Como state é booleano, qualquer valor diferente
      				  //de 0 será tratado como 1
    }
    if(PIND & 0b00000100){ //Se pino PD2 receber valor HIGH, muda estado
     state = 1; 
    }
  }; 
  
  USART_Escreve_Flash(txtInicio); //Escrita do texto indicando inicio de operação
  PORTD |= 0b10000000; //Aciona LED da porta PD7 para indicar que está ligado
}


////////Movimentação retilínea do robô
//Ela consiste em 7 etapas
//1)Acionar os seinais digitais de PB0,PB1,PB2 e PB4 para indicar o sentido de giro dos motores
//2)Limpar os valores digitais dos pinos PD5 e PD6 (Acionadores dos motores)
//2.1)Apesar de serem PWM's , eles recebem sinais digitais HIGH para travar as rodas do motor
//   (dada certas condições)
//3)A partir do fator de velocidade desejado (25,50,75 ou 100% da vMax),
//  os valores de parâmetros do PWM são setados para emitir os respectivos sinais
//  nos pinos PD5 e PD6. Com isso, o robô iniciará a locomoção
//4)A velocidade linear atual será determinada pelo fator de velocidade e pela velocidade linear máxima  
//5)A partir da direção atual do robô, ele irá atualizar sua posição atual
//  e a enviará para o MCU em intervalos de 1 segundo.
//6)Ele andará por um tempo pré-determinado a partir da rota escolhida
//7)Após alcançar o tempo necessário para chegar no ponto de coleta,
//  todos os pinos responsáveis pelos motores (PD5,PD6,PB0,PB1,PB2,PB4),
//  irão receber valores HIGH para travar o robô na posição.
//  Caso não tivesse esses sinais, ele poderia se locomover por inércia


void andar(uint8_t fatorVel, uint8_t target){ //target  = quantos s
  USART_Escreve_Flash(txtAndar); //usado na simulação para indicar que está andando
    
  PORTB &= 0b11111100; //Set dos pinos para determinar orientação de giro
  PORTB |= 0b00010100;
  PORTD &= 0b10011111; //Limpa os sinais HIGH que possuíam por estarem parados por freio
  
  //If - else para setar os valores de OCR0A e B para gerar os PWM's desejados
  if(fatorVel == 1){ //25%
    USART_Escreve_Flash(txtVel1);
  	OCR0A = 64;
    OCR0B = 64;
    
  }
  
  else if(fatorVel == 2){//50%
    USART_Escreve_Flash(txtVel2);
  	OCR0A = 128;
  	OCR0B = 128;
  }
  
  else if(fatorVel == 3){//75%
    USART_Escreve_Flash(txtVel3);
  	OCR0A = 192;
  	OCR0B = 192;
  }
  
  else{ //100%
    USART_Escreve_Flash(txtVel4);
  	OCR0A = 255;
  	OCR0B = 255;
  }
  
   //velocidade linear máxima fracionada usando o fator de velocidade escolhido
  velocLinear = velocLinMax*fatorVel/4;
  
  //Primeiro envio da informação de posíção inicial via USART
  USART_Transmit(posX);
  USART_Transmit(posY);
  
  //Esse trecho é usado apenas para visualizar a informação pelo TinkerCad
  //Caso usasse o USART, o valor que apareceria na plataforma
  //seria a decodificação ASCII do byte.
  //Serial.print(posX); 
  //Serial.print("-");  
 // Serial.println(posY);
  delay(10);
  
  //Como a direção de locomoção influenciará a posição do robô,
  //Usou-se um switch-case para cada caso.
  //Essa direção é modificada apenas na função de giro.
  switch(direcao){
    case 0: //Sentido +y
      //Contadores de overflow e de segundos das interrupções são zerados
  	  //para iniciar uma contagem de tempo  
      atualizar = 0;
      seg = 0;
      cont=0;
    //enquanto não der o tempo pré-determinado para chegar na posição desejada
        while(seg < target){
            delay(10); 
            if(atualizar){ //Se ocorreu acréscimo de segundo, atualiza posição e envia para a MCU
                atualizar = 0; //desliga o booleano para não realizar a operação 
                              	//novamente (até ser acionado pelo interruptor)
  	     	    posY += velocLinear; //atualiza posição em Y
            	USART_Transmit(posX); //envia posições X e Y atuais
            	USART_Transmit(posY);
              //Trecho usado apenas para simular no tinkercad
            	//Serial.print(posX);
            	//Serial.print("-");
            	//Serial.println(posY);
            }
        }
        break;
    case 1:      //+x
        atualizar=0;
        seg=0;
        cont=0;
        while(seg < target){
            delay(10);
            if(atualizar){
                atualizar = 0;
        	    posX += velocLinear;//
            	USART_Transmit(posX);
            	USART_Transmit(posY);
            	//Serial.print(posX);
            	//Serial.print("-");
            	//Serial.println(posY);
            }
        }
    	break;
    case 2:		//-y
        atualizar=0;
        seg=0;
        cont=0;
        while(seg < target){
            delay(10);
 			if(atualizar){
                atualizar = 0;
                posY -= velocLinear;//
            	  USART_Transmit(posX);
            	  USART_Transmit(posY);
              //Serial.print(posX);
            	//Serial.print("-");
            	//Serial.println(posY);
            }
        }
       	break;
    case 3:   //-x
        atualizar = 0;
        seg=0;
        cont=0;
        while(seg < target){
          delay(10);
 		  if(atualizar){
                atualizar = 0;
        	    posX -= velocLinear;//
            	USART_Transmit(posX);
            	USART_Transmit(posY);
            	//Serial.print(posX);
            	//Serial.print("-");
            	//Serial.println(posY);
          }
        }
    	break;    
  }

  //Habilita o freio do motor para não ser movido pela inércia.
  PORTB |= 0b00010111; //Todos os controles de sentido do motor em HIGH
  PORTD |= 0b01100000; //Acionador dos motores em HIGH
  delay(10); //usado apenas para executar melhor a simulação no Tinkercad
}


/////////////Funções de coletas de dados analógicos e digitais/////

//Para cada sensor fora utilizado uma função respectiva. 
//Os sensores analógicos (Luz, temperatura e Gás) apresentam a mesma estrutura,
//com poucas diferenças no acionamento de bits
//Então, será explicado na função senLuz o funcionamento principal dos 
//receptores analógicos

void senLuz(){ 
  ADMUX &= 0b11110000; //Limpa os bits referentes à seleção da porta analógica a ser lida
  ADMUX |= 0b00000000; //Seleciona a porta analógica desejada (neste caso, A0)
  ADCSRA = 0b10000000; //Habilita ADEN. Ativa o processo de conversão AD
  ADCSRA |= 0b01000000;//Aciona ADSC. Inicia processo de conversão
  while(!(ADCSRA & 0b00010000));//Enquanto a conversão não termina, o processo entra em espera
  //Quando a flag ADIF vira 1, a conversão ocorreu e o valor está armazenado em ADC
  sensorLuz = ADC; //Resistor de 100 ohms faz o fotorresistor gerar valores 1-169 distribuidos quase linearmente.
  USART_Escreve_Flash(txtLuz); //Envio de texto para indicar que o valor seguinte é do sensor de luz
  USART_Transmit(sensorLuz); //transmite byte do valor recebido pelo sensor
  //Serial.println(sensorLuz); //Apenas para fins de simulação no tinkercad
  delay(10);
}

void senTemp(){//20-358 mapeado para 0-169
  ADMUX &= 0b11110000; //limpa portas do MUX
  ADMUX |= 0b00000001; //escolhe A1
  ADCSRA = 0b10000000; 
  ADCSRA |= 0b01000000;
  while(!(ADCSRA & 0b00010000));
  //Range de valores recebidos em ADC varia de 20-358
  //Subtração por 20 e divisão por 2 faz o range ser de 0-169
  sensorTemp = (ADC-20)/2; 
  
  USART_Escreve_Flash(txtTemp); //texto indicativo de que será valor de temperatura
  USART_Transmit(sensorTemp); //envio do valor de sensorTemp
  //Serial.println(sensorTemp);// Usado apenas para simulação no tinkercad
  delay(10);
}

void senGas(){//320-763 para 0-147
  ADMUX &= 0b11110000;
  ADMUX |= 0b00000010;//Porta A2
  ADCSRA = 0b10000000;
  ADCSRA |= 0b01000000;
  while(!(ADCSRA & 0b00010000));
  
  //converter range 320-763 para 0-147
  sensorGas = (ADC-320)/3;
  
  USART_Escreve_Flash(txtGas);
  USART_Transmit(sensorGas);
 // Serial.println(sensorGas);
  delay(10);
}

//Como o sensor PIR envia sinais HIGH ou LOW,
//A recepção do sinal realizou-se em uma porta digital (PB5)
//Caso recebe HIGH, envia um texto, caso contrário, envia o outro.

void senMov(){
  
  USART_Escreve_Flash(txtMov1);
  if(PINB & 0b00100000){
    USART_Escreve_Flash(txtMov2);
  }
  else{
    USART_Escreve_Flash(txtMov3);
  } 
  delay(10);
}

////////////Coleta das informações///////
//O processo de coleta deu-se em 6 etapas
//1)Acionar LED de coleta (PD4)
//2)Chamar as funções de sensor de luz, temperatura e gás 
//  para realizar apenas uma coleta de valor no ponto
//3)Modificar OCR2B (PD3)para valores específicos que 
//  gerem posições do servo de 0 graus, +90 e -90 graus
//4)Zerar contador de interrupções para entrar em um laço
//  de espera. Necessário para dar tempo para o servo motor 
//  chegar na posição designada antes de realizar a coleta
//5)Chama a função de sensor de movimento.
//6)Por fim, Desliga o LED indicador de coleta de dados

void coleta(){
  
  PORTD |= 0b00010000; //LED PD4
    
  senLuz();
  senTemp();
  senGas();
  
  //0 graus
  OCR2B = 94;  //Giro para a posição 0 graus
  cont = 0;
  seg = 0;
  while(seg<2)delay(10); //2 segundos de espera para o servo chegar na posição
  
  USART_Escreve_Flash(txtServ1);
  senMov();   //coleta dado de movimento
  
  //+90graus
  OCR2B = 255;
  cont = 0;
  seg = 0;
  while(seg<2)delay(10);
  
  USART_Escreve_Flash(txtServ2);
  senMov();
  
  
  //-90 graus
  OCR2B = 0;
  cont = 0;
  seg = 0;
  while(seg<2)delay(10);
  
  USART_Escreve_Flash(txtServ3);
  senMov();
 
  PORTD &= 0b11101111;  //desliga LED PD4
}

////////Giro do robô/////////
// 5 etapas
//1)Desliga sinais digitais em PD5 e 6
//2)Aciona PWM de 25% de velocidade
//3)Dado o parâmetro "lado", os pinos PB0,1,2,4 irão determinar a orientação do giro
//4)Dada a direção atual e o giro feito, 
//  um switch-case irá selecionar a direção atual do robô
//5)Zera contador de ovf e tempo e entra em um laço de espera 
//  até o robô realizar a rotação de 90 graus. 

void giro(bool lado){
  
  USART_Escreve_Flash(txtGiro1); //Indica ao usuário que será feito um giro
  
  PORTD &= 0b10011111; //zera PD5 e 6 para liberar freios
  OCR0A = 64;          //25% do PWM
  OCR0B = 64;
  if(lado){//determinado como esquerdo
    
    USART_Escreve_Flash(txtGiro2);
  	PORTB |= 0b00010001; //acionar e zerar bits necessários
    PORTB &= 0b11111001;
    
    switch(direcao){ //orientação sentido anti-horário
      case 0: //+y
    	direcao = 3;  
    	break;
      case 1: //+x
    	direcao = 0;
    	break;
      case 2: //-y 
    	direcao = 1;
    	break;
      case 3: //-x
    	direcao = 2;
    	break;
    }
  }
  else{//lado direito
    USART_Escreve_Flash(txtGiro3);
  	PORTB |= 0b00000110;
    PORTB &= 0b11101110;
    switch(direcao){ //sentido horário
      case 0:
    	direcao = 1;
    	break;
      case 1:
    	direcao = 2;
    	break;
      case 2:
    	direcao = 3;
    	break;
      case 3:
    	direcao = 0;
    	break;
    }
  }
  cont = 0;
  seg = 0;
  while(seg < 2)delay(10); //Tempo usado para realizar o giro de 90 graus
  delay(10);
}

/////////Encerrar funcionamento/////
//1)Aciona e zera bits de motor para realizar o giro necessário
//2)Aciona PWM de 50%
//3)zera contadores ovf e seg para realizar os dois giros no eixo
//4)Durante os giros, os LEDs PD4 e PD7 alternam seus estados
//  a partir do valor do contador de ovf. 
//  Para isso, usou-se a operação de resto de divisão para separar
//  em intervalos de 0,5s
//5)Após realizar os giros,os pinos responsáveis pelos motores são zerados
//  e a operação do robô se encerra.

void encerrar(){
  
  USART_Escreve_Flash(txtFim1); 
  PORTD &= 0b10011111;  //tira robô do freio
  PORTB |= 0b00010001;  //aciona bits de orientação dos motores
  PORTB &= 0b11111001;
  OCR0A = 128;     //50% PWM
  OCR0B = 128;
  
  cont = 0;
  seg = 0;  
  while(seg < 8){ //tempo necessário para 2 voltas completas
    delay(10);
    if((cont % 244) > 122){ //intervalos de 0,5 segundos
    	PORTD |= 0b10000000;
    	PORTD &= 0b11101111; 
    }
    else{
    	PORTD |= 0b00010000;
    	PORTD &= 0b01111111;   
    }
  }
  
  PORTD &= 0b01101111; //zera todos os bits responsaveis pelos motores
  PORTB &= 0b11101000;
  OCR0A = 0;
  OCR0B = 0;

  USART_Escreve_Flash(txtFim2);
}

/////////SETUP//////
//1)Set de pinos para determinar se são entradas ou saidas
//2)Set de valores dos Timer/Counter Control 0 e 2
//3)Set de interrupção
//4)Set de Voltagem de referencia do ADMUX
//5)Set de configuração do USART

void setup(){
  
  //Entradas e saídas
  DDRB |= 0b00011111; 
  DDRB &= 0b11011111;
  DDRD |= 0b11111000;
  DDRC &= 0b11111000;
  
  //Timers para PWM e contadores
  TCCR0A |= 0b10100011; //FastPWM, limpa valores de A e B ao comparar, seta OCnA e B para BOTTOM
  TCCR0B |= 0b00000011; //Prescaler do clock em 64
  TCCR2A |= 0b10100011; 
  TCCR2B |= 0b00000110;//Prescaler do clock em 256
  
  TIMSK2 |= 0b00000001; //Usa timer 2 para interrupções do tipo overflow.
  sei(); //habilita funções de interrução
  
  ADMUX |= 0b01000000; //Set de Vcc de referencia para 5V
  
  USART_Config(MYUBRR); //Configura USART usando valor pré-definido de ubrr
  						
  //Serial.begin(9600); //Usado apenas para simulações no tinkercad
}


////////Interrupção///

//Apenas conta quantas interrupções foram feitas (cont)
//Se cont atingir um valor específico, ele será zerado e 
//o tempo em segundos sofrerá acréscimo
ISR(TIMER2_OVF_vect){
  cont++;  //Tempo de cada valor cont = 0.004096s.
  		   //TempoCont = (256/16Mhz)*256
  if(cont == 244){ //0.004096 * 244 = 0.999424s ~ 1s
   cont = 0; 
   seg++;
   atualizar = 1; //Habilita flag de mudança do tempo de segundos para
           //realizar o envio da posição ao MCU
  }
}

///////Execução de todas as funções

void loop(){
  
  //Ligar robô por switch ou input em USART
  ligar();
  //Posição inicial (0,0)
  
  //Andar até ponto 1 a 75% de velocidade por 4 segundos
  andar(3,4); 
  //Posição (0,12) (decímetros)
  
  //coletar valores em p1
  coleta();
  
  //girar para a direita
  giro(0);
  
  //Andar até p2 a 50% de velocidade por 4 segundos
  andar(2,4);
  //Posição (8,12)

  //coletar valores em p2
  coleta();
  
  //girar para a esquerda
  giro(1);
  
  //andar até p3 a 25% por 8 segundos
  andar(1,8);
  //Posicao (8,20)
  
  //coletar valores em p3
  coleta();
  
  //girar para direita
  giro(0);
  
  //andar até p4 a 50% por 6 segundos
  andar(2,6);
  //Posicao (20,20)
  
  //coletar valores em p4
  coleta();
  
  //girar para direita
  giro(0);
  
  //andar até p5 a 100% por 5 segundos
  andar(4,5);
  //Posição Final (20,0)
  
  //Encerrar funcionamento
  encerrar();

  exit(0);
} 
