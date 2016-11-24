#define MOTOR_D1    6
#define MOTOR_D2    5
#define MOTOR_E1    10
#define MOTOR_E2    9
#define LED         13
#define VELOCIDADE_MAXIMA         254
#define VELOCIDADE_BASE_RETA      130
#define VELOCIDADE_BASE_CURVA      55
#define NUMERO_DE_SENSORES          7
#define LIMIAR 100
#define KPR     1.6    // kp para reta
#define KDR     0      // kd para reta
#define KIR     0      // ki para reta
#define KPC     10     // Kp para cruva
#define KDC     200    // kd para curva
#define KIC     0.5    // ki para curva
#define RETA        1
#define CURVA      -1
#define DEBUG       0
#define DELAY     1000
#define DELAY_PARADA 300
#define BRANCA      true
#define PRETA       false
#define TEMPO_PARA_FINAL           80000 //milissegundos
#define TEMPO_LIMITE_ERRO_MARCACAO   200
#define ESQUERDA    0
#define DIREITA     1
#define TESTE_DE_MARCACAO  10
#define FINAL_POR_TEMPO     0
#define FINAL_POR_MARCACAO  1
#define FINAL_POR_CONTADOR  1
#define NUMERO_DE_CRUZAMENTOS  2
#define LIMITE_TEMPO_ESQUERDA  500
#define N_CARACTERES           30
#define ULTIMA_MARCACAO_ESQUERDA  19

short tipo = RETA;
float sensores[NUMERO_DE_SENSORES]           = {A6, A5, A4, A3, A2, A1, A0};
float erros[NUMERO_DE_SENSORES]              = {-40.0, -25.0,-15.0, 0 , 15.0, 25.0, 40.0};
float verificador_esquerda = A7;
float verificador_direita = 4;
float erro            = 0;
float correcao        = 0;
float erroAnterior    = 0;
float somatorioDeErro = 0;
float kpr = KPR;
float kdr = KDR;
float kir = KIR;
float kpc = KPC;
float kdc = KDC;
float kic = KIC;
float deltaTime = 0;
long int ti = 0;
long int tempo_ultima_marcacao   = 0;
long int tempo_ultimo_cruzamento = 0; 
int velocidadeAtualReta   = VELOCIDADE_BASE_RETA;
int velocidadeAtualCurva   = VELOCIDADE_BASE_CURVA;
int conta = 1;
int limiar            = 0;
int tipoDeFinal       =-1;
int contadorDeFinal   = 0;
int parada            = false;
int direito           = 0;
int esquerdo          = 0;
bool corDaLinha       = BRANCA;
bool flagDeCruzamento    = false; //Se o flag for verdadeiro a proxima vez que os verificadores virem algoa será o cruzamento
//bool flagDeVerificador[NUMERO_DE_VERIFICADORES] = {false, false};
unsigned int ultimoProcesso = 0;
unsigned int contadorDeVerificacaoEsquerda = 0;
char buffer[N_CARACTERES+2];

void setup() {
  if(DEBUG){
     Serial.begin(9600);
     Serial.flush();
  }
  Serial.println("Inicio");
  pinMode(MOTOR_E1, OUTPUT);
  pinMode(MOTOR_E2, OUTPUT);
  pinMode(MOTOR_D1, OUTPUT);
  pinMode(MOTOR_D2, OUTPUT);
  pinMode(LED,      OUTPUT);
  pinMode(13, OUTPUT);
  
  for(int i = 0; i < NUMERO_DE_SENSORES; i++)
    pinMode(sensores[i], INPUT);
  pinMode(verificador_esquerda, INPUT);
  pinMode(verificador_direita, INPUT);
  calibrarSensores(10);
  delay(1000);

  while(lerSensor(verificador_esquerda) == true){
    digitalWrite(13, HIGH);
    Serial.print("Esperando.");
  }
  for(int c=0; c<5 ; c++){
    digitalWrite(13, LOW);
    delay(50);
    digitalWrite(13, HIGH);
    delay(50);
  }
  ti = millis();
  digitalWrite(13, LOW);
}

void loop() { 
   
    deltaTime = (millis() - ultimoProcesso);
  
    ultimoProcesso = millis();
    
    erro = lerPontoAtual();
    analizadorDeVerificadores();
    correcao = TipoDeCorrecao(tipo); // editar tipo...
    if(tipo == RETA)
    { 
      digitalWrite(13, LOW);
      correcaoReta(correcao);
    }
    else
      if(tipo == CURVA)
      { 
        correcaoCurva(correcao);
        digitalWrite(13, HIGH);
      }
    erroAnterior = erro;
    prevenirWindUp();
    somatorioDeErro += erro;
    if (erro == 0) 
      somatorioDeErro = 0; 
    if(parada) 
      para();

 // if(verificaMarcacao(DIREITA))
 // {para();}
    if(millis() - ti >= TEMPO_PARA_FINAL){
      para();
    if(DEBUG)
      delay(DELAY);
 }
}

/*################################# FUNÇÕES DE LEITURA ########################################### */
//Ler sensor. Recebe a porta que ele deve ler e retorna true se o sensor está fora da linha (na mesa)
void calibrarSensores(int numeroDeIteracoes) 
  {
  //Lê a media de luminosidade na mesa e na linha. Esses valores definem a cor da mesa
  unsigned int mediaDaMesa = 0;
  unsigned int mediaDaLinha = 0;

  digitalWrite(13, HIGH);
  delay(50);
  digitalWrite(13, LOW);

  for (int i = 0; i < numeroDeIteracoes; i++) 
  {
    mediaDaMesa += (analogRead(verificador_esquerda));
  }
 
 for (int i = 0; i < numeroDeIteracoes; i++)
 {
    mediaDaLinha += analogRead(sensores[3]);
 }

  mediaDaMesa /= numeroDeIteracoes;
  mediaDaLinha /= numeroDeIteracoes;
  //O limiar é a média das médias
  limiar = (mediaDaMesa + mediaDaLinha) / 8;
//    limiar = LIMIAR;
  Serial.print("Media da mesa : ");
  Serial.println(mediaDaMesa);
  Serial.print("Media da linha: ");   
  Serial.println(mediaDaLinha);
  Serial.print("Limite        : ");    
  Serial.println(limiar);  
  if (mediaDaMesa > mediaDaLinha)  
  {
    corDaLinha = BRANCA;
  }
  else 
  {
    corDaLinha = PRETA;
  }
}
bool lerSensor(int porta) {
  int leitura = 0;
  
  if(porta>13)
  { 
    leitura = analogRead(porta);
    if ((corDaLinha == PRETA && leitura > limiar) || (corDaLinha == BRANCA && leitura < limiar)) {
      return true;
      Serial.print(analogRead(porta));   
    }
    else {
      return false;
    }
  }
  else
  {  
    leitura = digitalRead(porta);
    if ((corDaLinha == PRETA && leitura == true) || (corDaLinha == BRANCA && leitura == false)) {
      return true;
    }
    else {
      return false;
    }
  }
}
//Faz uma analize sobre a leitura dos verificadores

void analizadorDeVerificadores()
{
    bool esquerda = 0; 
    bool direita  = 0;
    esquerda = lerSensor(verificador_esquerda);
    direita  = lerSensor(verificador_direita);

    if((lerSensor(A4) == true && lerSensor(A3)== true) && lerSensor(A2) == true)
    {
       if(millis() - tempo_ultimo_cruzamento >= TEMPO_LIMITE_ERRO_MARCACAO) {
          flagDeCruzamento = false;
          tempo_ultimo_cruzamento = millis();
       }
    }
    else
    if(esquerda) { 
       if(millis() - tempo_ultima_marcacao >= TEMPO_LIMITE_ERRO_MARCACAO) {
          if(flagDeCruzamento == true)
             flagDeCruzamento = false;
          else if(conta!= 7 && conta!= 12){
             tipo *= -1;
        }
        conta++;
        tempo_ultima_marcacao = millis();
       }
    }
    else
    { 
     if(conta == ULTIMA_MARCACAO_ESQUERDA){ 
      if(direita)
      {
        if(flagDeCruzamento == false)
        {
          para();
        }
      }
    }
   }  
}
//Lê os sensores guias e retorna valores maiores que 0 para direita e menor que zero para esquerda
float lerPontoAtual() {
  int numerador = 0;
  int denominador = 0;

  for (int i = 0; i < NUMERO_DE_SENSORES; i++) {
    if (lerSensor(sensores[i])) {
      numerador += erros[i];
      denominador++;
    }
  }

  if (denominador == 0) 
    denominador = 1;

  return numerador / denominador;
}
/*
bool verificaMarcacao(int lado) {
  bool viuMarcacao = lerSensor(verificador);

  if(viuMarcacao) {
    contadorDeMarcacao[lado]++;
  }
  else {
    flagDeVerificador[lado] = false;
    contadorDeMarcacao[lado] = 0;
  }

  if(contadorDeMarcacao[lado] > TESTE_DE_MARCACAO && !flagDeVerificador[lado]) {
    flagDeVerificador[lado] = true;
    return true;
  }
  else {
    return false;
  }
}
*/
/*################################### FUNÇÕES DE CONTROLE ############################################### */
void prevenirWindUp() {
    if(somatorioDeErro >= VELOCIDADE_MAXIMA)
        somatorioDeErro = somatorioDeErro / 3;
}
float TipoDeCorrecao(int tipo)
{
  if(tipo == RETA)
    return (kpr * erro) + (kdr * (erro - erroAnterior)/deltaTime/1000.00) + (kir * somatorioDeErro*deltaTime/1000.00);
  else
    if(tipo == CURVA)
      return (kpc * erro) + (kdc * (erro - erroAnterior)/deltaTime/1000.00) + (kic * somatorioDeErro*deltaTime/1000.00); 
}
void correcaoCurva(int correcao_curva) {
  if(erro > 0) {
    Serial.println("Erro positivo ");
    esquerdo = velocidadeAtualCurva + correcao_curva;
    direito = velocidadeAtualCurva - correcao_curva;
  }
  else if(erro < 0) {
    Serial.println("Erro negativo");
    esquerdo = velocidadeAtualCurva + correcao_curva;
    direito = velocidadeAtualCurva - correcao_curva;
  }
  else {
    Serial.println("Sem erro");  
    esquerdo = velocidadeAtualCurva;
    direito = velocidadeAtualCurva;
  }
  motorEsquerdo(esquerdo);
  motorDireito(direito);
  Serial.print("Motor Esquerdo: ");
  Serial.println(esquerdo);    
  Serial.print("Motor Direito : ");
  Serial.println(direito);
  Serial.print("Correcao : ");
  Serial.println(correcao_curva);  
}

void correcaoReta(int correcao_reta) {
  if(erro > 0) {
    Serial.println("Erro positivo");
    direito  = velocidadeAtualReta;
    esquerdo = velocidadeAtualReta + correcao_reta;
  }
    else if(erro < 0) {
      Serial.println("Erro negativo"); 
      direito = velocidadeAtualReta - correcao_reta;    
      esquerdo = velocidadeAtualReta;
    }
      else
      {
        Serial.println("Sem erro");  
        direito = velocidadeAtualReta;
        esquerdo = velocidadeAtualReta;
      }
  motorEsquerdo(esquerdo);
  motorDireito(direito);
  Serial.print("Motor Esquerdo: ");
  Serial.println(esquerdo);    
  Serial.print("Motor Direito : ");
  Serial.println(direito);
  Serial.print("Correcao : ");
  Serial.println(correcao_reta);  
}

/*#################################### FUNÇÕES DE PARADA ##########################################*/
/*
bool devePararPorMarcacao() {
  bool teste = false;
  float t0 = millis(); 
  if((millis() - t0) > 5000) {
    bool viuMarcacao = verificaMarcacao(DIREITA);
    
    if(viuMarcacao) {
        marcacaoVista = true;
    }
    if(marcacaoVista && !verificaMarcacao(ESQUERDA)) {
      contadorDeVerificacaoEsquerda++;
    }
    else if(marcacaoVista) {
      marcacaoVista = false;
      contadorDeVerificacaoEsquerda = 0;
    }
    teste = (contadorDeVerificacaoEsquerda > LIMITE_TEMPO_ESQUERDA) && viuMarcacao;
  }
  else {
    teste = false;
  }
  if(teste) {
    tipoDeFinal = FINAL_POR_MARCACAO;
    return true;
  }
  else {
    return false;
  }
}
*/
/*
bool devePararPorContador() {
  if(verificaMarcacao(DIREITA)) {
    contadorDeFinal++;
  }
  
  bool teste = contadorDeFinal >= (2 + 2 * NUMERO_DE_CRUZAMENTOS);

  if(teste) {
    tipoDeFinal = FINAL_POR_CONTADOR;
    return true;
  }
  else {
    return false;
  }
}
*/
void para() 
{
  delay(TEMPO_PARA_PARAR);

    while(1)
    {
      digitalWrite(13, HIGH);
      motorEsquerdo(0);
      motorDireito(0);
      delay(100);
      digitalWrite(13, LOW);
      Serial.println(" Parado");
      delay(100);
    }
}
//############################### FUNÇÕES DOS MOTORES ##########################################

//Função para permitir apenas potências abaixo da máxima
int limitadorPotencia(int potencia) {
  int novaPotencia = potencia;
  if (abs(potencia) > VELOCIDADE_MAXIMA) {
    novaPotencia = (potencia / abs(potencia)) * VELOCIDADE_MAXIMA;
  }
  return novaPotencia;
}

//Controla o motor esquerdo para uma dada potência após ser limitada
void motorEsquerdo(int potencia) {
  int pwr = limitadorPotencia(potencia);
  
  if(DEBUG == 0){
    if (pwr < 0) {
      analogWrite(MOTOR_E1, abs(pwr));
      digitalWrite(MOTOR_E2, LOW);
    }
    else {
      analogWrite(MOTOR_E2, abs(pwr));
      digitalWrite(MOTOR_E1, LOW);
    }
  }
}

//Controla o motor direito para uma dada potência após ser limitada
void motorDireito(int potencia) {
  int pwr = limitadorPotencia(potencia);
  if(DEBUG==0){
    if (pwr < 0) {
      digitalWrite(MOTOR_D1, LOW);
      analogWrite(MOTOR_D2, abs(pwr));
    }
    else {
      digitalWrite(MOTOR_D2, LOW);
      analogWrite(MOTOR_D1, abs(pwr));
    }
  }
}


