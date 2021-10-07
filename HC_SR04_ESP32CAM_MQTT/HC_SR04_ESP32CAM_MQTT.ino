//*****************************************************************************
//                HC_SR04_ESP32CAM_MQTT V2.2
//
//    Programa que envía el valor de distancia medido a un dashboard utilizando
//  un sensor HC-SR04. La medición la envía un ESP32CAM a un broker por medio
//  de protocolo MQTT a un broker
//    Esta nueva versión, cambia la IP del broker que cambió (nueva IP 3,122,36,163)
//    Este programa se modifica el 4 de Oct del 2021
//
//    Versión modificada por:
//        Jorge Mguel Jaimes Ponce
//        Etstudiante del Diplomado de CodigoIoT
//
//
//    El programa base es proporcionado por:
//        Prof: Hugo Escalpelo
//              Diplomado IoT
//
//.............................................................................
//    Octubre 4 de 2021
//.............................................................................
//
//    Se utiiza la librería HCSR04.h
//    Pines para el sensor 12 y 13
//    Se crea una variable de tipo foltante "float distancia"
//    Para enviar la medición de distancia se crean nuevos tópicos
//    Topico que envía valor de distancia -> codigoiot/distancia/jjaimes
//    Topico que recibe valor del Led -> codigoiot/foco/jjaimes
//
//******************************************************************************


#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include <HCSR04.h>

UltraSonicDistanceSensor distanceSensor(12, 13);  // Initialize sensor that uses digital pins 12 (Trig) and 13 (Echo)

float distancia = 0;


unsigned long t_0 = 0;
unsigned long t_1 = 0;
byte  medir = 0;

/*
 * Conexión básica por MQTT del NodeMCU
 * por: Hugo Escalpelo
 * Fecha: 28 de julio de 2021
 * 
 * Este programa envía datos  por Internet a través del protocolo MQTT. Para poder
 * comprobar el funcionamiento de este programa, es necesario conectarse a un broker
 * y usar NodeRed para visualzar que la información se está recibiendo correctamente.
 * Este programa no requiere componentes adicionales.
 * 
 * Componente     PinESP32CAM     Estados lógicos
 * ledStatus------GPIO 33---------On=>LOW, Off=>HIGH
 * ledFlash-------GPIO 4----------On=>HIGH, Off=>LOW
 */

//Bibliotecas
#include <WiFi.h>  // Biblioteca para el control de WiFi
#include <PubSubClient.h> //Biblioteca para conexion MQTT

//Datos de WiFi
const char* ssid = "*********";  // Aquí debes poner el nombre de tu red
const char* password = "************";  // Aquí debes poner la contraseña de tu red

//Datos del broker MQTT
const char* mqtt_server = "3.122.36.163"; // Si estas en una red local, coloca la IP asignada, en caso contrario, coloca la IP publica
IPAddress server(3,122,36,163);

// Objetos
WiFiClient espClient; // Este objeto maneja los datos de conexion WiFi
PubSubClient client(espClient); // Este objeto maneja los datos de conexion al broker

// Variables
int flashLedPin = 4;  // Para indicar el estatus de conexión
int statusLedPin = 33; // Para ser controlado por MQTT
long timeNow, timeLast; // Variables de control de tiempo no bloqueante
float data = 0; // Contador
int wait = 5000;  // Indica la espera cada 5 segundos para envío de mensajes MQTT

// Inicialización del programa
void setup() {
  // Iniciar comunicación serial
  Serial.begin (115200);
  pinMode (flashLedPin, OUTPUT);
  pinMode (statusLedPin, OUTPUT);
  digitalWrite (flashLedPin, LOW);
  digitalWrite (statusLedPin, HIGH);

  Serial.println();
  Serial.println();
  Serial.print("Conectar a ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password); // Esta es la función que realiz la conexión a WiFi
 
  while (WiFi.status() != WL_CONNECTED) { // Este bucle espera a que se realice la conexión
    digitalWrite (statusLedPin, HIGH);
    delay(500); //dado que es de suma importancia esperar a la conexión, debe usarse espera bloqueante
    digitalWrite (statusLedPin, LOW);
    Serial.print(".");  // Indicador de progreso
    delay (5);
  }
  
  // Cuando se haya logrado la conexión, el programa avanzará, por lo tanto, puede informarse lo siguiente
  Serial.println();
  Serial.println("WiFi conectado");
  Serial.println("Direccion IP: ");
  Serial.println(WiFi.localIP());

  // Si se logro la conexión, encender led
  if (WiFi.status () > 0){
  digitalWrite (statusLedPin, LOW);
  }
  
  delay (1000); // Esta espera es solo una formalidad antes de iniciar la comunicación con el broker

  //Serial.end();

  // Conexión con el broker MQTT
  client.setServer(server, 1883); // Conectarse a la IP del broker en el puerto indicado
  client.setCallback(callback); // Activar función de CallBack, permite recibir mensajes MQTT y ejecutar funciones a partir de ellos
  //delay(1523);  // Esta espera es preventiva, espera a la conexión para no perder información
  delay(323);  // Esta espera es preventiva, espera a la conexión para no perder información

  timeLast = millis (); // Inicia el control de tiempo

  t_0 = millis();
  
}// fin del void setup ()

// Cuerpo del programa, bucle principal
void loop() {

  
// Sección de programa que genera un tiempo de espera para la medición. Utiiza función millis()
    t_1 = millis();
    
    if(t_1 - t_0 >= 1000){
      t_0 = t_1; 
      medir = 1;       
    }
    
    if(medir == 1){
       distancia = distanceSensor.measureDistanceCm();
       Serial.print(distancia);
       Serial.println("   cm   ");
       delay(2000);
       medir = 0;
    }

  
  //Verificar siempre que haya conexión al broker
  if (!client.connected()) {
    reconnect();  // En caso de que no haya conexión, ejecutar la función de reconexión, definida despues del void setup ()
  }// fin del if (!client.connected())
  client.loop(); // Esta función es muy importante, ejecuta de manera no bloqueante las funciones necesarias para la comunicación con el broker
  
  timeNow = millis(); // Control de tiempo para esperas no bloqueantes
  if (timeNow - timeLast > wait) { // Manda un mensaje por MQTT cada cinco segundos
    timeLast = timeNow; // Actualización de seguimiento de tiempo


    char dataString[8]; // Define una arreglo de caracteres para enviarlos por MQTT, especifica la longitud del mensaje en 8 caracteres

    data = distancia;
    dtostrf(data, 1, 2, dataString);
    client.publish("codigoiot/distancia/jjp",dataString); // Esta es la función que envía los datos por MQTT, especifica el tema y el valor
    
  }// fin del if (timeNow - timeLast > wait)
}// fin del void loop ()

// Funciones de usuario

// Esta función permite tomar acciones en caso de que se reciba un mensaje correspondiente a un tema al cual se hará una suscripción
void callback(char* topic, byte* message, unsigned int length) {

  // Indicar por serial que llegó un mensaje
  Serial.print("Llegó un mensaje en el tema: ");
  Serial.print(topic);

  // Concatenar los mensajes recibidos para conformarlos como una varialbe String
  String messageTemp; // Se declara la variable en la cual se generará el mensaje completo  
  for (int i = 0; i < length; i++) {  // Se imprime y concatena el mensaje
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  // Se comprueba que el mensaje se haya concatenado correctamente
  Serial.println();
  Serial.print ("Mensaje concatenado en una sola variable: ");
  Serial.println (messageTemp);

  // En esta parte puedes agregar las funciones que requieras para actuar segun lo necesites al recibir un mensaje MQTT

  // Ejemplo, en caso de recibir el mensaje true - false, se cambiará el estado del led soldado en la placa.
  // El ESP323CAM está suscrito al tema esp/output
  if (String(topic) == "codigoiot/foco/jjp") {  // En caso de recibirse mensaje en el tema esp32/output
    if(messageTemp == "true"){
      Serial.println("Led encendido");
      digitalWrite(flashLedPin, HIGH);
    }// fin del if (String(topic) == "codigoiot/foco/jjaimes")
    else if(messageTemp == "false"){
      Serial.println("Led apagado");
      digitalWrite(flashLedPin, LOW);
    }// fin del else if(messageTemp == "false")
  }// fin del if (String(topic) == "codigoiot/foco/jjaimes")
}// fin del void callback

// Función para reconectarse
void reconnect() {
  // Bucle hasta lograr conexión
  while (!client.connected()) { // Pregunta si hay conexión
    Serial.print("Tratando de contectarse...");
    // Intentar reconexión
    if (client.connect("ESP32CAMClient")) { //Pregunta por el resultado del intento de conexión
      Serial.println("Conectado");
      client.subscribe("codigoiot/foco/jjp"); // Esta función realiza la suscripción al tema
    }// fin del  if (client.connect("ESP32CAMClient"))
    else {  //en caso de que la conexión no se logre
      Serial.print("Conexion fallida, Error rc=");
      Serial.print(client.state()); // Muestra el codigo de error
      Serial.println(" Volviendo a intentar en 5 segundos");
      // Espera de 5 segundos bloqueante
      delay(493);
      Serial.println (client.connected ()); // Muestra estatus de conexión
    }// fin del else
  }// fin del bucle while (!client.connected())
}// fin de void reconnect()
