#include<ESP8266WiFi.h>



const char* ssid = "CPH1707";    // This is Your Wifi SSID
const char* pwd = "12345678";   //This is Your WIFi Hotspot Password

WiFiServer server(5000);           // 5000 is Your Port Number on which communication take place
char data[250], dcr[250];          // For Data Recieve / Send
int ind = 0 , icd=0;

//WiFiClient client;                //Socket for incoming data
WiFiClient clients[2];


void setup() {
  Serial.begin(115200);             //For Arduino Serial Communication
  delay(10);
  WiFi.mode(WIFI_STA);            //WIFI Mode
  WiFi.begin(ssid,pwd);           //WiFi Setup
  Serial.print("Connecting");     //Arduino Will Print This Message
  while(WiFi.status()!=WL_CONNECTED){ //Until it get connected
    delay(500);
  }
  server.begin();                 //Server Will Start Listening
  Serial.print("Connected");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Check if a new client has connected
  WiFiClient newClient = server.available();
  if (newClient) {
    Serial.println("new client");
    // Find the first unused space
    for (int i=0 ; i<2 ; ++i) {
        if (NULL == clients[i]) {
            clients[i] = WiFiClient(newClient);
            break;
        }
     }
  }
//  if(!client.connected()){    // Try to get a Client
//    client= server.available();
//  }
  
//  else{                     // When Client is Connected
    if(Serial.available()>0){
      while(Serial.available()){
        dcr[icd] = Serial.read();   // Get Data From Arduino to Buffer
        icd++;
      }
      for(int j=0;j<icd;j++){
        clients[1].print(dcr[j]);       //printing the Recieve data from ardunio to WIFI Socket
      }
    }
    if(clients[0].available()>0){
      while(clients[0].available()){     //run until data is available for reading
        data[ind] = clients[0].read();    //reading data
        ind++;
      }
      clients[0].flush();
      for(int j=0;j<ind;j++){
        Serial.print(data[j]);      //printing/Sending Data to Ardunio
        clients[1].print(data[j]);
      }
        Serial.println();      
    }
    icd=0;            //reset the buffer counter
    ind=0;            //reset the buffer counter
  }

//



