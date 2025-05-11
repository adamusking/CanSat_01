#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESP32_FTPClient.h>
#include "octocat.h"

WiFiMulti wiFiMulti;

#define WIFI_SSID "Unicron"
#define WIFI_PASSWORD "Unicron1234rq-"

// add access info about the server
char ftp_server[] = "192.168.0.1";
char ftp_user[]   = "anonymous";
char ftp_pass[]   = "heslo";

// you can pass a FTP timeout and debbug mode on the last 2 arguments
ESP32_FTPClient ftp (ftp_server,ftp_user,ftp_pass, 5000, 2);

void setup()
{
    Serial.begin(9600);
    delay(10);

    // We start by connecting to a WiFi network
    wiFiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

    Serial.println();
    Serial.println();
    Serial.print("Waiting for WiFi... ");

    while(wiFiMulti.run() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    delay(500);

    ftp.OpenConnection();

    ftp.ChangeWorkDir("/images");
    ftp.InitFile("Type I");
    ftp.NewFile("octocat.jpg");
    ftp.WriteData( octocat_pic, sizeof(octocat_pic) );
    ftp.CloseFile();
    
    ftp.ChangeWorkDir("/data");  
    ftp.InitFile("Type A");
    ftp.NewFile("hello_world.csv"); 
    ftp.Write("Hello World");
    ftp.CloseFile();
  
    ftp.CloseConnection();
    Serial.println("All files uploaded successfully!");
}


void loop()
{

}