/* 
**** EEPROM MEMORY MARKING ****
* #0 - The flag of the first launch, if the number 25 is saved in memory, then the first launch was already there, if not, then it was not
* #1 - #128 - Device settings (32 parameters, 4 bytes each, including config ID)
* #129 - #3872 - Events store (stamps + time) 3744 bytes
*		holds 468 records of 2 numbers by 4 bytes in the following order:
*		1 - [Timestamp]
*		2 - [UID]
* #3873 - #3900 - 28 bytes of reserve, in case the data store suddenly goes out of memory due to an error in the code
* #3901-3904 - The number of records in the storage at the moment
* #3905 - CRC sum of currently saved settings
*/

#include <DS3231.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SimpleTimer.h>
#include <MFRC522.h>
#include <OneWire.h>
#include <MemoryFree.h>
#include <pgmStrToRAM.h>
#include <EEPROM.h>


#define addrStartStorage 129			// Address of the first memory location for the event store
#define addrCountRecordsStorage 3901	// 3901 - 3904 Cell address with the number of entries in the storage at the moment
#define addrCRCsettings 3905			// Memory cell address with CRC 8 bit sum of the currently stored settings

bool FIRST_RUN = 0;  // First start. Do not touch unnecessarily, at the first start it will set itself to TRUE (FIRST start erase all data in EEPROM memory)

#define DEVICE_ID 1					// Unique device ID
/////////////////////
// DEFAULT SETTINGS (these settings will be updated after syncing with the server or after reading from EEPROM)
long idSettings = 1;				// Version of settings for update from server
long a_debuging = 1;				// Debug mode
long b_PriorityServer = 1;			// Priority server ID for sending data
long c_CountStorageRecords = 468;		// Limit the number of event records in EEPROM, in case the server is unavailable (0 - disable saving events)
long d_ip1 = 192;					// ******
long e_ip2 = 168;					// * IP	
long f_ip3 = 0;						// * address
long g_ip4 = 27;					// ******
long h_DHCP = 0;					// 1 - DHCP ON, 0 - DHCP - off
// DEFAULT SETTINGS
/////////////////////

#define storageMaxRecsCount 468		// Maximum number of events in storage EEPROM
#define pinBuzzer 3

// Servers for sending data
enum 
{
	rfidServer1,
	rfidServer2,
	countSERVERS
};

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };	//MAC address
IPAddress ip(d_ip1, e_ip2, f_ip3, g_ip4);				//IP address (ipv4)
byte dns1[] = { 8, 8, 8, 8 };							//DNS server (ipv4)
byte gateway[] = {192, 168, 0, 1 };						//Gateway
char server1[] = "myserver1.com";						//Server1 Address
char server2[] = "myserver2.com";						//Server2 Address

unsigned long currentTime = 0; //Current timestamp from RTC module

EthernetClient client;
DS3231 rtc(SDA, SCL);
MFRC522 rfid(7, 9); // 9 port - RST, 7 port - SDA

#define countTimers 4
int timers[countTimers]=
{
	0,	//	sendCurentData (ID timer of simpleTimer)
	0,	//	sendStorageData (ID timer of simpleTimer)
	0,	//	syncTime (ID timer of simpleTimer)
	0	//	checkRFID (ID timer of simpleTimer)
};

SimpleTimer timer;

int _sendStep = 0;			// Current step of sending data (1 - ethernet module is ready to send data; 2 - sending data to the server; 3 - geting a response from server)
bool isBusyRFID = false;	// RFID is busy true|false

unsigned long getRespondTime = 0; 	// How much time has passed since the request was sent (msec)
int timerGetRespond = 0; 			// Timer ID from simpleTimer. This dynamic timer is used to receive a response from the server

unsigned long last_uTime = 0;	// Timestamp (Unix) of last RFID
unsigned long last_UID = 0;		// Last RFID serial of tag

unsigned long uTime = 0;	// Current timestamp (Unix)
unsigned long UID = 0;		// Current RFID serial of tag

void setup() {
	Serial.begin(115200);
	while (!Serial) {
		; // Wait for serial port to connect
	}
	
	checkSettingsCRC(); // Checking data integrity
	if (FIRST_RUN==1) { // If this is the first launch, then clear the memory
		for (int i = 0 ; i < 4096; i++) {
			EEPROM.write(i, 0);
		}
		saveSettings();
	}
	loadSettings();
	
	
	SPI.begin(); 		// Initializing SPI
	rfid.PCD_Init(); 	// Initializing RFID
	ethernetStart();	// Initializing Ethernet
	rtc.begin();		// Initializing RTC module
	
	//Running periodic tasks
	timers[0]=timer.setInterval(500, sendCurentData);	// Check and send current tag with timestamp, every 0.5 sec
	timers[1]=timer.setInterval(5000, sendStorageData);	// Check and send tags and timestamps from storage, every 5 sec
	timers[2]=timer.setInterval(3600000, syncTime);		// Time synchronization with the server, every hour
	timers[3]=timer.setInterval(50, checkRFID);			// Сheck if the card was presented, every 50 msec
}

void loop() {
	timer.run();
}

/////////////////////////////////////////////
// FUNCTIONS FOR SENDING AND RECEIVING DATA
//

/*	function send request to the server
 *
 *	typeSend //type of transmitted data
 *		0 - time synchronization with the server
 *		1 - sending the current RFID tag
 *		2 - sending tags from storage
 */
void _sendRequestServer(byte typeSend=0) {
	if (_sendStep<2) {
		_sendStep = 2;
		char *server;
		switch(b_PriorityServer-1) {
			case 	rfidServer1:
				server = server1;
			break;
			case 	rfidServer2:
				server = server2;
			break;
		}
		
		if (client.connect(server, 80)) {
			
			String serverAddr = "";
			serverAddr += getString(PSTR("Host: "))+String(server);
			
			String request = getString(PSTR("GET /?id="));
			request += String(DEVICE_ID)+
								"&is="+String(idSettings)+
								"&t="+String(getTime());
			
			// If we send the current RFID tag
			if (typeSend==1) {
				request +=
				"&ui="+String(UID)+ 	// RFID card serial ID
				"&ut="+String(uTime); 	// Timestamp
			}
			
			// If we send a record from a EEPROM storage
			if (typeSend==2) {
				long countRecsStorage = EEPROMReadlong(addrCountRecordsStorage);
				// If save is enabled and the count of saved records is more than 0
				if (c_CountStorageRecords > 0 && countRecsStorage>0) {
					request +=
						"&dd="+String(countRecsStorage)+  // Count of records in memory
						"&ui="+String(getRecordFromStorage(2, false))+ // RFID card serial ID
						"&ut="+String(getRecordFromStorage(1, false)); // Timestamp
				} 
			}
			
			request += getString(PSTR(" HTTP/1.1"));
			debuglogS(PSTR("serverAddr:")); debuglog(serverAddr); debuglog();
			debuglogS(PSTR("request:")); debuglog(request); debuglog();
			debuglogS(PSTR("connected")); debuglog();
			client.println(request);
			client.println(serverAddr);
			client.println("Connection: close");
			client.println();
			
			if (!timerGetRespond) timerGetRespond=timer.setInterval(50, getRespond);
			getRespondTime = millis();
			
		} else {
			debuglogS(PSTR("connection failed")); debuglog();
			if (isBusyRFID) {
				saveToStorage();
			}
			_sendStep=1;
			
			//Change priority server
			switch(b_PriorityServer-1) {
				case 	rfidServer1:
					b_PriorityServer = rfidServer2-1;
				break;
				case 	rfidServer2:
					b_PriorityServer = rfidServer1-1;
				break;
			}
		}
	}
}

// Function gets server time
void syncTime() {
	_sendRequestServer(0);
}

// Function send current RFID card serial ID and current timestamp to the server
void sendCurentData() {
	if (isBusyRFID && UID>0 && uTime>0) {
		_sendRequestServer(1);
	}
}

// Function send saved RFID card serial IDs and timestamps  to the server
void sendStorageData() {
	long countRecsStorage = EEPROMReadlong(addrCountRecordsStorage);
	debuglogS(PSTR("countRecsStorage: ")); debuglog(String(countRecsStorage)); debuglog();
	// If save is enabled and the count of saved records is more than 0
	if (c_CountStorageRecords > 0 && countRecsStorage>0) {
		//If saved records is more than 0
		if (countRecsStorage>1) {
			// Delete the created timer (of simpleTimer) and increase the frequency of check updates
			timer.deleteTimer(timers[1]);
			timers[1]=timer.setInterval(1000, sendStorageData);
		//Otherwise, we slow down the frequency of sends
		} else {
			timer.deleteTimer(timers[1]);
			timers[1]=timer.setInterval(5000, sendStorageData);
		}
		
		_sendRequestServer(2);
	} else {
		
	}
}

// Function get respond from server
void getRespond() {
	if ((millis()-getRespondTime)>200) {
		_sendStep=3;
		if (client.available()>0) {
			int _crc=0;
			while(client.available()>0) {
				if (client.find("CRC:")) {
					_crc = client.parseInt();
					break;						
				}
			}
			
			while(client.available()>0) {
				if (client.read() == '<') {
					debuglogS(PSTR("FIND START")); debuglog();
					break;						
				}
			}
			
			
			int _i = 0;
			String htmlBody = "";
			while(client.available()>0) {
				char symbol = client.read();
				if (symbol != 10 && symbol != 13 && symbol!=' ') {
					if (symbol=='>') {
						while(client.available()>0) {
							client.read();
						}
						debuglogS(PSTR("FIND END")); debuglog();
						break;
					}
					htmlBody += String(symbol);
					_i++;
				}
			}
			
			char htmlBodyChar[_i];
			htmlBody.toCharArray(htmlBodyChar, _i+1);
			
			debuglog(String(htmlBodyChar)); debuglog();
			debuglog("CRC "+String(_crc)+" = "+String(OneWire::crc8((uint8_t*)htmlBodyChar, _i-1)));
			
			if (_crc == OneWire::crc8((uint8_t*)htmlBodyChar, _i-1)) {
				updateSettings(htmlBody);
			}
		}
			
			
		if (!client.connected()) {
			Serial.println();
			debuglogS(PSTR("disconnecting.")); debuglog();
			client.stop();
			
			_sendStep=1;
			timer.deleteTimer(timerGetRespond);
			timerGetRespond=0;
		}
	}
}

// Init ethernet module
void ethernetStart() {
	if (h_DHCP==1) {
		if (Ethernet.begin(mac) == 0) {
			debuglogS(PSTR("Failed to configure Ethernet using DHCP"));  debuglog();
			Ethernet.begin(mac, ip, dns1, gateway);
			_sendStep = 1;
		} 
	} else {
		Ethernet.begin(mac, ip, dns1, gateway);
		_sendStep = 1;
	}
	
	delay(1000);
	debuglogS(PSTR("connecting..."));  debuglog();
}

//
// FUNCTIONS FOR SENDING AND RECEIVING DATA
/////////////////////////////////////////////


/////////////////////////////////////
// LOAD AND SAVE SETTINGS FUNCTIONS
//

// Function save settings from server
void updateSettings(String htmlBody) {
	debuglogS(PSTR("updateSettings")); debuglog();
	int addrChar=0; 
	int addrCharSleep=0;
	int addrCharTimeStamp=0;
	htmlBody = " "+String(htmlBody);
	
	//Get the symbol index number of the current year
	addrCharTimeStamp=htmlBody.indexOf("[yy");
	if (addrCharTimeStamp > 0) {
		int yy=0; //Year
		int mm=0; //Month
		int dd=0; //Day
		int hh=0; //Hour
		int ii=0; //Minute
		int ss=0; //Second
		
		yy = htmlBody.substring(addrCharTimeStamp+3, htmlBody.indexOf("yy]", addrCharTimeStamp)).toInt();
		//If we managed to parse the year, we get the rest of the data
		if (yy > 0) {
				addrCharTimeStamp=0;
				//Get the symbol index number of the current month
				addrCharTimeStamp=htmlBody.indexOf("[mm");
				if (addrCharTimeStamp > 0) {
					mm = htmlBody.substring(addrCharTimeStamp+3, htmlBody.indexOf("mm]", addrCharTimeStamp)).toInt();
				}
				addrCharTimeStamp=0;
				//Get the symbol index number of the current day
				addrCharTimeStamp=htmlBody.indexOf("[dd");
				if (addrCharTimeStamp > 0) {
					dd = htmlBody.substring(addrCharTimeStamp+3, htmlBody.indexOf("dd]", addrCharTimeStamp)).toInt();
				}
				addrCharTimeStamp=0;
				//Get the symbol index number of the current hour
				addrCharTimeStamp=htmlBody.indexOf("[hh");
				if (addrCharTimeStamp > 0) {
					hh = htmlBody.substring(addrCharTimeStamp+3, htmlBody.indexOf("hh]", addrCharTimeStamp)).toInt();
				}
				addrCharTimeStamp=0;
				//Get the symbol index number of the current minute
				addrCharTimeStamp=htmlBody.indexOf("[ii");
				if (addrCharTimeStamp > 0) {
					ii = htmlBody.substring(addrCharTimeStamp+3, htmlBody.indexOf("ii]", addrCharTimeStamp)).toInt();
				}
				addrCharTimeStamp=0;
				//Get the symbol index number of the current second
				addrCharTimeStamp=htmlBody.indexOf("[ss");
				if (addrCharTimeStamp > 0) {
					ss = htmlBody.substring(addrCharTimeStamp+3, htmlBody.indexOf("ss]", addrCharTimeStamp)).toInt();
				}
				
				//Update RTC-module time
				rtc.setTime(hh, ii, ss);
				rtc.setDate(dd, mm, yy);
		}
	}
	debuglogS(PSTR("isBusyRFID")); debuglog(String(isBusyRFID)); debuglog();
	//If the RFID module is busy
	if (isBusyRFID) {
		//If the server successfully received the current RFID serial ID
		if (htmlBody.indexOf("[uid_ok]") > 0) {
			//Clearing the variables and freeing the RFID module
			unsigned long uTime=0;
			unsigned long UID=0;
			isBusyRFID = false;
		//If the server did not accept the current RFID serial ID
		} else {
			//Save it to the EEPROM storage
			saveToStorage();
		}
	}
	
	//If the server successfully received record from storage
	if (htmlBody.indexOf("[d1d]") > 0) {
		//Delete last record from storage
		getRecordFromStorage(1, true);
	}

	//If the server issues a "forced save to storage" command
	if (htmlBody.indexOf("[d2d]") > 0) {
		//Save current RFID serial ID to the EEPROM storage
		saveToStorage();
	}
	
	
	//Get the settings ID
	addrChar=htmlBody.indexOf("{sid");
	if (addrChar > 0) {
		idSettings = htmlBody.substring(addrChar+4, htmlBody.indexOf("}", addrChar)).toInt();
		
		addrChar=htmlBody.indexOf("a_");
		a_debuging = htmlBody.substring(addrChar+2, htmlBody.indexOf("|", addrChar)).toInt();
		
		addrChar=htmlBody.indexOf("b_");
		b_PriorityServer = htmlBody.substring(addrChar+2, htmlBody.indexOf("|", addrChar)).toInt();
		
		addrChar=htmlBody.indexOf("c_");
		c_CountStorageRecords = htmlBody.substring(addrChar+2, htmlBody.indexOf("|", addrChar)).toInt();
		
		addrChar=htmlBody.indexOf("d_");
		d_ip1 = htmlBody.substring(addrChar+2, htmlBody.indexOf("|", addrChar)).toInt();
		
		addrChar=htmlBody.indexOf("e_");
		e_ip2 = htmlBody.substring(addrChar+2, htmlBody.indexOf("|", addrChar)).toInt();
		
		addrChar=htmlBody.indexOf("f_");
		f_ip3 = htmlBody.substring(addrChar+2, htmlBody.indexOf("|", addrChar)).toInt();
		
		addrChar=htmlBody.indexOf("g_");
		g_ip4 = htmlBody.substring(addrChar+2, htmlBody.indexOf("|", addrChar)).toInt();
		
		addrChar=htmlBody.indexOf("h_");
		h_DHCP = htmlBody.substring(addrChar+2, htmlBody.indexOf("|", addrChar)).toInt();
		
		//If received settings is not valid
		if ( idSettings<1 
				|| a_debuging>1 
				|| (b_PriorityServer<1 || b_PriorityServer>countSERVERS)
				|| (c_CountStorageRecords<0 || c_CountStorageRecords>storageMaxRecsCount)
				|| d_ip1<0 ||  d_ip1>255
				|| e_ip2<0 ||  e_ip2>255
				|| f_ip3<0 ||  f_ip3>255
				|| g_ip4<0 ||  g_ip4>255
				|| h_DHCP<0 ||  h_DHCP>1
			)
		{
			loadSettings();
			return;
		} else {
			saveSettings();
			loadSettings();
		}
	}
	
}

// Function load settings from EEPROM to RAM
void loadSettings() {	
	//If the first launch has already been
	if (EEPROM.read(0)==25) {
		if (!FIRST_RUN) { // If the first launch is not force enabled
		 FIRST_RUN = 0; 
		}
	} else {
		FIRST_RUN = 1;
		EEPROM.write(0, 25);
	}
	
	if (FIRST_RUN) {
		EEPROMWritelong(addrCountRecordsStorage, 0);
		debuglogS(PSTR("FIRST RUN ACTIVE")); debuglog();
	}
	
	
	unsigned int eeAddress=1;
	byte longSize=sizeof(long);
	idSettings = EEPROMReadlong(eeAddress);
	debuglogS(PSTR("idSettings: ")); debuglog(String(idSettings));
	
	eeAddress+=longSize;
	a_debuging = EEPROMReadlong(eeAddress);
	debuglogS(PSTR("a_debuging: ")); debuglog(String(a_debuging));
	
	eeAddress+=longSize;
	b_PriorityServer = EEPROMReadlong(eeAddress);
	debuglogS(PSTR("b_PriorityServer: ")); debuglog(String(b_PriorityServer));
	
	eeAddress+=longSize;
	c_CountStorageRecords = EEPROMReadlong(eeAddress);
	debuglogS(PSTR("c_CountStorageRecords: ")); debuglog(String(c_CountStorageRecords));
	
	eeAddress+=longSize;
	d_ip1 = EEPROMReadlong(eeAddress);
	debuglogS(PSTR("d_ip1: ")); debuglog(String(d_ip1));
	
	eeAddress+=longSize;
	e_ip2 = EEPROMReadlong(eeAddress);
	debuglogS(PSTR("e_ip2: ")); debuglog(String(e_ip2));
	
	eeAddress+=longSize;
	f_ip3 = EEPROMReadlong(eeAddress);
	debuglogS(PSTR("f_ip3: ")); debuglog(String(f_ip3));
	
	eeAddress+=longSize;
	g_ip4 = EEPROMReadlong(eeAddress);
	debuglogS(PSTR("g_ip4: ")); debuglog(String(g_ip4));
	
	eeAddress+=longSize;
	h_DHCP = EEPROMReadlong(eeAddress);
	debuglogS(PSTR("h_DHCP: ")); debuglog(String(h_DHCP));
	
}

// Function save settings from RAM to EEPROM
void saveSettings() {
	debuglogS(PSTR("saveSettings")); debuglog();
	byte longSize=sizeof(long);
	unsigned int eeAddress=1;
	String allSettings="";
	
	if (idSettings != EEPROMReadlong(eeAddress))
		EEPROMWritelong(eeAddress, idSettings);
	allSettings += String(idSettings);
	
	eeAddress+=longSize;
	if (a_debuging != EEPROMReadlong(eeAddress))
		EEPROMWritelong(eeAddress, a_debuging);
	allSettings += String(a_debuging);
		
	eeAddress+=longSize;
	if (b_PriorityServer != EEPROMReadlong(eeAddress))
		EEPROMWritelong(eeAddress, b_PriorityServer);
	allSettings += String(b_PriorityServer);
	
	eeAddress+=longSize;
	if (c_CountStorageRecords != EEPROMReadlong(eeAddress))
		EEPROMWritelong(eeAddress, c_CountStorageRecords);
	allSettings += String(c_CountStorageRecords);
	
	eeAddress+=longSize;
	if (d_ip1 != EEPROMReadlong(eeAddress))
		EEPROMWritelong(eeAddress, d_ip1);
	allSettings += String(d_ip1);
	
	eeAddress+=longSize;
	if (e_ip2 != EEPROMReadlong(eeAddress))
		EEPROMWritelong(eeAddress, e_ip2);
	allSettings += String(e_ip2);
	
	eeAddress+=longSize;
	if (f_ip3 != EEPROMReadlong(eeAddress))
		EEPROMWritelong(eeAddress, f_ip3);
	allSettings += String(f_ip3);
	
	eeAddress+=longSize;
	if (g_ip4 != EEPROMReadlong(eeAddress))
		EEPROMWritelong(eeAddress, g_ip4);
	allSettings += String(g_ip4);
	
	eeAddress+=longSize;
	if (h_DHCP != EEPROMReadlong(eeAddress))
		EEPROMWritelong(eeAddress, h_DHCP);
	allSettings += String(h_DHCP);
	
	int settingsLength = allSettings.length();
	char allSettingsChar[settingsLength];
	allSettings.toCharArray(allSettingsChar, settingsLength+1);
	
	// Save CRC sum
	EEPROM.write(addrCRCsettings, OneWire::crc8((uint8_t*)allSettingsChar,settingsLength-1)); 
	debuglogS(PSTR("SUMM: "));
	debuglog(String(OneWire::crc8((uint8_t*)allSettingsChar,settingsLength-1)));
	
}

// The function checks if the settings in memory are damaged (calculate CRC sum of settings in EEPROM)
void checkSettingsCRC() {
	// Total number of parameters
	byte countParams = 9; // Provided that last parametr is h_DHCP
	unsigned int eeAddress=1;
	byte longSize=sizeof(long);
	String allSettings="";
	
	for(int i=0; i<countParams; i++) {
		allSettings += String(EEPROMReadlong(eeAddress+(i*longSize)));
	}
	
	int settingsLength = allSettings.length();
	char allSettingsChar[settingsLength];
	allSettings.toCharArray(allSettingsChar, settingsLength+1);
	
	// If the CRC amounts do not match, then we tell the device that this is the first launch, so that all settings are set to default
	if (OneWire::crc8((uint8_t*)allSettingsChar,settingsLength-1) != EEPROM.read(addrCRCsettings)) {
		FIRST_RUN = 1;
		debuglogS(PSTR("CRC INVALID ")); debuglog();
	} else {
		debuglogS(PSTR("CRC IS VALID ")); debuglog();
	}
	
	debuglogS(PSTR("CRC CURRENT: ")); debuglog(String(OneWire::crc8((uint8_t*)allSettingsChar,settingsLength-1)));
	debuglogS(PSTR("CRC SAVED: ")); debuglog(String(EEPROM.read(addrCRCsettings)));
}

//
// LOAD AND SAVE SETTINGS FUNCTIONS
/////////////////////////////////////


//////////////////////////////////////////
// READ AND WRITE TAGs RECORDS FUNCTIONS
//

// Save readed tag and timestamp to EEPROM storage
void saveToStorage() {
	debuglogS(PSTR("saveToStorage")); debuglog();
	byte longSize=sizeof(long);
	long countRecordsInStorage = EEPROMReadlong(addrCountRecordsStorage); // Getting the number of records in the storage
	//If the storage is full, then exit the function
	if (countRecordsInStorage >= c_CountStorageRecords) return;
	unsigned int eeAddress = addrStartStorage + countRecordsInStorage * 2 * longSize; // Get the address of a free cell
	
	//1 - Save the timestamp
	EEPROMWritelong(eeAddress, uTime);
	
	//2 - Save the RFID serial ID
	eeAddress+=longSize;
	EEPROMWritelong(eeAddress, UID);
	
	// Updating information about the amount of records in the storage
	EEPROMWritelong(addrCountRecordsStorage, countRecordsInStorage+1);
	
	delay(200); //We pause so that we can write to EEPROM memory
	
	uTime = 0;
	UID = 0;
	isBusyRFID = false;
}

/* The function read record from storage and delete it
 *	cellAddr = 
 *		1 - [Timestamp]
 *		2 - [RFID serial ID]
 *
 *	clean = (true|false) - Clear last cell 
 */
unsigned long getRecordFromStorage(byte cellAddr, bool clean) {
	debuglogS(PSTR("getRecordFromStorage")); debuglog();
	long countRecordsInStorage = EEPROM.read(addrCountRecordsStorage); // Get the count of records in the storage
	//If there are still records in the storage
	if (countRecordsInStorage>0) {
		//If the command is given to clear the last cell
		if (clean) {
			//Updating information about the count of records in the storage
			EEPROMWritelong(addrCountRecordsStorage, countRecordsInStorage-1);
		}
		// If the cell address was specified correctly, then we return the value of this cell from the last record
		if (cellAddr > 0 && cellAddr < 3) {
			unsigned long preValue = EEPROMReadlong(addrStartStorage + ((countRecordsInStorage-1) * 2 * sizeof(long)) + ((cellAddr-1)*sizeof(long)));
			return preValue;
		} else {
			return 0;
		}
	//If there are no records in the repository	
	} else {
		return 0;
	}
	
}

//
// READ AND WRITE TAGs RECORDS FUNCTIONS
//////////////////////////////////////////


////////////////////
// OTHER FUNCTIONS
//

// The function scans the RFID card and saves the result in the "UID" variable and saves current timestamp in the "uTime" variable
void checkRFID() {
	if (!isBusyRFID) {
		isBusyRFID = true;
		// If the key is brought to the reader 
		if ( rfid.PICC_IsNewCardPresent()) {
			// If it was possible to read the serial number of the key
			if (rfid.PICC_ReadCardSerial()) {

				long four 	= rfid.uid.uidByte[3];
				long three 	= rfid.uid.uidByte[2];
				long two 	= rfid.uid.uidByte[1];
				long one 	= rfid.uid.uidByte[0];
				UID = ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
				uTime = getTime();

				// If the mark is brought up more often than once every 10 seconds, then we do nothing
				if (last_UID>0 && last_UID==UID && (uTime-last_uTime)<10) {
					UID = 0;
					uTime = 0;
					isBusyRFID = false;
					return;
				} 
				
				last_UID=UID;
				last_uTime=uTime;
				playTone();
				debuglogS(PSTR("UID:")); debuglog(String(UID)); debuglog();
			} else {
				isBusyRFID = false;
			}
		} else {
			isBusyRFID = false;
		}
	}
}

// Play beep
void playTone() {
	tone(pinBuzzer, 1620);
	timer.setTimeout(500, stopTone);
}

// Stop play beep
void stopTone() {
	noTone(pinBuzzer);
}

// Read time from RTC module and save it to "currentTime" variable
unsigned long getTime() {
	currentTime = rtc.getUnixTime(rtc.getTime());
	debuglogS(PSTR("UNIX:")); debuglog(String(currentTime)); debuglog();
	return currentTime;
}

// Function write 4 bytes int to EEPROM
void EEPROMWritelong(int address, long value)
{
	// Decomposition from a long to 4 bytes by using bitshift.
	// One = Most significant -> Four = Least significant byte
	byte four = (value & 0xFF);
	byte three = ((value >> 8) & 0xFF);
	byte two = ((value >> 16) & 0xFF);
	byte one = ((value >> 24) & 0xFF);

	// Write the 4 bytes into the EEPROM
	EEPROM.write(address, four);
	EEPROM.write(address + 1, three);
	EEPROM.write(address + 2, two);
	EEPROM.write(address + 3, one);
}

// Function read 4 bytes int to EEPROM
long EEPROMReadlong(long address)
{
	//Read the 4 bytes from the EEPROM
	long four = EEPROM.read(address);
	long three = EEPROM.read(address + 1);
	long two = EEPROM.read(address + 2);
	long one = EEPROM.read(address + 3);

	// Return the recomposed long by using bitshift.
	return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

// Function print debug static string from FLASH memory (if debug mode is ON)
void debuglogS(PGM_P str) {
	if (a_debuging) {
		char c;
		Serial.print("   ");
		while ((c = pgm_read_byte(str++)) != 0)
				Serial.print(c);
	}
}

// Function print debug value of variable (if debug mode is ON)
void debuglog(String str) {
	if (a_debuging) {
		Serial.println(str);
	}
}

// Function print debug line break (if debug mode is ON)
void debuglog(void) {
	if (a_debuging) {
		Serial.println();
	}
}

// Function get string from FLASH memory
String getString(PGM_P s) {
	char c;
	String returnString="";
	while ((c = pgm_read_byte(s++)) != 0)
			returnString+=String(c);
	return returnString;
}

//
// OTHER FUNCTIONS
////////////////////