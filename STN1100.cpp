/* Original Copyright 2011 David Irvine @Loguino
 *
 * Loguino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Loguino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with Loguino.  If not, see <http://www.gnu.org/licenses/>.
 *
 * $Rev$
 * $Author$
 * $Date$

*/


#include "STN1100.h"

byte Stn1100::begin(){
    byte status;
    STN1100_PORT.begin(STN1100_BAUD_RATE);
    STN1100_PORT.setTimeout(STN1100_TIMEOUT);
    char data[20];
    runCommand("AT E0",data,20); //no echo 
    runCommand("AT S0",data,20); //no spaces
    return runCommand("AT SP 0",data,20); //auto protocol search
}

byte Stn1100::engineLoad(byte &load){
    byte status;
    byte values[1];
    status=getBytes("01","04",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    load=values[0]*100/255;
    return STN1100_SUCCESS;
}

byte Stn1100::coolantTemperature(int &temp){
    byte status;
    byte values[1];
    status=getBytes("01","05",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    temp=values[0]-40;
    return STN1100_SUCCESS;
}

byte Stn1100::getFuelTrim(const char *pid, int &percent){
    byte status;
    byte values[1];
    status=getBytes("01",pid,values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    percent=(values[0] - 128) * 100/128;
    return STN1100_SUCCESS;
}

byte Stn1100::fuelTrimBank1ShortTerm(int &percent){
    return getFuelTrim("06",percent);
}
byte Stn1100::fuelTrimBank1LongTerm(int &percent){
    return getFuelTrim("07",percent);
}
byte Stn1100::fuelTrimBank2ShortTerm(int &percent){
    return getFuelTrim("08",percent);
}
byte Stn1100::fuelTrimBank2LongTerm(int &percent){
    return getFuelTrim("09",percent);
}

byte Stn1100::fuelPressure(int &pressure){
    byte status;
    byte values[1];
    char mode[]="01";
    char chkMode[]="41";
    char pid[]="0A";
    status=getBytes("01","0A",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    pressure=values[0]*3;
    return STN1100_SUCCESS;
}

byte Stn1100::intakeManifoldAbsolutePressure(byte &pressure){
    byte status;
    byte values[1];
    status=getBytes("01","0B",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    pressure=values[0];
    return STN1100_SUCCESS;
}

byte Stn1100::engineRPM(int &rpm){
    byte status;
    byte values[2];
    status=getBytes("01","0C",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    rpm=((values[0]*256)+values[1])/4;
    return STN1100_SUCCESS;
}

byte Stn1100::vehicleSpeed(byte &speed){
    byte status;
    byte values[1];
    status=getBytes("01","0D",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    speed=values[0];
    return STN1100_SUCCESS;
}
byte Stn1100::timingAdvance(int &advance){
    byte status;
    byte values[1];
    status=getBytes("01","0E",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    advance=values[0]/2-64;
    return STN1100_SUCCESS;
}

byte Stn1100::intakeAirTemperature(int &temperature){
    byte status;
    byte values[1];
    status=getBytes("01","0F",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    temperature=values[0]-40;
    return STN1100_SUCCESS;
}

byte Stn1100::MAFAirFlowRate(unsigned int &rate){
    byte status;
    byte values[2];
    status=getBytes("01","10",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    rate = ((256 * values[0]) + values[1])/100 ;
    return STN1100_SUCCESS;
}

byte Stn1100::throttlePosition(byte &position){
    byte status;
    byte values[1];
    status=getBytes("01","11",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    position=(values[0]*100)/255;
    return STN1100_SUCCESS;
}

byte Stn1100::steeringWheelAngle(int &angle){
    byte status;
    byte values[1];
    status=getBytes("22","3201",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    angle = (6.25 * values[0]) - 800;
    return STN1100_SUCCESS;
}

byte Stn1100::o2SensorBank1Sensor1(byte &voltage, byte &trim){
    return o2sensorRead("14", voltage, trim);
}
byte Stn1100::o2SensorBank1Sensor2(byte &voltage, byte &trim){
    return o2sensorRead("15", voltage, trim);
}
byte Stn1100::o2SensorBank1Sensor3(byte &voltage, byte &trim){
    return o2sensorRead("16", voltage, trim);
}
byte Stn1100::o2SensorBank1Sensor4(byte &voltage, byte &trim){
    return o2sensorRead("17", voltage, trim);
}
byte Stn1100::o2SensorBank2Sensor1(byte &voltage, byte &trim){
    return o2sensorRead("18", voltage, trim);
}
byte Stn1100::o2SensorBank2Sensor2(byte &voltage, byte &trim){
    return o2sensorRead("19", voltage, trim);
}
byte Stn1100::o2SensorBank2Sensor3(byte &voltage, byte &trim){
    return o2sensorRead("1A", voltage, trim);
}
byte Stn1100::o2SensorBank2Sensor4(byte &voltage, byte &trim){
    return o2sensorRead("1B", voltage, trim);
}

byte Stn1100::o2sensorRead(const char *bank, byte &voltage, byte &trim){
    byte status;
    byte values[2];
    status=getBytes("01",bank,values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    voltage = values[0] / 200;
    trim=( values[1] * 100 ) / 128;
    return STN1100_SUCCESS;
}


byte Stn1100::auxiliaryInputStatus(bool &auxStatus){
    byte status;
    byte values[1];
    status=getBytes("01","1E",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    auxStatus=getBit(values[0], 1);
    return STN1100_SUCCESS;
}

byte Stn1100::engineRunTime(unsigned int &runTime){
    byte status;
    byte values[2];
    status=getBytes("01","1F",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    runTime=(values[0]*256)+values[1];
    return STN1100_SUCCESS;
}

byte Stn1100::distanceMIL(unsigned int &distance){
    byte status;
    byte values[2];
    status=getBytes("01","21",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    distance=(values[0]*256)+values[1];
    return STN1100_SUCCESS;
}

byte Stn1100::relativeFuelRailPressure(unsigned int &pressure){
    byte status;
    byte values[2];
    status=getBytes("01","22",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    pressure=((values[0]*256)+values[1])*0.079;
    return STN1100_SUCCESS;
}

byte Stn1100::absoluteFuelRailPressure(unsigned int &pressure){
    byte status;
    byte values[2];
    status=getBytes("01","23",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    pressure=((values[0]*256)+values[1])*10;
    return STN1100_SUCCESS;
}


byte Stn1100::o2S1WRVoltage(unsigned int &equivRatio, unsigned int &voltage){
    return o2WRVoltage("24", equivRatio, voltage);
}
byte Stn1100::o2S2WRVoltage(unsigned int &equivRatio, unsigned int &voltage){
    return o2WRVoltage("25", equivRatio, voltage);
}
byte Stn1100::o2S3WRVoltage(unsigned int &equivRatio, unsigned int &voltage){
    return o2WRVoltage("26", equivRatio, voltage);
}
byte Stn1100::o2S4WRVoltage(unsigned int &equivRatio, unsigned int &voltage){
    return o2WRVoltage("27", equivRatio, voltage);
}
byte Stn1100::o2S5WRVoltage(unsigned int &equivRatio, unsigned int &voltage){
    return o2WRVoltage("28", equivRatio, voltage);
}
byte Stn1100::o2S6WRVoltage(unsigned int &equivRatio, unsigned int &voltage){
    return o2WRVoltage("29", equivRatio, voltage);
}
byte Stn1100::o2S7WRVoltage(unsigned int &equivRatio, unsigned int &voltage){
    return o2WRVoltage("2A", equivRatio, voltage);
}
byte Stn1100::o2S8WRVoltage(unsigned int &equivRatio, unsigned int &voltage){
    return o2WRVoltage("2B", equivRatio, voltage);
}
byte Stn1100::o2WRVoltage(const char *sensor, unsigned int &equivRatio, unsigned int &voltage){
    byte status;
    byte values[4];
    status=getBytes("01",sensor,values,4);
    if (status != STN1100_SUCCESS){
        return status;
    }
    equivRatio=((values[0] * 256)+ values[1])*2/65535; // or ((A*256)+B)/32768
    voltage=((values[2]*256)+values[3])*8/65535;// or ((C*256)+D)/8192;
    return STN1100_SUCCESS;
}

byte Stn1100::commandedEGR(byte &egr){
    byte status;
    byte values[1];
    status=getBytes("01","2C",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    egr=(100 * values[0])/255;
    return STN1100_SUCCESS;
}

byte Stn1100::EGRError(int &error){
    byte status;
    byte values[1];
    status=getBytes("01","2D",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    error =(values[0]-128)*100/128;
    return STN1100_SUCCESS;
}

byte Stn1100::commandedEvaporativePurge(byte &purge){
    byte status;
    byte values[1];
    status=getBytes("01","2E",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    purge= (100 * values[0])/255;
    return STN1100_SUCCESS;
}

byte Stn1100::fuelLevel(byte &level){
    byte status;
    byte values[1];
    status=getBytes("01","2F",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    level= (100 * values[0])/255;
    return STN1100_SUCCESS;
}

byte Stn1100::warmUpsSinceLastCleared(byte &warmUps){
    byte status;
    byte values[1];
    status=getBytes("01","30",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    warmUps=values[0];
    return STN1100_SUCCESS;
}

byte Stn1100::distanceSinceLastCleared(unsigned int &distance){
    byte status;
    byte values[2];
    status=getBytes("01","31",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    distance=(values[0]*256) + values[1];
    return STN1100_SUCCESS;
}

byte Stn1100::evapPressure(int &pressure){
    byte status;
    byte values[2];
    status=getBytes("01","32",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    pressure=(256*(values[0]-128) + values[1])/4;
    return STN1100_SUCCESS;
}

byte Stn1100::barometricPressure(byte  &pressure){
    byte status;
    byte values[1];
    status=getBytes("01","33",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    pressure=values[0];
    return STN1100_SUCCESS;
}


byte Stn1100::o2WRCurrent(const char *sensor, unsigned int &equivRatio, int &current){
    byte status;
    byte values[4];
    status=getBytes("01",sensor,values,4);
    if (status != STN1100_SUCCESS){
        return status;
    }
    equivRatio=((values[0] * 256)+ values[1])*2/32768; 
    current=((values[2]*256)+values[3])*8/265-128;
    return STN1100_SUCCESS;
}

byte Stn1100::o2S1WRCurrent(unsigned int &equivRatio, int &current){
    return o2WRCurrent("34",equivRatio,current);
}
byte Stn1100::o2S2WRCurrent(unsigned int &equivRatio, int &current){
    return o2WRCurrent("35",equivRatio,current);
}
byte Stn1100::o2S3WRCurrent(unsigned int &equivRatio, int &current){
    return o2WRCurrent("36",equivRatio,current);
}
byte Stn1100::o2S4WRCurrent(unsigned int &equivRatio, int &current){
    return o2WRCurrent("37",equivRatio,current);
}
byte Stn1100::o2S5WRCurrent(unsigned int &equivRatio, int &current){
    return o2WRCurrent("38",equivRatio,current);
}
byte Stn1100::o2S6WRCurrent(unsigned int &equivRatio, int &current){
    return o2WRCurrent("39",equivRatio,current);
}
byte Stn1100::o2S7WRCurrent(unsigned int &equivRatio, int &current){
    return o2WRCurrent("3A",equivRatio,current);
}
byte Stn1100::o2S8WRCurrent(unsigned int &equivRatio, int &current){
    return o2WRCurrent("3B",equivRatio,current);
}

byte Stn1100::catalystTemperatureBank1Sensor1(int &temperature)
{
    return catTemperature("3C", temperature);
}
byte Stn1100::catalystTemperatureBank2Sensor1(int &temperature)
{
    return catTemperature("3D", temperature);
}
byte Stn1100::catalystTemperatureBank1Sensor2(int &temperature)
{
    return catTemperature("3E", temperature);
}
byte Stn1100::catalystTemperatureBank2Sensor2(int &temperature)
{
    return catTemperature("3F", temperature);
}

byte Stn1100::catTemperature(const char *sensor, int &temperature){
    byte status;
    byte values[2];
    status=getBytes("01",sensor,values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    temperature=((values[0]*256)+values[1])/10 - 40;
    return STN1100_SUCCESS;
}

byte Stn1100::controlModuleVoltage(unsigned int &voltage){
    byte status;
    byte values[2];
    status=getBytes("01","42",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    voltage=((values[0]*256)+values[1])/1000;
    return STN1100_SUCCESS;
}

byte Stn1100::absoluteLoadValue(unsigned int &load){
    byte status;
    byte values[2];
    status=getBytes("01","43",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    load=((values[0]*256)+values[1])*100/255;
    return STN1100_SUCCESS;
}

byte Stn1100::commandEquivalenceRatio(float &ratio){
    byte status;
    byte values[2];
    status=getBytes("01","44",values,2);
    if (status != STN1100_SUCCESS){
        return status;
    }
    ratio=((values[0]*256)+values[1])/32768;
    return STN1100_SUCCESS;
}


byte Stn1100::relativeThrottlePosition(byte &position){
    byte status;
    byte values[1];
    status=getBytes("01","45",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    position=(100*values[0])/255;
    return STN1100_SUCCESS;
}

byte Stn1100::ambientAirTemperature(int &temperature){
    byte status;
    byte values[1];
    status=getBytes("01","46",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    temperature=values[0]-40;
    return STN1100_SUCCESS;
}

byte Stn1100::absoluteThrottlePositionB(byte &position){
    byte status;
    byte values[1];
    status=getBytes("01","47",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    position=(100*values[0])/255;
    return STN1100_SUCCESS;
}
byte Stn1100::absoluteThrottlePositionC(byte &position){
    byte status;
    byte values[1];
    status=getBytes("01","48",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    position=(100*values[0])/255;
    return STN1100_SUCCESS;
}
byte Stn1100::acceleratorPedalPositionD(byte &position){
    byte status;
    byte values[1];
    status=getBytes("01","49",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    position=(100*values[0])/255;
    return STN1100_SUCCESS;
}
byte Stn1100::acceleratorPedalPositionE(byte &position){
    byte status;
    byte values[1];
    status=getBytes("01","4A",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    position=(100*values[0])/255;
    return STN1100_SUCCESS;
}
byte Stn1100::acceleratorPedalPositionF(byte &position){
    byte status;
    byte values[1];
    status=getBytes("01","4B",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    position=(100*values[0])/255;
    return STN1100_SUCCESS;
}
byte Stn1100::commandedThrottleActuator(byte &position){
    byte status;
    byte values[1];
    status=getBytes("01","4C",values,1);
    if (status != STN1100_SUCCESS){
        return status;
    }
    position=(100*values[0])/255;
    return STN1100_SUCCESS;
}


byte Stn1100::getBytes( const char *mode, const char *pid, byte *values, unsigned int numValues){
    char data[64];
    byte status;
    char hexVal[]="0x00";
    int i,j,t;
    int pid_length=strlen(pid);  
    char cmd[pid_length+3];      // the length of command is pid + 2 (mode) + 1 (null)

    cmd[0]=mode[0];              // we assume mode is always 2 chars
    cmd[1]=mode[1];
    for (int i=0; i<pid_length; i++) {
        cmd[i+2] = pid[i];
    }
    cmd[pid_length+2]='\0';

    status=runCommand(cmd,data,64);
    if ( status != STN1100_SUCCESS ) {
        return status;
    };

    //check the first 2 bytes, by validating the mode response
    if (data[0] - mode[0] != 4 || data[1] != mode[1]) {
        return STN1100_GARBAGE;
    }

    for (j=0; j<pid_length;j++) {
        if (data[j+2] != pid[j]) {
            return STN1100_GARBAGE;
        }
    }

    /*
    // Check the mode returned was the one we sent
    if ( data[0]!=chkMode[0] 
      or data[1]!=chkMode[1]
      or data[3]!=pid[0]
      or data[4]!=pid[1] ){
        return STN1100_GARBAGE;
    }*/
    
    // For each byte expected after the headers, pack it up
    for (t=0; t<numValues; t++){
        hexVal[2]=data[pid_length + 2 + (2*t)]; // i=0 > data[6], i=1 > data[8]
        hexVal[3]=data[pid_length + 3 + (2*t)]; // i=0 > data[7], i=1 > data[9]
        values[t]=strtol(hexVal,NULL,16);
    }

    return STN1100_SUCCESS;
}

byte Stn1100::runCommand(const char *cmd, char *data, unsigned int dataLength)
{   
    byte cmdLength;
    
    // Flush any leftover data from the last command.
    
    // Send the specified command to the controller.
    flush();
    STN1100_PORT.print(cmd);
    STN1100_PORT.print('\r');

    unsigned long timeOut;
    int counter;
    bool found;
    byte streamReadResponse;
    
    // Start reading the data right away and don't stop 
    // until either the requested number of bytes has 
    // been read or the timeout is reached, or the >
    // has been returned.
    //
    counter=0;
    timeOut=millis()+STN1100_TIMEOUT;
    found=false;
    //streamReadResponse = STN1100_PORT.readBytesUntil('>', data, dataLength);
    while (!found && counter<( dataLength ) && millis()<timeOut)
    {
        if ( STN1100_PORT.available() ){
            data[counter]=STN1100_PORT.read();
            if (  data[counter] == '>' ){
                found=true;
                data[counter]='\0';
            }else{
                ++counter;
            }
        }
    }

    // If there is still data pending to be read, raise OVERFLOW error.
    if (!found  && counter>=dataLength)
    {
        // Send a character, this should cancel any operation on the STN1100 device
        // so that it doesnt spuriously inject a response during the next 
        // command
        STN1100_PORT.print("XXXXXXXXX\r\r\r");
        delay(300);
        return STN1100_BUFFER_OVERFLOW;
    }
    
    // If not found, and there is still buffer space, then raise no response error.
    if (!found && counter<dataLength){
        // Send a character, this should cancel any operation on the STN1100 device
        // so that it doesnt spuriously inject a response during the next 
        // command
        STN1100_PORT.print("XXXXXXXXX\r\r\r");
        delay(300);
        return STN1100_NO_RESPONSE;
    }

    char *match;
    match=strstr(data,"UNABLE TO CONNECT");
    if (match != NULL){
        return STN1100_UNABLE_TO_CONNECT;
    }
    match=strstr(data,"NO DATA");
    if (match != NULL){
        return STN1100_NO_DATA;
    }
    if (strncmp(data,"SEARCHING...",12)==0)
    {
        // Remove searching...
        byte i=12;
        while (data[i]!='\0'){
            data[i-12]=data[i];
            i++;
        }
        data[i]='\0';
    }

    // Otherwise return success.
    return STN1100_SUCCESS;
}

byte Stn1100::getVersion(String &rev)
{
    char data[20];
    byte status;
    char cmd[]="ATI";
    status=runCommand(cmd,data,20);
    rev=String(data);
    return status;
}

byte Stn1100::reset(String &response)
{
    char data[20];
    byte status;
    char cmd[]="ATZ";
    status=runCommand(cmd,data,20);
    response=String(data);
    return status;
}

byte Stn1100::getIgnMon(bool &powered){
    char data[20];
    byte status;
    char cmd[]="ATIGN";
    status=runCommand(cmd,data,20);
    if (cmd[1] == 'N' ){
        powered=true;
    }else{
        powered=false;
    }
    return status;
}

byte Stn1100::getVoltage(float &voltage){
    char data[20];
    byte status;
    char cmd[]="ATRV";
    status=runCommand(cmd,data,20);
    if (status==STN1100_SUCCESS){
        voltage=atof(data);
    }
    return status;
}


void Stn1100::flush(){
    while (STN1100_PORT.read() >= 0)
           ; // do nothing
}

/** returns the value of the specified bit p in byte b
 */
bool Stn1100::getBit(byte b, byte p)
{
    b<<=(7-p);
    if (b>=127){
        return true;
    }
    return false;
}
