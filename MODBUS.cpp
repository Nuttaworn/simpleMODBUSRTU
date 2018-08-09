#include "arduino.h"
using namespace std;


//================================================================== Userdata class ======================================================================//
class userData
{
private :
  byte *rawData;				//	raw data buffer pointer
  byte data[15];
  unsigned int CRC;				//	CRC
  unsigned int dataCount;		//	number of userdata byte
  unsigned int rawdataCount;   	// 	number of raw data byte

public :
  //-------------------------------- Constructor -------------------------------------//
  userData()
  {
    
  }
  
  userData(byte inputBuffer[],unsigned int length)
  {
    rawData = inputBuffer;			// set raw data input pointer
    rawdataCount = length;			// set raw data count
    dataCount = length - 2;			// set user data count (exclude 2 bytes of CRC)
    for(int i = 0 ; i < dataCount ; i++)
    {
      data[i] = inputBuffer[i];
    }
    CRC = (inputBuffer[length-1]*256)+inputBuffer[length-2];	// set CRC <---- | CRC_L | CRC_H |
  }
  //----------------------------------------------------------------------------------//

  //------------------------------------- Method -------------------------------------//
  void inputData(byte inputBuffer[],unsigned int length)
  {
    rawData = inputBuffer;			// set raw data input pointer
    rawdataCount = length;			// set raw data count
    dataCount = length - 2;			// set user data count (exclude 2 bytes of CRC)
    for(int i = 0 ; i < dataCount ; i++)
    {
      data[i] = inputBuffer[i];
    }
    CRC = (inputBuffer[length-1]*256)+inputBuffer[length-2];	// set CRC <---- | CRC_L | CRC_H |
  }  
  
  unsigned int Calculate()
  {
    unsigned int crc = 0xFFFF;
    for(int i = 0; i<dataCount;i++)
    {
      crc = CRC16(crc,data[i]);
    }
    return crc;
  }

  unsigned char Calculate_L()
  {
    unsigned int crc = 0xFFFF;
    for(int i = 0; i<dataCount;i++)
    {
      crc = CRC16(crc,data[i]);
    }
    return (crc & 0x00FF);
  }

  unsigned char Calculate_H()
  {
    unsigned int crc = 0xFFFF;
    for(int i = 0; i<dataCount;i++)
    {
      crc = CRC16(crc,data[i]);
    }
    return ((crc & 0xFF00)/256);
  }

  boolean crcCheck()
  {
    unsigned int crc = Calculate();
    if(CRC == crc)
    {
      return true;
    }
    else return false;
  }
  //-----------------------------------------------------------------------------------//

  //-----------------------------getter------------------------------------------------//
  unsigned int get_CRC()
  {
    return CRC;
  }

  byte* get_rawData()
  {
    return rawData;
  }

  unsigned int get_rawdataCount()
  {
    return rawdataCount;
  }

  byte* get_Data()
  {
    return data;
  }

  unsigned int get_dataCount()
  {
    return dataCount;
  }


private :
  unsigned int CRC16(unsigned int crc, unsigned int data) 
  { 
    const unsigned int Poly16=0xA001; 
    unsigned int LSB, i; 
    crc = ((crc^data) | 0xFF00) & (crc | 0x00FF); 
    for (i=0; i<8; i++) 
    { 
      LSB=(crc & 0x0001); 
      crc=crc/2; 
      if (LSB) crc=crc^Poly16; 
    } 
    return(crc); 
  } 
  //----------------------------------------------------------------------------------//
};
//========================================================================================================================================================//


//============================================================== Class modbusRTU =========================================================================//
class modbusRTU
{
  byte *userdata;
  unsigned int dataCount;
  unsigned int address;
  unsigned int functionCode;
  unsigned int startReg;    // 2 Byte ---- | Hbyte | Lbyte |
  unsigned int countReg;    // 2 Byte ---- | Hbyte | Lbyte |
  unsigned int slaveAddress;
  unsigned int registerAnalog[7];  // 7 available registers
  byte responseBuffer[30];
  unsigned int bufferIndex;

public :
  //-------------------------------- Constructor -------------------------------------//
  modbusRTU(unsigned int slaveAddr)           
  {
    slaveAddress = slaveAddr;
    registerAnalog[0] = 0;
    registerAnalog[1] = 0;
    registerAnalog[2] = 0;
    registerAnalog[3] = 0;
    registerAnalog[4] = 0;
    registerAnalog[5] = 0;
    registerAnalog[6] = 0;          // not use
    bufferIndex = 0;
  }
  //----------------------------------------------------------------------------------//

  //------------------------------------- Method -------------------------------------//
  void responseMsg(byte buffer[],unsigned int Count)
  {
    userdata = buffer;
    dataCount = Count;

    address = userdata[0];
    functionCode = userdata[1];

      if( (address == slaveAddress) && (functionCode == 4) )
      {
        unsigned int start = (userdata[2]*256) + userdata[3];
        unsigned int count = (userdata[4]*256) + userdata[5];

		//------------------------- All requested Register Addresses are available ------------//
        if(checkRegAddr(start,count))     // Check All Request Register Address
        {
          //----- Header------//
          responseBuffer[0] = slaveAddress;
          responseBuffer[1] = 0x04;
          responseBuffer[2] = count*2;
          bufferIndex = 3;
          //------------------//

          //------ Data ------//
          for(int i=start;  i<(start+count);  i++)
          {
            responseBuffer[bufferIndex] = (registerAnalog[i] & 0xFF00)/256; // HByte
            bufferIndex++;
            responseBuffer[bufferIndex] = registerAnalog[i] & 0x00FF;  // LByte
            bufferIndex++;
          }
          //-----------------//

          //----- CRC -------//
          responseBuffer[bufferIndex] = Calculate_L(responseBuffer,bufferIndex);
          responseBuffer[bufferIndex+1] = Calculate_H(responseBuffer,bufferIndex);
          bufferIndex = bufferIndex + 2;  // 2 counts for CRC
          //-----------------//

          Serial.write(responseBuffer,bufferIndex);
          Serial.flush();
          bufferIndex = 0;         
        }

        else if( (address == slaveAddress) && (functionCode == 3) )
      {
        unsigned int start = (userdata[2]*256) + userdata[3];
        unsigned int count = (userdata[4]*256) + userdata[5];

    //------------------------- All requested Register Addresses are available ------------//
        if(checkRegAddr(start,count))     // Check All Request Register Address
        {
          //----- Header------//
          responseBuffer[0] = slaveAddress;
          responseBuffer[1] = 0x03;
          responseBuffer[2] = count*2;
          bufferIndex = 3;
          //------------------//

          //------ Data ------//
          for(int i=start;  i<(start+count);  i++)
          {
            responseBuffer[bufferIndex] = (registerAnalog[i] & 0xFF00)/256; // HByte
            bufferIndex++;
            responseBuffer[bufferIndex] = registerAnalog[i] & 0x00FF;  // LByte
            bufferIndex++;
          }
          //-----------------//

          //----- CRC -------//
          responseBuffer[bufferIndex] = Calculate_L(responseBuffer,bufferIndex);
          responseBuffer[bufferIndex+1] = Calculate_H(responseBuffer,bufferIndex);
          bufferIndex = bufferIndex + 2;  // 2 counts for CRC
          //-----------------//

          Serial.write(responseBuffer,bufferIndex);
          Serial.flush();
          bufferIndex = 0;         
        }
		//------------------------------------------------------------------------------------//
      }
    }
  }
  
  
  void writeReg(unsigned int value,unsigned int addr)
  {
    if( addr >= 0 && addr <=5)
    {
      registerAnalog[addr] = value;
    }
  }

  boolean checkRegAddr(unsigned int addr,unsigned int count)
  {
    if(addr >= 0 && ((addr+count) <= 7))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  //-------------------------------------- Setter ------------------------------------//
   void set_slaveAddress(unsigned int slaveAddr)
   {
     if((slaveAddr >= 0) || (slaveAddr <= 3))    //check 2 bit address range = 0 to 3 
     {
       slaveAddress = slaveAddr;
     }
   }

  //------------------------------------- getter -------------------------------------//
  unsigned int get_slaveAddress()
  {
    return slaveAddress;
  }
  //----------------------------------------------------------------------------------//
  
private :

  unsigned char Calculate_L(byte data[],unsigned int dataCount)
  {
    unsigned int crc = 0xFFFF;
    for(int i = 0; i<dataCount;i++)
    {
      crc = CRC16(crc,data[i]);
    }
    return (crc & 0x00FF);
  }

  unsigned char Calculate_H(byte data[],unsigned int dataCount)
  {
    unsigned int crc = 0xFFFF;
    for(int i = 0; i<dataCount;i++)
    {
      crc = CRC16(crc,data[i]);
    }
    return ((crc & 0xFF00)/256);
  }

  unsigned int CRC16(unsigned int crc, unsigned int data) 
  { 
    const unsigned int Poly16=0xA001; 
    unsigned int LSB, i; 
    crc = ((crc^data) | 0xFF00) & (crc | 0x00FF); 
    for (i=0; i<8; i++) 
    { 
      LSB=(crc & 0x0001); 
      crc=crc/2; 
      if (LSB) crc=crc^Poly16; 
    } 
    return(crc); 
  } 
  //----------------------------------------------------------------------------------//
  
};
//========================================================================================================================================================//

















