#define LFS_MBED_RP2040_VERSION_MIN_TARGET      "LittleFS_Mbed_RP2040 v1.1.0"
#define LFS_MBED_RP2040_VERSION_MIN             1001000

#define _LFS_LOGLEVEL_          1
#define RP2040_FS_SIZE_KB       1536//1536

#define FORCE_REFORMAT          false

#include <LittleFS_Mbed_RP2040.h>
#include "filesystem.h"

LittleFS_MBED *myFS;


void readFile(const char * path) 
{
  Serial.print("Reading file: "); Serial.print(path);

  FILE *file = fopen(path, "r");
  
  if (file) 
  {
    Serial.println(" => Open OK");
  }
  else
  {
    Serial.println(" => Open Failed");
    return;
  }

  char c;
  uint32_t numRead = 1;
  
  while (numRead) 
  {
    numRead = fread((uint8_t *) &c, sizeof(c), 1, file);

    if (numRead)
      Serial.write(c); //not print since we want the serial data.
  }
  
  fclose(file);
}



void writeFile(const char * path, const char * message, size_t messageSize) 
{
  Serial.print("Writing file:    "); Serial.println(path);
  Serial.print("Writing message: "); Serial.println(message);
  Serial.print("With size:       "); Serial.println(messageSize);
  
  FILE *file = fopen(path, "w");
  
  if (file) 
  {
    Serial.println(" => Open OK");
  }
  else
  {
    Serial.println(" => Open Failed");
    return;
  }
 
  if (fwrite((uint8_t *) message, 1, messageSize, file)) 
  {
    Serial.println("* Writing OK");
  } 
  else 
  {
    Serial.println("* Writing failed");
  }
  
  fclose(file);
}

void appendFile(const char * path, const char * message, size_t messageSize) 
{
  Serial.print("Appending file: "); Serial.print(path);

  FILE *file = fopen(path, "a");
  
  if (file) 
  {
    Serial.println(" => Open OK");
  }
  else
  {
    Serial.println(" => Open Failed");
    return;
  }

  if (fwrite((uint8_t *) message, 1, messageSize, file)) 
  {
    Serial.println("* Appending OK");
  } 
  else 
  {
    Serial.println("* Appending failed");
  }
   
  fclose(file);
}


void testFileIO(const char * path) 
{
  Serial.print("Testing file I/O with: "); Serial.print(path);

  #define BUFF_SIZE     512
  
  static uint8_t buf[BUFF_SIZE];
  
  FILE *file = fopen(path, "w");
  
  if (file) 
  {
    Serial.println(" => Open OK");
  }
  else
  {
    Serial.println(" => Open Failed");
    return;
  }

  size_t i;
  Serial.println("- writing" );
  
  uint32_t start = millis();

  size_t result = 0;

  // Write a file only 1/4 of RP2040_FS_SIZE_KB
  for (i = 0; i < RP2040_FS_SIZE_KB / 2; i++) 
  {
    result = fwrite(buf, BUFF_SIZE, 1, file);

    if ( result != 1)
    {
      Serial.print("Write result = "); Serial.println(result);
      Serial.print("Write error, i = "); Serial.println(i);

      break;
    }
  }
  
  Serial.println("");
  uint32_t end = millis() - start;
  
  Serial.print(i / 2);
  Serial.print(" Kbytes written in (ms) ");
  Serial.println(end);
  
  fclose(file);

  printLine();

  /////////////////////////////////

  file = fopen(path, "r");
  
  start = millis();
  end = start;
  i = 0;
  
  if (file) 
  {
    start = millis();
    Serial.println("- reading" );

    result = 0;

    fseek(file, 0, SEEK_SET);

    // Read file only 1/4 of RP2040_FS_SIZE_KB
    for (i = 0; i < RP2040_FS_SIZE_KB / 2; i++) 
    {
      result = fread(buf, BUFF_SIZE, 1, file);

      if ( result != 1 )
      {
        Serial.print("Read result = "); Serial.println(result);
        Serial.print("Read error, i = "); Serial.println(i);

        break;
      }
    }
      
    Serial.println("");
    end = millis() - start;
    
    Serial.print((i * BUFF_SIZE) / 1024);
    Serial.print(" Kbytes read in (ms) ");
    Serial.println(end);
    
    fclose(file);
  } 
  else 
  {
    Serial.println("- failed to open file for reading");
  }
}



void deleteFile(const char * path) 
{
  Serial.print("Deleting file: "); Serial.print(path);
  
  if (remove(path) == 0) 
  {
    Serial.println(" => OK");
  }
  else
  {
    Serial.println(" => Failed");
    return;
  }
}



void printLine()
{
  Serial.println("====================================================");
}

void setupLFS() 
{
  //Serial.begin(115200);

  Serial.print("\nStart LittleFS_Test on "); Serial.println(BOARD_NAME);
  Serial.println(LFS_MBED_RP2040_VERSION);
  
#if defined(LFS_MBED_RP2040_VERSION_MIN)
  if (LFS_MBED_RP2040_VERSION_INT < LFS_MBED_RP2040_VERSION_MIN)
  {
    Serial.print("Warning. Must use this example on Version equal or later than : ");
    Serial.println(LFS_MBED_RP2040_VERSION_MIN_TARGET);
  }
#endif

  myFS = new LittleFS_MBED();

  if (!myFS->init()) 
  {
    Serial.println("LITTLEFS Mount Failed");
    
    return;
  }

  /* char fileName1[] = MBED_LITTLEFS_FILE_PREFIX "/helloTESTITEST.txt";
  //char fileName2[] = MBED_LITTLEFS_FILE_PREFIX "/hello2.txt";
  
  char message[]  = "Hello World!";
  
  printLine();
  writeFile(fileName1, message, sizeof(message));
  printLine();
  readFile(fileName1);
  printLine(); */

  /* appendFile(fileName1, message, sizeof(message));
  printLine();
  readFile(fileName1);
  printLine(); */


}


