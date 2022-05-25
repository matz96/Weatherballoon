#ifndef filesystem_H_
#define filesystem_H_
#endif



void readFile(const char * path);
void writeFile(const char * path, const char * message, size_t messageSize);
void appendFile(const char * path, const char * message, size_t messageSize);
void testFileIO(const char * path);
void deleteFile(const char * path);
void printLine();
void setupLFS();