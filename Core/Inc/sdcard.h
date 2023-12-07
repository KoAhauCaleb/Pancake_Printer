void myprintf(const char *fmt, ...);
//void Dequeue(void);
//void Enqueue(const char* line);
//void OpenFilesystem(void);
//void GCodeEnqueueFromCard(void);
//void RandomPrinting(void *argument);
void GetLine(BYTE readBuf[50]);
bool OpenFile(char* file_name);
void CloseFile(void);
int eof(void);

