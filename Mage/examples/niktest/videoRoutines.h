int SyncVideo(long frame);
int CaptureVideo(int frame);
int SetVideoChannel(int channel);
int CloseVideoDevice(unsigned char *buf, long height, long width, long nBytes, long nFrames);
unsigned char *OpenVideoDevice(long *height, long *width, long nBytes, long nFrames, int verbose, long offsets[]);
