/*
    05/07/2017
    CS363 S17
    Project7
    Liwei Jiang
	
	header file of play_audio.cpp
    play_audio.h
*/

#include <iostream>
#include <SDL2/SDL.h>
#include <unistd.h>
#include <string.h>

/* struct storing the current position and the length of the audio file. */
struct AudioData {
	Uint8* pos;
	Uint32 length;
};

/* monitoring the audio length and the current position it is playing */
void MyAudioCallback(void* userdata, Uint8* stream, int streamLength);

/* play the audio */
int audio_play(std::string FILE_PATH);






