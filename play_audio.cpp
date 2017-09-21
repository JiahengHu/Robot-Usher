/*
    05/07/2017
    CS363 S17
    Project7
    Liwei Jiang
	
	Play an audio .wav file.
    play_audio.cpp
*/


#include "play_audio.h"

/* monitoring the audio length and the current position it is playing */
void MyAudioCallback(void* userdata, Uint8* stream, int streamLength) {
	AudioData* audio = (AudioData*)userdata;

	if (audio->length == 0) {
		return;
	}

	Uint32 length = (Uint32) streamLength;
	length = (length > audio->length ? audio->length : length);

	SDL_memcpy(stream, audio->pos, length);

	audio->pos += length;
	audio->length -= length;
}

/* play the audio */
int audio_play(std::string FILE_PATH) {
	SDL_Init(SDL_INIT_EVERYTHING);
	char c;
	SDL_AudioSpec wavSpec;
	Uint8* wavStart;
	Uint32 wavLength;
	const char *file_path = FILE_PATH.c_str();

	if (SDL_LoadWAV(file_path, &wavSpec, &wavStart, &wavLength) == NULL) {
		// TODO: Proper error handling
		std::cerr << "Error: " << FILE_PATH << " could not be loaded as an audio file" << std::endl;
		return 1;
	}

	AudioData audio;
	audio.pos = wavStart;
	audio.length = wavLength;

	wavSpec.callback = MyAudioCallback;
	wavSpec.userdata = &audio;

	SDL_AudioDeviceID device = SDL_OpenAudioDevice(NULL, 0, &wavSpec, NULL, SDL_AUDIO_ALLOW_ANY_CHANGE);

	if (device == 0) {
		// TODO: Proper error handling
		std::cerr << "Error: " << SDL_GetError() << std::endl;
		return 1;
	}

	SDL_PauseAudioDevice(device, 0);

	while (audio.length > 0) {
		printf("length: %d\n", audio.length);
		read(0, &c, 1);
		if((c == 't') || (c == 'T')) {
			break;
		}
		SDL_Delay(100);
	}

	SDL_CloseAudioDevice(device);
	SDL_FreeWAV(wavStart);
	SDL_Quit();
    return 0;
 }


// 
// 
// 
// /* main function */
// int main (int argc, char** argv) {
// 	std::string FILE_PATH;
// 	FILE_PATH = "introduction.wav";
// 	audio_play(FILE_PATH);
// }

