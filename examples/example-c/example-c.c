#include "example-c.h"
#include <cave_talk.h>
#include <stdio.h>

int main(){
	CaveTalk_Handle_t bro;
	bro.buffer_size = 670;
	printf("%ld\n", bro.buffer_size);
	return 0;
}