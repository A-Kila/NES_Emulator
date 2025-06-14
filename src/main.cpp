#include <cassert>
#include <stdio.h>
#include <string>
#include <cassert>
#include "SDL_tools/sdl_init.h"
#include "nes/nes.h"

#define STRING(x) #x
#define XSTRING(x) STRING(x)

int main(int argc, char *argv[])
{
    // assert(argc == 2 && "Usage: nes_emulator $filepath");

    const std::string cartridge = "../../build/tests/system/resources/donkey_kong.nes"; //argv[1];
    // const std::string cartridge = "../../build/tests/system/resources/nestest.nes"; //argv[1]; 

    SDL::sdl_init_t sdl_init;
    NES::nes_t nes(cartridge);

    nes.reset();
    nes.run();

    return 0;
}