#include <iostream>
#include <signal.h>
#include "src/App.h"

int main(int argc, char **argv)
{
    Application app(argc, argv);

    signal(SIGINT, SIG_DFL);
    return 0;
}
