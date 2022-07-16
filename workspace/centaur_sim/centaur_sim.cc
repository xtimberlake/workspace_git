#include "centaur_sim.h"

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::log()->info("hello centaur~");

    drake::workspace::centaur_sim::DoMain();
    return 0;
}
