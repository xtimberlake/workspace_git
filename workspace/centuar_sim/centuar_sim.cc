#include "centuar_sim.h"

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::log()->info("hello centuar~");

    drake::workspace::centuar_sim::DoMain();
    return 0;
}
