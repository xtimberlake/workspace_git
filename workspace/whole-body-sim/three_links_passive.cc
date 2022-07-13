#include "three_links_passive.h"

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::log()->info("hello there~");

    drake::workspace::threelinks::DoMain();

    return 0;
}
