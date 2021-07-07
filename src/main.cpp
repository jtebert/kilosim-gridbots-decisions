#include "HybridBot.cpp"
#include <kilosim/World.h>
#include <kilosim/Viewer.h>

#include <unistd.h>

int main(int argc, char *argv[])
{
    // Create 3m x 3m world (no background image, for now)
    Kilosim::World world(
        80, 80, "../imgs/8x8-test.png" // World image
    );

    Kilosim::Viewer viewer(world, 1200);

    int num_robots = 1;

    // Create robots and place in world
    std::vector<Kilosim::HybridBot *> robots(num_robots);
    for (int n = 0; n < num_robots; n++)
    {
        robots[n] = new Kilosim::HybridBot();
        world.add_robot(robots[n]);
        robots[n]->robot_init(n, n, 0);
    }

    // robots[0]->set_path(10, 10, 12, 5);
    // for (int i = 0; i < path.size(); i++)
    // {
    //     std::cout << path[i].x << ", " << path[i].y << std::endl;
    // }

    // Verify that robots are within World bounds and not overlapping
    // world.check_validity();

    double trial_duration = 300; // seconds
    sleep(2);
    while (world.get_time() < trial_duration)
    {
        // printf("stepping\n");
        viewer.draw();
        world.step();
        sleep(2);
    }

    printf("Finished");

    return 0;
}
