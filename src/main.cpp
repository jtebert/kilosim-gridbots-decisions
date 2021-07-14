#include "HybridBot.cpp"
#include <kilosim/World.h>
#include <kilosim/Viewer.h>
#include <kilosim/ConfigParser.h>

#include <unistd.h>

int main(int argc, char *argv[])
{
    // Get config file name
    std::vector<std::string> args(argv, argv + argc);
    if (args.size() < 2)
    {
        std::cout << "ERROR: You must provide a config file name" << std::endl;
        exit(1);
    }
    Kilosim::ConfigParser config(args[1]);

    // Get configuration values
    const uint start_trial = config.get("start_trial");
    const uint num_trials = config.get("num_trials");
    const uint trial_duration = config.get("trial_duration"); // in ticks
    const std::string img_dir = config.get("img_dir");
    const uint env_octaves = config.get("env_octaves");
    const uint world_width = (int)config.get("world_grid_width") * 10;
    const int world_height = (int)config.get("world_grid_height") * 10;
    const int num_robots = config.get("num_robots");

    for (auto trial = start_trial; trial < start_trial + num_trials; trial++)
    {
        std::string trial_str = std::to_string(trial);
        trial_str.insert(trial_str.begin(), 3 - trial_str.size(), '0');
        const std::string img_filename = img_dir + "/img_oct=" + std::to_string(env_octaves) +
                                         "_" + trial_str + ".png";
        std::cout << img_filename << std::endl;

        // Create 3m x 3m world (no background image, for now)
        Kilosim::World world(
            world_width, world_height, img_filename // World image
        );

        Kilosim::Viewer viewer(world, 1200);

        // Create robots and place in world
        std::vector<Kilosim::HybridBot *> robots(num_robots);
        for (int n = 0; n < num_robots; n++)
        {
            robots[n] = new Kilosim::HybridBot();
            world.add_robot(robots[n]);
            robots[n]->robot_init(0, 0, 0);
            robots[n]->step_interval = config.get("step_interval");
            robots[n]->start_interval = (int)config.get("step_interval") * 2;
            robots[n]->end_condition = config.get("end_condition");
            robots[n]->end_val = config.get("end_val");
            robots[n]->pso_max_speed = config.get("pso_max_speed");
            robots[n]->pso_inertia = config.get("pso_inertia");
            robots[n]->pso_self_weight = config.get("pso_self_weight");
            robots[n]->pso_group_weight = config.get("pso_group_weight");
            robots[n]->gradient_weight = config.get("gradient_weight");
            // comm_range is used by comm_criteria to determine communication range
            // In config, comm_range is in grid cells (as dimension)
            robots[n]->comm_range = (int)config.get("comm_range") * 10;
            robots[n]->num_neighbors = num_robots - 1;
            robots[n]->rx_table_timeout = config.get("rx_table_timeout");
        }

        // robots[0]->set_path(10, 10, 12, 5);
        // for (int i = 0; i < path.size(); i++)
        // {
        //     std::cout << path[i].x << ", " << path[i].y << std::endl;
        // }

        // Verify that robots are within World bounds and not overlapping
        // world.check_validity();

        sleep(2);
        while (world.get_time() < trial_duration / world.get_tick_rate())
        {
            // printf("stepping\n");
            viewer.draw();
            world.step();
            robots[0]->print_neighbor_array();
            // usleep(5000); //.05s
            // usleep(1000000); //.05s
        }
    }

    printf("Finished");

    return 0;
}
