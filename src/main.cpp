#include "HybridBot.cpp"
#include "SweepBot.cpp"
#include <kilosim/World.h>
#include <kilosim/Viewer.h>
#include <kilosim/ConfigParser.h>
#include <kilosim/Logger.h>

#include <unistd.h>

// AGGREGATORS

std::vector<double> decision_states(std::vector<Kilosim::Robot *> &robots)
{
    // Get the current state of the robots' decisions. (undecided, self decided, all decided)
    // This is represented by the robot's m_state
    std::vector<double> decision_states;
    for (auto i = 0ul; i < robots.size(); i++)
    {
        Kilosim::BaseBot *bot = (Kilosim::BaseBot *)robots[i];
        decision_states[i] = bot->get_state();
    }
    return decision_states;
}

std::vector<double> robot_coverage(std::vector<Kilosim::Robot *> &robots)
{
    // Get the total number of cells covered by each robot.
    // This includes the number of cells covered by the robot's sensors, not where the robot walked.
    // It also doesn't account for duplicates (ie, if a robot is in a cell twice, it will only count once).
    std::vector<double> coverage(robots.size());
    int i = 0;
    for (auto &robot : robots)
    {
        Kilosim::BaseBot *bot = (Kilosim::BaseBot *)robot;
        coverage[i] = bot->map_coverage_count();
        i++;
    }
    return coverage;
}

std::vector<double> robot_distance(std::vector<Kilosim::Robot *> &robots)
{
    // Get the distance each robot has traveled.
    // This is the number of cells the robot has walked, not the number of cells covered by the robot's sensors.
    std::vector<double> distances(robots.size());
    for (auto i = 0ul; i < robots.size(); i++)
    {
        Kilosim::BaseBot *bb = (Kilosim::BaseBot *)robots[i];
        distances[i] = bb->map_visited_count();
    }
    return distances;
}

std::vector<double> collective_coverage(std::vector<Kilosim::Robot *> &robots)
{
    // Add up how much of the map the robots have observed (over all robots).
    // This is very time-intensive -- AT LEAST O(nwh) (n=num_robots, w=width, h=height)
    // Probably most useful to only call at the end to check how much was covered
    std::vector<std::vector<int>> collective_map;
    for (auto i = 0ul; i < robots.size(); i++)
    {
        Kilosim::BaseBot *bb = (Kilosim::BaseBot *)robots[i];
        std::vector<std::vector<int>> map = bb->get_map();
        // On the first go, resize the map to match the robot's map and fill with zeros
        if (i == 0)
        {
            collective_map.resize(map.size(), std::vector<int>(map[0].size(), 0));
            // m_map.resize(m_arena_grid_height, std::vector<int>(m_arena_grid_width, -1))
        }
        // Add the robot's map to the collective map
        for (auto j = 0ul; j < map.size(); j++)
        {
            for (auto k = 0ul; k < map[j].size(); k++)
            {
                if (map[j][k] != -1)
                {
                    // Only add 1 if the cell was observed
                    collective_map[j][k] += std::min(1, map[j][k]);
                }
            }
        }
    }
    // Add up the collective map to get the total number of cells covered
    double total_coverage = 0;
    for (auto j = 0ul; j < collective_map.size(); j++)
    {
        for (auto k = 0ul; k < collective_map[j].size(); k++)
        {
            total_coverage += collective_map[j][k];
        }
    }
    return std::vector<double>{total_coverage};
}

// -------------------------------------------------------------------------------------------------

// Check if all of the robots in the world are finished
// (either they all found the source and returned home, or the time limit expired)
bool is_finished(Kilosim::World &world, std::vector<Kilosim::HybridBot *> robots, std::string end_condition, int end_val)
{
    if (end_condition == "time")
    {
        return world.get_time() * world.get_tick_rate() >= end_val;
    }
    else if (end_condition == "value")
    {
        for (auto robot : robots)
        {
            if (!robot->is_home())
            {
                // At least one robot is not home (ie finished)
                return false;
            }
        }
    }
    else if (end_condition == "first_find")
    {
        for (auto robot : robots)
        {
            // Robot is finished if it found a target below threshold
            if (robot->is_finished())
            {
                return true;
            }
        }
        // There are no robots finished
        return false;
    }
    // Else no robots that *didn't* meet the end condition
    return true;
}

bool is_finished(Kilosim::World &world, std::vector<Kilosim::SweepBot *> robots, std::string end_condition, int end_val)
{
    // HACKY: This is just a copy of the HybridBot version
    if (end_condition == "time")
    {
        return world.get_time() * world.get_tick_rate() >= end_val;
    }
    else if (end_condition == "value")
    {
        for (auto robot : robots)
        {
            if (!robot->is_home())
            {
                // At least one robot is not home (ie finished)
                return false;
            }
        }
    }
    // Else no robots that *didn't* meet the end condition
    return true;
}

void print_pos_vec(std::vector<Pos> pos_vec)
{
    for (auto pos : pos_vec)
    {
        std::cout << pos.x << ", " << pos.y << std::endl;
    }
}

std::vector<std::vector<Pos>> compute_sweep_paths(int arena_width, int arena_height, int num_robots)
{
    // Compute the predetermined sweep paths for the robots to follow
    std::vector<std::vector<Pos>> paths;
    for (int i = 0; i < num_robots; i++)
    {
        Pos start_pos = {0, 0};
        int x_max = arena_width - i * 3 - 1;
        int x_min = num_robots * 3 - i * 3 - 2;
        int y_move_r = (num_robots * 3 - i * 3) * 2 - 3;
        int y_move_l = (i * 3) * 2 + 3;
        int x = 0;
        int y = i * 3 + 1;
        std::vector<Pos> path = {start_pos};

        // Start by going from the origin to the starting position along the left side
        // To get everyone lined up takes a fixed time dependent on the number of robots
        // Then subtract off the number of ticks it takes for a robot to get to its OWN start pos.
        // time_to_lineup = num_robots * 3 - 1  # Should be -2, but we're accounting for origin @ t=0
        int next_y = start_pos.y;
        int next_x = start_pos.x;
        while (path.back() != Pos({x, y}))
        {
            if (x > next_x)
            {
                next_x += 1;
            }
            next_y += 1;
            path.push_back({next_x, next_y});
        }

        // Go until hitting the bottom
        while (path.back().y < arena_height)
        {
            // x right
            if ((y >= arena_height - y_move_r && arena_height / num_robots % 2 == 1) || num_robots >= arena_width / 3)
            {
                // Last row-set of an odd number or multiples of number of robots
                for (int x_ = 0; x_ < arena_width - 1; x_++)
                {
                    path.push_back({x_, y});
                }
                break;
            }
            else
            {
                for (int x_ = x + 1; x_ < x_max; x_++)
                {
                    path.push_back({x_, y});
                }
                x = x_max - 1;
                // down (right)
                for (int y_ = y + 1; y_ < y_move_r + y; y_++)
                {
                    path.push_back({x, y_});
                }
                y = y + y_move_r;
                // x left
                for (int x_ = x_max - 1; x_ > x_min - 1; x_--)
                {
                    path.push_back({x_, y});
                }
                x = x_min;
                // down (left)
                for (int y_ = y + 1; y_ < y_move_l + y; y_++)
                {
                    path.push_back({x, y_});
                }
                y = y + y_move_l;
                // ... repeat
            }
        }
        // Filter out points outside of the arena
        path.erase(std::remove_if(path.begin(), path.end(), [&](Pos p)
                                  { return p.x < 0 || p.x >= arena_width || p.y < 0 || p.y >= arena_height; }),
                   path.end());
        std::reverse(path.begin(), path.end());
        paths.push_back(path);
    }
    return paths;
}

void hybrid_sim(Kilosim::World &world, Kilosim::Logger &logger, Kilosim::ConfigParser &config)
{
    // Kilosim::Viewer viewer(world, 1200);
    // viewer.set_show_network(true);
    // viewer.set_show_tags(true);

    int num_robots = config.get("num_robots");
    std::string end_condition = config.get("end_condition");
    int end_val = config.get("end_val");
    unsigned long int max_duration = config.get("max_trial_duration");

    std::vector<Kilosim::HybridBot *> robots(num_robots);

    int i = 0;
    for (int n = 0; n < num_robots; n++)
    {
        robots[n] = new Kilosim::HybridBot();
        world.add_robot(robots[n]);
        robots[n]->robot_init(0, 0, 0);
        robots[n]->pso_step_interval = config.get("pso_step_interval");
        robots[n]->boids_step_interval = config.get("boids_step_interval");
        robots[n]->start_interval = (int)config.get("pso_step_interval") * 3;
        robots[n]->end_condition = config.get("end_condition");
        robots[n]->end_val = config.get("end_val");
        robots[n]->max_speed = config.get("max_speed");
        robots[n]->pso_inertia = config.get("pso_inertia");
        robots[n]->pso_self_weight = config.get("pso_self_weight");
        robots[n]->pso_group_weight = config.get("pso_group_weight");
        robots[n]->gradient_weight = config.get("gradient_weight");
        // comm_range is used by comm_criteria to determine communication range
        // In config, comm_range is in grid cells (as dimension)
        robots[n]->comm_range = (int)config.get("comm_range") * 10;
        robots[n]->num_neighbors = num_robots - 1;
        robots[n]->rx_table_timeout = config.get("rx_table_timeout");
        // Boids parameters
        robots[n]->lj_a = config.get("lj_a");
        robots[n]->lj_b = config.get("lj_b");
        robots[n]->lj_epsilon = config.get("lj_epsilon");
        robots[n]->lj_gamma = config.get("lj_gamma");
        robots[n]->boids_step_interval = config.get("boids_step_interval");
        // TEMPORARY
        // robots[n]->velocity = {(double)i, (double)i * 2};
        i++;
    }
    // sleep(2);
    // while (world.get_time() < trial_duration * world.get_tick_rate())
    while (!is_finished(world, robots, end_condition, end_val) &&
           world.get_tick() <= max_duration)
    {
        // viewer.draw();
        world.step();
        // if ((int)(world.get_time() * world.get_tick_rate()) % 10 == 0)
        // usleep(5000); //.05s
        // sleep(1);
    }
}

void sweep_sim(Kilosim::World &world, Kilosim::Logger &logger, Kilosim::ConfigParser &config)
{
    // Kilosim::Viewer viewer(world, 1200);
    // viewer.set_show_network(true);
    // viewer.set_show_tags(true);

    int num_robots = config.get("num_robots");
    std::string end_condition = config.get("end_condition");
    int end_val = config.get("end_val");
    unsigned long int max_duration = config.get("max_trial_duration");

    std::vector<std::vector<Pos>> sweep_paths =
        compute_sweep_paths(config.get("world_grid_width"),
                            config.get("world_grid_height"),
                            config.get("num_robots"));
    int max_path_len = 0;
    for (auto path : sweep_paths)
    {
        if (path.size() > max_path_len)
        {
            max_path_len = path.size();
        }
    }
    std::cout << max_path_len << std::endl;

    std::vector<Kilosim::SweepBot *> robots(num_robots);
    for (int n = 0; n < num_robots; n++)
    {
        std::vector<Pos> this_path = sweep_paths[n];

        Pos start_pos = this_path.back();
        this_path.pop_back();
        robots[n] = new Kilosim::SweepBot();
        world.add_robot(robots[n]);
        robots[n]->robot_init(start_pos.x, start_pos.y, 0);
        robots[n]->end_condition = end_condition;
        robots[n]->end_val = end_val;
        robots[n]->set_path_external(this_path);
        robots[n]->max_path_len = max_path_len;
        // In config, comm_range is in grid cells (as dimension)
        robots[n]->comm_range = (int)config.get("comm_range") * 10;
    }
    // sleep(2);
    while (!is_finished(world, robots, end_condition, end_val) &&
           world.get_tick() <= max_duration)
    {
        // viewer.draw();
        world.step();
        // usleep(100000); //.1s
    }
}

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
    const int start_trial = config.get("start_trial");
    const int num_trials = config.get("num_trials");
    const std::string img_dir = config.get("img_dir");
    const int env_octaves = config.get("env_octaves");
    const int world_grid_width = (int)config.get("world_grid_width");
    const int world_grid_height = (int)config.get("world_grid_height");
    const int world_width = world_grid_width * 10;
    const int world_height = world_grid_height * 10;
    const std::string movement_type = config.get("movement_type");

    for (auto trial = start_trial; trial < start_trial + num_trials; trial++)
    {
        std::string trial_str = std::to_string(trial);
        trial_str.insert(trial_str.begin(), 3 - trial_str.size(), '0');
        const std::string img_filename = img_dir + "/" + trial_str +
                                         "_oct=" + std::to_string(env_octaves) + ".png";
        // const std::string img_filename = img_dir + "/img_oct=" + std::to_string(env_octaves) +
        //                                  "_" + trial_str + ".png";
        // std::cout << img_filename << std::endl;

        // Create 3m x 3m world (no background image, for now)
        Kilosim::World world(
            world_width, world_height, img_filename // World image
        );
        world.set_comm_rate(1);

        // Create log file
        std::string log_filename = (std::string)config.get("log_dir") + "data.h5";
        Kilosim::Logger logger(world, log_filename, trial, true);

        // Run the right simulation
        if (movement_type == "hybrid")
        {
            hybrid_sim(world, logger, config);
        }
        else if (movement_type == "sweep")
        {
            sweep_sim(world, logger, config);
        }

        // These are things that are only logged once, at the end of the experiment!
        logger.add_aggregator("robot_distance", robot_distance);
        logger.add_aggregator("robot_coverage", robot_coverage);
        logger.add_aggregator("collective_coverage", collective_coverage);
        logger.log_state();

        std::cout << "Finished trial " << trial << "\tt=" << world.get_time() * world.get_tick_rate() << std::endl;
    }

    printf("Finished\n");

    return 0;
}