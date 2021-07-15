/*
 * Gridbots doing target localization (min/max)
 *
 * Hybrid algorithm is PSO + gradient descent + Boids
 *
 * Created 2020-07 by Julia Ebert
 */

#include "Gridbot.h"

#include <cmath>

namespace Kilosim
{

    typedef struct neighbor_info_t
    {
        // one entry (row) in a table of observations/messages from neighbor robots
        uint16_t id;
        uint min_val;
        uint min_loc_x;
        uint min_loc_y;
        uint curr_pos_x;
        uint curr_pos_y;
        double curr_vel_x;
        double curr_vel_y;
        // std::vector<double> pso_velocity;
        uint32_t tick_added;
    } neighbor_info_t;

    class HybridBot : public Gridbot
    {
    public:
        // Variables for aggregators
        // TODO: Add aggregators
        Pos curr_pos;

        // ---------------------------------------------------------------------

        // TODO: Add anything here that needs to be set by main (ie externally)
        // (Particularly parameters from external config files)

        // Every how many ticks to update the PSO target/velocity
        int step_interval;
        int boids_step_interval = 10;
        // Time to wait before doing first PSO update, so they spread out at start
        int start_interval;

        // PSO Parameters (set by main function)
        // Inertia (omega) from 0-1(ish)
        double pso_inertia; // 1.2
        // Constant personal cognition (weight)
        double pso_self_weight; // 0.0
        // Constant collective/group cognition (weight)
        double pso_group_weight; // 0.0
        // Current PSO velocity (for next step)
        // TODO: Starting PSO velocity might be set by parameters
        std::vector<double> velocity = {uniform_rand_real(-1, 1),
                                        uniform_rand_real(-1, 1)};
        // Maximum allowed magnitude of the PSO internal velocity (per tick?)
        // TODO: Not sure about the magnitude of this max_speed (self.max_pso_vel)
        double pso_max_speed; // 25

        // GRADIENT DESCENT parameters
        double gradient_weight;

        // DECISION-MAKING parameters
        std::string end_condition = "value"; // or "time"
        // Threshold for ending (declaring target found)
        // For "value": decision if min_val is <= end_val, decision is made
        // For "time": decision if get_tick() >= end_val
        int end_val;

        // MISCELLANEOUS
        int num_neighbors;    // Used for making neighbor array table
        int rx_table_timeout; // Time neighbor messages stay in table

        // Data values
        // Minimum value (global/collective) and its location
        uint min_val = 99999; // start higher than anything observable
        Pos min_loc = {-1, -1};
        // Lowest value from OWN observations
        uint obs_min_val = 99999;
        Pos obs_min_loc = {-1, -1};
        // Lowest value from RECEIVED messages
        uint rx_min_val = 99999;
        Pos rx_min_loc = {-1, -1};

        // ---------------------------------------------------------------------

        // Set up things used in common between all decision-making robots
        // TODO: Run setup (self.setup())

        void print_neighbor_array()
        {
            printf("\n");
            std::cout << "Own ID=" << id << "\tv=" << min_val << "\tstate=" << (int)m_state << "\t(t=" << get_tick() << ")" << std::endl;
            std::cout << "ID\tMinX\tMinY\tMinVal\tTickAdded" << std::endl;
            for (auto &n : neighbor_table)
            {
                if (n.id != 0)
                {
                    std::cout << n.id << "\t" << n.min_loc_x << "\t" << n.min_loc_y << "\t" << n.min_val << "\t" << n.tick_added << std::endl;
                }
            }
        }

    private:
        // States
        static const uint8_t INIT = 0;
        static const uint8_t SPREAD = 1;
        static const uint8_t DO_PSO = 2;
        static const uint8_t DECIDED = 3;
        static const uint8_t GO_HOME = 4;
        static const uint8_t HOME = 5;

        // Utility attributes/variables
        // Current (starting) angle and velocity
        double start_angle = uniform_rand_real(5 * PI / 180, 85 * PI / 180);
        // Previous position (current position curr_pos is public for aggregators)
        Pos prev_pos;
        // Where the robot is going (used by move_toward_target)
        Pos target_pos;

        uint8_t m_state = INIT;

        std::vector<neighbor_info_t> neighbor_table;

        void setup()
        {
            move(1, 1);
            set_led(100, 0, 0);
            curr_pos = get_pos();
        };

        void loop()
        {
            // Update positions
            prev_pos = curr_pos;
            curr_pos = get_pos();

            // Call every loop
            // Process all messages in the queue since the last tick
            std::map<Pos, std::vector<double>> neighbor_pos_vel = process_msgs();
            prune_neighbor_table();
            update_send_msg();

            if (m_state == INIT)
            {
                // Set the velocity on the first take (angling away from origin)
                initialize_neighbor_array();
                std::vector<double> tmp_vel = {
                    start_interval * cos(start_angle),
                    start_interval * sin(start_angle)};
                target_pos = {2 * tmp_vel[0], 2 * tmp_vel[1]};
                set_pso_path(target_pos, tmp_vel, start_interval);
                target_pos = m_path_to_target[0]; // target is last position in path
                m_state = SPREAD;
            }
            else if (m_state == SPREAD)
            {
                // Spread away from the origin before doing PSO
                std::map<Pos, double> pos_samples = sample_around();
                map_samples(pos_samples);
                update_mins(pos_samples);
                if (get_tick() >= start_interval || min_val < end_val)
                {
                    m_state = DO_PSO;
                }
            }
            else if (m_state == DO_PSO)
            {
                int tick = get_tick();
                // if (tick % 2 == 0)
                // {
                // set_led(0, 100, 0);
                // }
                std::map<Pos, double> pos_samples = sample_around();
                map_samples(pos_samples);
                update_mins(pos_samples);
                if (tick % step_interval == 1)
                {
                    std::vector<double> new_pos_vel = pso_velocity_update(pos_samples);
                    target_pos = {new_pos_vel[0], new_pos_vel[1]};
                    velocity = {new_pos_vel[2], new_pos_vel[3]};
                    set_pso_path(target_pos, velocity);
                }
                if (is_finished())
                {
                    m_state = DECIDED;
                    printf("decision finished\n");
                }
            }
            else if (m_state == DECIDED)
            {
                // Share the decision with neighbors (aka do Boids)
                std::map<Pos, double> pos_samples = sample_around();
                map_samples(pos_samples);
                update_mins(pos_samples);
                if ((int)get_tick() % boids_step_interval == 0)
                {
                    // Boids!
                    std::vector<double> new_pos_vel = boids_velocity_update(neighbor_pos_vel);
                    target_pos = {new_pos_vel[0], new_pos_vel[1]};
                    velocity = {new_pos_vel[2], new_pos_vel[3]};
                    set_pso_path(target_pos, velocity);
                }
                if (all_neighbors_decided())
                {
                    m_state = GO_HOME;
                    set_led(100, 0, 0);
                }
            }
            else if (m_state == GO_HOME)
            {
                // Everyone knows about the target; go home
                target_pos = {0, 0};
                // velocity (={0,0}) doesn't matter here
                // path_len just has to be longer that whatever it generates
                set_pso_path(target_pos, {0, 0}, 1000);
                if (curr_pos == target_pos)
                {
                    m_state = HOME;
                }
            }
            else if (m_state == HOME)
            {
                // nothing to do
            }

            // prune_rx_table();
            // set_color_by_val();
            // send_msg();

            // // std::cout << m_grid_x << ", " << m_grid_y << std::endl;
            // if (m_test <= 8)
            // {
            //     std::cout << "\n"
            //               << m_test << std::endl;
            //     std::map<Pos, double> samples = sample_around();
            //     std::cout << samples.size() << std::endl;
            //     for (auto const &s : samples)
            //     {
            //         std::cout << s.first.x << ", " << s.first.y // string (key)
            //                   << ": "
            //                   << s.second // string's value
            //                   << std::endl;
            //     }
            // }
            // m_test++;
        };

        //----------------------------------------------------------------------
        // COMMUNICATION
        //----------------------------------------------------------------------

        void initialize_neighbor_array()
        {
            // Set up the neighbor array
            neighbor_table.resize(num_neighbors);
            for (auto i = 0; i < neighbor_table.size(); i++)
            {
                // Assigning all IDs to 0 indicates no robot in that row
                // (otherwise random ID from memory allocation could make it a mess)
                neighbor_table[i].id = 0;
            }
        }

        std::map<Pos, std::vector<double>> process_msgs()
        {
            // Process all messages since the last tick
            // Add each one to the neighbor table
            // RETURNS: map<Pos, Pos> of {position, velocity} of all neighbors heard from in this tick
            // (this is used for Boids velocity updates)

            std::vector<json> new_msgs = get_msg();

            bool was_added = false;
            int ind_to_fill = -1;
            int curr_tick = get_tick();
            std::map<Pos, std::vector<double>> neighbor_pos_vel;

            for (auto m = 0; m < new_msgs.size(); m++)
            {
                json msg = new_msgs[m];
                if (!msg.is_null())
                {
                    // Put into return map (position + velocity)
                    neighbor_pos_vel.insert({{msg.at("curr_pos_x"), msg.at("curr_pos_y")},
                                             {msg.at("curr_vel_x"), msg.at("curr_vel_y")}});
                    was_added = false;
                    for (auto i = 0; i < neighbor_table.size(); i++)
                    {
                        if (neighbor_table[i].id == msg.at("id"))
                        {
                            // printf("UPDATING message\n");
                            // Neighbor is already in the table
                            // Update the value and stop looking
                            add_msg(msg, i);
                            was_added = true;
                            break;
                            // Setting the ID to 0 indicates later that this message has been processed
                            // new_msgs[m]["id"] = 0;
                        }
                        else if (neighbor_table[i].id == 0)
                        {
                            // This is an empty slot that can be used for inserting new elements
                            ind_to_fill = i;
                        }
                    } // end neighbor_table loop
                    if (!was_added)
                    {
                        // printf("ADDING message\n");
                        // This wasn't updating an existing value in the table.
                        // Instead, add it at ind_to_fill
                        // NOTE: ind_to_fill should always be set if this step is reached, because the
                        // size of the table matches the number of neighbors
                        add_msg(msg, ind_to_fill);
                    }
                }
            } // end msg loop
            return neighbor_pos_vel;
        }

        void add_msg(json msg, int ind)
        {
            // Add a message to the neighbor table and check if it gives a new min value/location
            // NOTE: ind is the index of the neighbor in the neighbor table
            neighbor_table[ind].id = msg.at("id");
            neighbor_table[ind].min_loc_x = msg.at("min_loc_x");
            neighbor_table[ind].min_loc_y = msg.at("min_loc_y");
            neighbor_table[ind].min_val = msg.at("min_val");
            neighbor_table[ind].tick_added = get_tick();
            // Check if the new message is a new min value
            if (msg.at("min_val") < min_val)
            {
                min_val = msg.at("min_val");
                min_loc = {msg.at("min_loc_x"), msg.at("min_loc_y")};
                rx_min_val = msg.at("min_val");
                rx_min_loc = {msg.at("min_loc_x"), msg.at("min_loc_y")};
            }
        }

        void prune_neighbor_table()
        {
            // Remove old messages, based on some fixed time since they were added
            // (Removal means setting the ID to 0, so it's marked as empty/ignored)
            int curr_tick = get_tick();
            for (auto i = 0; i < neighbor_table.size(); i++)
            {

                if (neighbor_table[i].tick_added + rx_table_timeout <= curr_tick)
                {
                    neighbor_table[i].id = 0;
                }
            }
        }

        // Send message (continuously)
        void update_send_msg()
        {
            // Only set the message if a value has been observed (i.e. not max)
            if (min_val < 99999)
            {
                // Create message to send
                json msg;
                msg["id"] = id;
                msg["min_loc_x"] = min_loc.x;
                msg["min_loc_y"] = min_loc.y;
                msg["min_val"] = min_val;
                msg["curr_pos_x"] = curr_pos.x;
                msg["curr_pos_y"] = curr_pos.y;
                msg["curr_vel_x"] = velocity[0];
                msg["curr_vel_y"] = velocity[1];
                send_msg(msg);
            }
        }

        //----------------------------------------------------------------------
        // MOVEMENT
        //----------------------------------------------------------------------

        std::vector<double> pso_velocity_update(std::map<Pos, double> samples)
        {
            // Do an update of (PSO + gradient descent)
            // returns the new velocity AND updates target position (target_pos)

            // PSO:
            // For each axis (x,y):
            // velocity(t+1) = inertia * velocity(t) +
            //                 p_weight * rand(t) + (p_best-pos(t)) +
            //                 g_weight * rand(t) + (g_best-pos(t))
            // pos(t+1) = pos(t) + velocity(t+1)

            std::vector<int> local_min_loc;
            int tmp_min_val = 99999;
            for (auto const &s : samples)
            {
                if (s.second < tmp_min_val)
                {
                    local_min_loc = {s.first.x, s.first.y};
                }
            }

            std::vector<double> new_vel = {0, 0};
            std::vector<double> new_pos = {0, 0};
            // Convert to vectors for
            std::vector<int> v_obs_min_loc = {obs_min_loc.x, obs_min_loc.y};
            std::vector<int> v_min_loc = {min_loc.x, min_loc.y};
            std::vector<int> v_curr_pos = {curr_pos.x, curr_pos.y};

            // Compute PSO
            for (auto i = 0; i < 2; i++)
            {
                double inertia = pso_inertia + velocity[i];
                double p_term = pso_self_weight * uniform_rand_real(0, 1) * (v_obs_min_loc[i] - v_curr_pos[i]);
                double g_term = pso_group_weight * uniform_rand_real(0, 1) * (v_min_loc[i] - v_curr_pos[i]);
                double gradient = gradient_weight * uniform_rand_real(0, 1) * (local_min_loc[i] - v_curr_pos[i]);
                // Velocity is used to determine target position and next PSO update
                new_vel[i] = inertia + p_term + g_term + gradient;
                // This is the new target position to draw a straight (target)
                // line to before the next update
                new_pos[i] = v_curr_pos[i] + new_vel[i] * step_interval;
            }

            // TODO: Normalize velocity here?
            double vel_magnitude = sqrt(pow(new_vel[0], 2) + pow(new_vel[1], 2));

            new_vel = normalize_velocity(new_vel);

            return {new_pos[0], new_pos[1], new_vel[0], new_vel[1]};
        }

        std::vector<double> boids_velocity_update(std::map<Pos, std::vector<double>> neighbor_pos_vel)
        {
            // This is Boids-based flocking algorithm for the robots to used once they've made a
            // decision. At this point, they'll spread their information fastest if they
            // maintain a communication network while spreading out as much as possible.
            // IN: Each pair is a (pos, vel) pair for a robot heard from in this tick (ie currently
            // in communication range)
            // RETURN: The new target position AND velocity to apply to the robot
            uint neighbor_count = neighbor_pos_vel.size();

            std::vector<double> new_vel(2);
            if (neighbor_count > 0)
            {
                double avg_vel_x = 0;
                double avg_vel_y = 0;
                // LENNARD-JONES FORCE FOR COHESION AND SEPARATION
                // double mass = .0000001;
                std::vector<double> lj_acc = lennard_jones_potential(neighbor_pos_vel, comm_range * .75);
                // lj_acc[0] *= mass;
                // lj_acc[1] *= mass;
                lj_acc = normalize_velocity(lj_acc);

                // ALIGNMENT: Compute the average velocity of all neighbors
                // TODO: See if this velocity needs normalization
                for (auto const &s : neighbor_pos_vel)
                {
                    avg_vel_x += s.second[0];
                    avg_vel_y += s.second[1];
                }
                avg_vel_x /= neighbor_count;
                avg_vel_y /= neighbor_count;
                // Align steering vector
                std::vector<double> align_steering = {avg_vel_x - velocity[0],
                                                      avg_vel_y - velocity[1]};
                align_steering = normalize_velocity(align_steering);

                // double acceleration_x = lj_acc[0] + align_steering[0];
                // double acceleration_y = lj_acc[1] + align_steering[1];
                // double acceleration_x = align_steering[0];
                // double acceleration_y = align_steering[1];
                new_vel = {2 * velocity[0] + lj_acc[0] + align_steering[0],
                           2 * velocity[1] + lj_acc[1] + align_steering[1]};
                new_vel = normalize_velocity(new_vel);

                std::cout << "[" << neighbor_count << "]\tVelocity: ("
                          << velocity[0] << "," << velocity[1] << ")\t+ ("
                          << lj_acc[0] << "," << lj_acc[1] << ")\t+ ("
                          << align_steering[0] << "," << align_steering[1] << ")\t= ("
                          << new_vel[0] << "," << new_vel[1] << ")" << std::endl;
            }
            else
            {
                // If no neighbors, set a random velocity
                double angle = uniform_rand_real(0, 2 * PI);
                // convert angle to unit vector
                // new_vel = {cos(angle), sin(angle)};
                new_vel = {uniform_rand_real(-1, 1), uniform_rand_real(-1, 1)};
                // std::cout << "---\t"
                //           << "Velocity: " << new_vel[0] << "," << new_vel[1] << std::endl;
            }

            // This is equlivant to setting a step_interval of 1 (ie update this every tick)
            std::vector<double> target = {curr_pos.x + velocity[0] * boids_step_interval,
                                          curr_pos.y + velocity[1] * boids_step_interval};

            // std::cout << "Target: " << target[0] << ", " << target[1] << std::endl;
            // std::cout << "Velocity: " << new_vel[0] << ", " << new_vel[1] << std::endl;

            return {target[0], target[1], new_vel[0], new_vel[1]};
        }

        std::vector<double> lennard_jones_potential(std::map<Pos, std::vector<double>> neighbor_pos_vel, double target_dist)
        {
            // lj_force derives the Lennard-Jones potential and force based on the relative
            // positions of all neighbors and the desired target_dist to neighbors. The force is a
            // gain factor, attracting or repelling a robot from a neighbor. The center is a point
            // in space toward which the robot will move, based on the sum of all weighted neighbor
            // positions.
            // Get the sum of all weighted neighbor positions
            std::vector<Pos> relative_pos(neighbor_pos_vel.size());
            std::vector<double> dist(neighbor_pos_vel.size());
            auto i = 0;
            for (auto const &n : neighbor_pos_vel)
            {
                relative_pos[i] = {n.first.x - curr_pos.x,
                                   n.first.y - curr_pos.y};
                dist[i] = sqrt(pow(relative_pos[i].x, 2) + pow(relative_pos[i].y, 2));
            }

            // (a=12, b=6) is standard for Lennard-Jones potential. The ratio has to be 2:1
            // lower numbers mean less aggressive repulsion (eg a=6, b=3)
            int a = 6;
            int b = 3;
            int epsilon = 100;                  // depth of the potential well, V_LJ(target_dist) = epsilon
            int gamma = 1;                      // force gain
            double r_const = 1.1 * target_dist; // target distance from neighbors
            double center_x = 0;
            double center_y = 0;
            for (auto i = 0; i < neighbor_pos_vel.size(); i++)
            {
                double r = std::min(r_const, dist[i]);
                r = std::max(r, 1.0); // minimum distance is 1 (not on same cell)
                double f_lj = -gamma * epsilon / r * (a * pow(target_dist / r, a) - 2 * b * pow(target_dist / r, b));
                center_x += f_lj * relative_pos[i].x;
                center_y += f_lj * relative_pos[i].y;
            }
            center_x /= neighbor_pos_vel.size();
            center_y /= neighbor_pos_vel.size();
            return {center_x, center_y};
        }

        std::vector<double> normalize_velocity(std::vector<double> vel)
        {
            // Normalize the velocity to the max_speed
            double speed = sqrt(pow(vel[0], 2) + pow(vel[1], 2));
            if (speed > pso_max_speed)
            {
                vel[0] *= pso_max_speed / speed;
                vel[1] *= pso_max_speed / speed;
            }
            return vel;
        }

        //----------------------------------------------------------------------
        // POST-DECISION
        //----------------------------------------------------------------------

        // Compute whether all the robots in the neighbor_table have found a min_val below the threshold
        bool all_neighbors_decided()
        {
            // std::cout << "neighbor count: " << neighbor_table.size() << std::endl;
            for (auto const &n : neighbor_table)
            {
                if (n.id != 0)
                {
                    if (n.min_val > end_val)
                    {
                        // There's a robots that has not found a min_val below the threshold
                        // so we can stop searching. Don't
                        return false;
                    }
                }
            }
            // We haven't found any undecided neighbors
            return true;
        }

        bool alt_all_neighbors_decided()
        {
            int neighbor_count = 0;
            bool any_undecided = false;
            for (auto const &n : neighbor_table)
            {
                if (n.id != 0)
                {
                    neighbor_count++;
                    if (n.min_val > end_val)
                    {
                        any_undecided = true;
                    }
                }
            }
            // std::cout << "neighbor count: " << neighbor_count << std::endl;
            return any_undecided;
        }

        //----------------------------------------------------------------------
        // PATHS
        //----------------------------------------------------------------------

        std::vector<double> set_pso_path(Pos target, std::vector<double> velocity, int path_len = 0)
        {
            // use set_path (straight line path), but then adapt to do reflections
            // and terminate to number of steps in interval
            if (path_len == 0)
            {
                path_len = step_interval;
            }

            // Set the initial path
            set_path(curr_pos.x, curr_pos.y, target.x, target.y);

            // Terminate unused length of the path
            if (path_len < m_path_to_target.size())
            {
                // If the new path is longer than the target length, cut it short
                m_path_to_target.erase(m_path_to_target.begin(), m_path_to_target.end() - path_len);
                // If the new path is SHORTER, let it stay that length
            }

            // When path hits the edge of the arena, flip the line using the edge as a mirror
            std::vector<int> flips = reflect_path();

            for (auto i = 0; i < 2; i++)
            {
                // iterate over x,y
                int dim_flips = flips[i];
                if (dim_flips)
                {
                    velocity[i] *= -1 * dim_flips;
                }
            }

            return velocity;
        }

        std::vector<int> reflect_path()
        {
            // Flip the m_path_to_target wherever it hits the edge of the arena
            std::reverse(m_path_to_target.begin(), m_path_to_target.end());
            std::vector<int> old_flips = {0, 0};
            std::vector<int> flip_count = {0, 0};
            for (auto i = 0; i < m_path_to_target.size(); i++)
            {
                Pos p = m_path_to_target[i];
                std::vector<int> pos_and_flips = _flip_pos(p);
                m_path_to_target[i] = {pos_and_flips[0], pos_and_flips[1]};
                std::vector<int> new_flips = {pos_and_flips[3], pos_and_flips[4]};
                flip_count = {flip_count[0] + new_flips[0] - old_flips[0],
                              flip_count[1] + new_flips[1] - old_flips[1]};
                old_flips = new_flips;
            }
            std::reverse(m_path_to_target.begin(), m_path_to_target.end());
            return flip_count;
        }

        std::vector<int> _flip_pos(Pos pos)
        {
            // Flip (mirror) a position if it crosses a boundary of the arena
            std::vector<int> grid_dim = {m_arena_grid_width, m_arena_grid_height};
            std::vector<int> vpos = {pos.x, pos.y};
            // Reflect the point over the boundary
            std::vector<int> flip_count = {0, 0}; // flips over x and y
            // Check each axis
            for (auto i = 0; i < 2; i++)
            {
                // Keep flipping until it's within the bounds
                while (vpos[i] < 0 || vpos[i] >= grid_dim[i])
                {
                    if (vpos[i] < 0)
                    {
                        vpos[i] = -vpos[i];
                        flip_count[i] = 1;
                    }
                    else if (vpos[i] >= grid_dim[i])
                    {
                        // (d-1)-(x-(d-1))
                        vpos[i] = 2 * grid_dim[i] - vpos[i] - 2;
                    }
                }
            }

            return {vpos[0], vpos[1], flip_count[0], flip_count[1]};
        }

        //----------------------------------------------------------------------
        // MAPS
        //----------------------------------------------------------------------

        void map_samples(std::map<Pos, double> samples)
        {
            // Put the (position, sample) pairs into the map
            // Loop over the samples in the map
            for (auto const &s : samples)
            {
                if (s.first.x >= 0 && s.first.x < m_arena_grid_width &&
                    s.first.y >= 0 && s.first.y < m_arena_grid_height)
                {
                    m_map[s.first.x][s.first.y] = s.second;
                }
            }
        }

        void update_mins(std::map<Pos, double> pos_samples, bool is_own_obs = false)
        {
            // Update the minimum location and value, if any of the new values are lower
            for (auto const &s : pos_samples)
            {
                if (s.second < min_val && s.second != -1) // (-1=not real obs.)
                {
                    // Set as the minimum if it's lower than anything seen before
                    min_val = s.second;
                    min_loc = s.first;
                    // std::cout << min_loc.x << "," << min_loc.y << " (" << min_val << ")" << std::endl;
                    if (is_own_obs)
                    {
                        // If this came from own observations, set that too
                        obs_min_val = min_val;
                        obs_min_loc = min_loc;
                    }
                }
            }
        }

        void print_map()
        {
            // Print the map to the console
            int len = 5; // total length of a number (to pad)
            for (auto row = m_map.rbegin(); row != m_map.rend(); row++)
            {
                // "row" is a pointer to the row in the map (reverse iteration)
                for (auto val = row->begin(); val != row->end(); val++)
                {
                    // Now I have a pointer to the value
                    std::string val_str = std::to_string(*val);
                    val_str.insert(val_str.begin(), len - val_str.size(), ' ');
                    printf(val_str.c_str());
                }
                printf("\n"); // end row
            }
            printf("\n"); // end map block
        }

        //----------------------------------------------------------------------
        // MISCELLANEOUS
        //----------------------------------------------------------------------

        bool is_finished()
        {
            if (end_condition == "time")
            {
                return get_tick() >= end_val;
            }
            else if (end_condition == "value")
            {
                return min_val <= end_val;
            }
            else
            {
                return false;
            }
        }

    }; // end class
} // namespace Kilosim
