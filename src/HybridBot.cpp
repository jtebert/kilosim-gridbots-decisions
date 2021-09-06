/*
 * Gridbots doing target localization (min/max)
 *
 * Hybrid algorithm is PSO + gradient descent + Boids
 *
 * Created 2020-07 by Julia Ebert
 */

#include "Gridbot.h"
#include "BaseBot.h"

#include <cmath>

namespace Kilosim
{

    typedef struct pos_vel
    {
        // A pair of position and velocity
        Pos pos;
        std::vector<double> vel;
    } pos_vel;

    class HybridBot : public BaseBot
    {
    public:
        // Variables for aggregators
        // TODO: Add aggregators
        // Pos curr_pos;

        // ---------------------------------------------------------------------

        // TODO: Add anything here that needs to be set by main (ie externally)
        // (Particularly parameters from external config files)

        // Every how many ticks to update the PSO target/velocity
        int pso_step_interval;
        int boids_step_interval;
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
        std::vector<double> velocity;
        // Maximum allowed magnitude of the PSO internal velocity (per tick)
        double max_speed; // 25

        // GRADIENT DESCENT parameters
        double gradient_weight;

        // POST-DECISION MOVEMENT OPTIONS
        std::string post_decision_movement;

        // BOIDS PARMETERS
        double lj_a;
        double lj_b;
        double lj_epsilon;
        double lj_gamma;

        // MISCELLANEOUS
        int num_neighbors;    // Used for making neighbor array table
        int rx_table_timeout; // Time neighbor messages stay in table

        // Data values
        // min_val and min_loc are defined in BaseBot
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
            std::cout << "Own ID=" << id << "\tv=" << min_val << "\tstate=" << (int)m_state
                      << "\t(t=" << get_tick() << ")"
                      << " (" << get_pos().x << "," << get_pos().y << ")" << std::endl;
            std::cout << "ID\tMinX\tMinY\tMinVal\tCurrX\tCurrY\tTickAdded" << std::endl;
            for (auto &n : neighbor_table)
            {
                if (n.id != 0)
                {
                    std::cout << n.id << "\t" << n.min_loc_x << "\t" << n.min_loc_y << "\t" << n.min_val
                              << "\t" << n.curr_pos_x << "\t" << n.curr_pos_y << "\t" << n.tick_added << std::endl;
                }
            }
            printf("\n");
        }

    private:
        // Utility attributes/variables

        // Previous position (current position curr_pos is public for aggregators)
        Pos prev_pos;
        Pos curr_pos;
        // Where the robot is going (used by move_toward_target)
        Pos target_pos;

        void setup()
        {
            set_led(100, 0, 0);
            // curr_pos = get_pos();

            m_state = INIT;
        };

        void loop()
        {
            // Update positions
            prev_pos = curr_pos;
            curr_pos = get_pos();

            // EVERY TICK: Sample around the robot, then mark the observed/visited positions
            // This is used by aggregators to determine collective coverage
            std::map<Pos, double> pos_samples = sample_around();
            map_coverage(pos_samples);
            if (prev_pos != curr_pos)
            {
                map_visited(get_pos());
            }

            if (m_state == INIT)
            {
                // Set the velocity on the first take (angling away from origin)
                initialize_neighbor_array();
                // Current (starting) angle and velocity
                double start_angle = uniform_rand_real(5 * PI / 180, 85 * PI / 180);
                std::vector<double> tmp_vel = {
                    start_interval * cos(start_angle),
                    start_interval * sin(start_angle)};
                target_pos = {2 * tmp_vel[0], 2 * tmp_vel[1]};
                set_pso_path(target_pos, tmp_vel, start_interval);
                target_pos = m_path_to_target[0]; // target is last position in path
                velocity = normalize_velocity({uniform_rand_real(-max_speed, max_speed),
                                               uniform_rand_real(-max_speed, max_speed)});
                m_state = SPREAD;
            }
            // else if (m_state == INIT)
            // {
            //     // Alternative INIT state: set a target as a random position in the arena
            //     initialize_neighbor_array();
            //     // Pick a random position in the arena
            //     target_pos = {uniform_rand_real(0, m_arena_grid_width),
            //                   uniform_rand_real(0, m_arena_grid_height)};
            //     // Current position is (0,0), so velocity vector toward target is target_pos.
            //     // Just normalize it to get the "valid" velocity. (But have to convert to double)
            //     // std::vector<double> tmp_vel = normalize_velocity({target_pos.x, target_pos.y});
            //     velocity = normalize_velocity({uniform_rand_real(-max_speed, max_speed),
            //                                    uniform_rand_real(-max_speed, max_speed)});
            //     // Max path length is the diagonal of the arena
            //     velocity = set_pso_path(target_pos, velocity, sqrt(pow(m_arena_width, 2) + pow(m_arena_height, 2)));
            //     m_state = SPREAD;
            //     set_led(100, 0, 100);
            // }
            else if (m_state == SPREAD)
            {
                // Spread away from the origin before doing PSO
                neighbor_count = process_msgs();
                update_mins(pos_samples);
                if (is_time_to_go_home())
                {
                    m_state = SEND_HOME;
                }
                // else if (get_tick() >= start_interval || min_val < end_val)
                else if (curr_pos == target_pos || min_val < end_val)
                {
                    m_state = DO_PSO;
                    set_led(100, 0, 0);
                }
            }
            else if (m_state == DO_PSO)
            {
                int tick = get_tick();
                neighbor_count = process_msgs();
                update_mins(pos_samples);
                if (tick % pso_step_interval == 1)
                {
                    pos_vel new_pos_vel = pso_velocity_update(pos_samples);
                    target_pos = new_pos_vel.pos;
                    velocity = set_pso_path(target_pos, new_pos_vel.vel);
                }
                if (is_time_to_go_home())
                {
                    m_state = SEND_HOME;
                }
                else if (is_finished())
                {
                    m_state = DECIDED;
                    set_led(0, 100, 0);
                }
            }
            else if (m_state == DECIDED)
            {
                // Share the decision with neighbors (aka do Boids)
                neighbor_count = process_msgs();
                update_mins(pos_samples);

                // Various post-decision movement options
                if (post_decision_movement == "hybrid")
                {
                    // Keep doing the same thing as before making decision
                    // (copied from DO_PSO state)
                    if ((int)get_tick() % pso_step_interval == 2)
                    {
                        pos_vel new_pos_vel = pso_velocity_update(pos_samples);
                        target_pos = new_pos_vel.pos;
                        velocity = set_pso_path(target_pos, new_pos_vel.vel);
                    }
                }
                else if ((int)get_tick() % boids_step_interval == 2)
                {
                    // Use neighbor pos + velocity SINCE LAST BOIDS UPDATE
                    std::map<uint16_t, pos_vel> new_neighbor_pos_vel;
                    int curr_tick = get_tick();
                    for (auto &n : neighbor_table)
                    {
                        if (n.tick_added >= curr_tick - boids_step_interval)
                        {
                            new_neighbor_pos_vel.insert({n.id, {{n.curr_pos_x, n.curr_pos_y}, {n.curr_vel_x, n.curr_vel_y}}});
                        }
                    }
                    if (post_decision_movement == "flock")
                    {
                        // Use Boids-based flocking
                        double grid_comm_range = comm_range / 10;
                        pos_vel new_pos_vel = boids_velocity_update(new_neighbor_pos_vel, grid_comm_range * .75);
                        target_pos = new_pos_vel.pos;
                        velocity = set_pso_path(target_pos, new_pos_vel.vel, boids_step_interval);
                    }
                    else if (post_decision_movement == "disperse")
                    {
                        // Use just the LJ component of Boids, but with big target distance
                        // pos_vel new_pos_vel = boids_velocity_update(new_neighbor_pos_vel, comm_range * 10);
                        pos_vel new_pos_vel = dispersion_velocity_update(new_neighbor_pos_vel);
                        target_pos = new_pos_vel.pos;
                        velocity = set_pso_path(target_pos, new_pos_vel.vel, boids_step_interval);
                    }
                    else
                    {
                        printf("INVALID post_decision_movement: must be hybrid, flock, or disperse\n");
                    }
                }

                // std::cout << get_tick() << " " << id << " (" << get_pos().x << "," << get_pos().y << "): " << velocity[0] << "," << velocity[1] << std::endl;
                // This state is only used when end_val != time, so don't need to check is_time_to_go_home()
                if (all_neighbors_decided())
                {
                    m_state = SEND_HOME;
                }
            }
            else if (m_state == SEND_HOME)
            {
                // Prepape for the GO_HOME state
                target_pos = home_pos;
                set_pso_path(target_pos, {0, 0}, 1000);
                set_led(0, 0, 100);
                m_state = GO_HOME;
            }
            else if (m_state == GO_HOME)
            {
                // Everyone knows about the target; go home
                // velocity (={0,0}) doesn't matter here
                // path_len just has to be longer that whatever it generates
                get_msg(); // clear the queue
                if (get_pos() == target_pos)
                {
                    m_state = HOME;
                }
            }
            else if (m_state == HOME)
            {
                // Nothing to do
                // But this lets the is_home() function know that the robot is totally done.
                get_msg(); // clear the queue
            }

            prune_neighbor_table();
            update_send_msg();
        };

        //----------------------------------------------------------------------
        // COMMUNICATION
        //----------------------------------------------------------------------

        void initialize_neighbor_array()
        {
            // Set up the neighbor array
            neighbor_table.resize(0);
            // neighbor_table.resize(num_neighbors);
            // for (auto i = 0; i < neighbor_table.size(); i++)
            // {
            //     // Assigning all IDs to 0 indicates no robot in that row
            //     // (otherwise random ID from memory allocation could make it a mess)
            //     neighbor_table[i].id = 0;
            // }
        }

        void add_neighbor(json msg, unsigned long int ind, bool from_table = false)
        {
            // Add a message to the neighbor table and check if it gives a new min value/location
            // NOTE: ind is the index of the neighbor in the neighbor table
            if (ind >= neighbor_table.size())
            {
                neighbor_table.resize(ind + 1);
            }
            neighbor_table[ind].id = msg.at("id");
            neighbor_table[ind].min_loc_x = msg.at("min_loc_x");
            neighbor_table[ind].min_loc_y = msg.at("min_loc_y");
            neighbor_table[ind].min_val = msg.at("min_val");
            neighbor_table[ind].curr_pos_x = msg.at("curr_pos_x");
            neighbor_table[ind].curr_pos_y = msg.at("curr_pos_y");
            neighbor_table[ind].curr_vel_x = msg.at("curr_vel_x");
            neighbor_table[ind].curr_vel_y = msg.at("curr_vel_y");

            if (from_table)
            {
                neighbor_table[ind].tick_added = msg.at("tick_added");
                neighbor_table[ind].from_neighbor = true;
            }
            else
            {
                neighbor_table[ind].tick_added = get_tick();
                neighbor_table[ind].from_neighbor = false;
            }
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

            auto iterator = neighbor_table.begin();
            while (iterator != neighbor_table.end())
            {
                // remove expired messages
                if (iterator->tick_added + rx_table_timeout <= curr_tick)
                {
                    iterator = neighbor_table.erase(iterator);
                }
                else
                {
                    ++iterator;
                }
            }
            // for (auto i = 0; i < neighbor_table.size(); i++)
            // {

            //     if (neighbor_table[i].tick_added + rx_table_timeout <= curr_tick)
            //     {
            //         neighbor_table[i].id = 0;
            //     }
            // }
        }

        // Send message (continuously)
        void update_send_msg()
        {
            // Only set the message if a value has been observed (i.e. not max)
            if (min_val < 99999)
            {
                // Create message to send
                Pos curr_pos = get_pos();
                json msg;
                msg["id"] = id;
                msg["min_loc_x"] = min_loc.x;
                msg["min_loc_y"] = min_loc.y;
                msg["min_val"] = min_val;
                msg["curr_pos_x"] = curr_pos.x;
                msg["curr_pos_y"] = curr_pos.y;
                msg["curr_vel_x"] = velocity[0];
                msg["curr_vel_y"] = velocity[1];
                msg["neighbors"] = neighbors_to_json();
                send_msg(msg);
            }
        }

        json neighbors_to_json()
        {
            // Convert the neighbor table to a json object
            json neighbors;
            for (auto i = 0; i < neighbor_table.size(); i++)
            {
                json neighbor;
                if (neighbor_table[i].id != 0)
                {
                    neighbor["id"] = neighbor_table[i].id;
                    neighbor["min_loc_x"] = neighbor_table[i].min_loc_x;
                    neighbor["min_loc_y"] = neighbor_table[i].min_loc_y;
                    neighbor["min_val"] = neighbor_table[i].min_val;
                    neighbor["tick_added"] = neighbor_table[i].tick_added;
                    neighbor["curr_pos_x"] = neighbor_table[i].curr_pos_x;
                    neighbor["curr_pos_y"] = neighbor_table[i].curr_pos_y;
                    neighbor["curr_vel_x"] = neighbor_table[i].curr_vel_x;
                    neighbor["curr_vel_y"] = neighbor_table[i].curr_vel_y;
                    neighbors.push_back(neighbor);
                }
            }
            return neighbors;
        }

        //----------------------------------------------------------------------
        // MOVEMENT
        //----------------------------------------------------------------------

        pos_vel pso_velocity_update(std::map<Pos, double> samples)
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
            Pos curr_pos = get_pos();
            std::vector<int> v_curr_pos = {curr_pos.x, curr_pos.y};

            // Compute PSO (across 2 dimensions)
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
                new_pos[i] = v_curr_pos[i] + new_vel[i] * pso_step_interval;
            }

            // TODO: Normalize velocity here?
            double vel_magnitude = sqrt(pow(new_vel[0], 2) + pow(new_vel[1], 2));

            new_vel = normalize_velocity(new_vel);
            Pos ret_pos = Pos(new_pos[0], new_pos[1]);

            return {ret_pos, new_vel};
        }

        pos_vel dispersion_velocity_update(std::map<uint16_t, pos_vel> neighbor_pos_vel)
        {
            // Velocity update based on dispersion: get the average velocity of all neighbors and go the opposite direction

            // This sets the comm_range to 10x the default comm_range in grid space!!
            std::vector<double> new_vel = {0, 0};
            if (neighbor_pos_vel.size() > 0)
            {
                double grid_comm_range = comm_range;
                std::vector<double> lj_acc = lennard_jones_potential(neighbor_pos_vel, grid_comm_range);
                // lj_acc = normalize_velocity(lj_acc);

                // lj_acc[0] *= mass;
                new_vel = {1 * velocity[0] + 1 * lj_acc[0],
                           1 * velocity[1] + 1 * lj_acc[1]};
                new_vel = normalize_velocity(new_vel);
                Pos curr_pos = get_pos();
            }
            else
            {
                // If no neighbors, add a velocity component in a random direction
                // (Should match boids_velocity_update)
                double angle = uniform_rand_real(0, 2 * PI);
                // convert angle to unit vector
                new_vel = {velocity[0] + 0.5 * cos(angle),
                           velocity[1] + 0.5 * sin(angle)};
                new_vel = normalize_velocity(new_vel);
                // new_vel = velocity;
            }
            Pos target = {curr_pos.x + new_vel[0] * boids_step_interval,
                          curr_pos.y + new_vel[1] * boids_step_interval};
            return {target, new_vel};
        }

        pos_vel boids_velocity_update(std::map<uint16_t, pos_vel> neighbor_pos_vel, double target_dist)
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
                // LENNARD-JONES FORCE FOR COHESION AND SEPARATION

                std::vector<double> lj_acc = lennard_jones_potential(neighbor_pos_vel, target_dist);
                // lj_acc[0] *= mass;
                lj_acc = normalize_velocity(lj_acc);

                // ALIGNMENT: Compute the average velocity of all neighbors
                double avg_vel_x = 0;
                double avg_vel_y = 0;
                for (auto const &n : neighbor_pos_vel)
                {
                    avg_vel_x += n.second.vel[0];
                    avg_vel_y += n.second.vel[1];
                }
                avg_vel_x /= neighbor_count;
                avg_vel_y /= neighbor_count;
                // std::vector<double> avg_vel = normalize_velocity({avg_vel_x, avg_vel_y});
                std::vector<double> avg_vel = {avg_vel_x, avg_vel_y};
                // Align steering vector
                std::vector<double> align_steering = {avg_vel[0] - velocity[0],
                                                      avg_vel[1] - velocity[1]};
                align_steering = normalize_velocity(align_steering);
                // std::cout << "align_steering: " << align_steering[0] << ", " << align_steering[1] << std::endl;

                new_vel = {2 * velocity[0] + 1 * lj_acc[0] + 1 * align_steering[0],
                           2 * velocity[1] + 1 * lj_acc[1] + 1 * align_steering[1]};
                // new_vel = {.5 * velocity[0] + .5 * lj_acc[0] + .5 * align_steering[0],
                //            .5 * velocity[1] + .5 * lj_acc[1] + .5 * align_steering[1]};
                // new_vel = {velocity[0] + align_steering[0],
                //            velocity[1] + align_steering[1]};
                // new_vel = lj_acc;
                // new_vel = {align_steering[0],
                //            align_steering[1]};

                new_vel = normalize_velocity(new_vel);

                // FOR DEBUGGING BOIDS
                // std::cout << get_tick() << " " << id << "\t[" << neighbor_count << "]\t(" << get_pos().x << "," << get_pos().y << "):\t("
                //           << velocity[0] << "," << velocity[1] << ")\t+ ("
                //           << lj_acc[0] << "," << lj_acc[1] << ")\t+ ("
                //           << align_steering[0] << "," << align_steering[1] << ")\t= ("
                //           << new_vel[0] << "," << new_vel[1] << ")" << std::endl;
                // print_neighbor_array();
            }
            else
            {
                // If no neighbors, add a velocity component in a random direction
                // (Should match dispersion_velocity_update)
                double angle = uniform_rand_real(0, 2 * PI);
                // convert angle to unit vector
                new_vel = {velocity[0] + 0.5 * cos(angle),
                           velocity[1] + 0.5 * sin(angle)};
                new_vel = normalize_velocity(new_vel);

                // new_vel = {pso_inertia * velocity[0] + uniform_rand_real(-10, 10),
                //            pso_inertia * velocity[1] + uniform_rand_real(-10, 10)};
                // new_vel = normalize_velocity(new_vel);

                // TEMPORARY: FOR TESTING
                // (keep the same velocity if you don't hear from other robots)
                // new_vel = velocity;

                // std::cout << "---\t"
                //           << "(" << get_pos().x << "," << get_pos().y << "):\t" << new_vel[0] << "," << new_vel[1] << std::endl;
            }

            // This is equivalent to setting a step_interval of 1 (ie update this every tick)
            Pos curr_pos = get_pos();
            Pos target = {curr_pos.x + new_vel[0] * boids_step_interval,
                          curr_pos.y + new_vel[1] * boids_step_interval};

            // std::cout << "Target: " << target[0] << ", " << target[1] << std::endl;
            // std::cout << "Velocity: " << new_vel[0] << ", " << new_vel[1] << std::endl;
            // pos_vel ret(target, new_vel);

            return {target, new_vel};
        }

        std::vector<double> lennard_jones_potential(std::map<uint16_t, pos_vel> neighbor_pos_vel, double target_dist)
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
            Pos curr_pos = get_pos();
            for (auto const &n : neighbor_pos_vel)
            {
                relative_pos[i] = {n.second.pos.x - curr_pos.x,
                                   n.second.pos.y - curr_pos.y};
                // This one (below) is wrong/swapped
                // relative_pos[i] = {curr_pos.x - n.first.x,
                //                    curr_pos.y - n.first.y};
                dist[i] = sqrt(pow(relative_pos[i].x, 2) + pow(relative_pos[i].y, 2));
                // dist[i] = std::sqrt(std::pow(relative_pos[i].x, 2) + std::pow(relative_pos[i].y, 2));
                i++;
            }

            // (a=12, b=6) is standard for Lennard-Jones potential. The ratio has to be 2:1
            // lower numbers mean less aggressive repulsion (eg a=6, b=3)
            // THESE NOW COME FROM A CONFIG FILE
            // double a = 6;
            // double b = 3;
            // double epsilon = 1; // depth of the potential well, V_LJ(target_dist) = epsilon
            // double gamma = 1;   // force gain
            // double r_const = .8 * target_dist; // target distance from neighbors
            double r_const = 1.1 * target_dist;
            double center_x = 0;
            double center_y = 0;
            for (auto i = 0ul; i < neighbor_pos_vel.size(); i++)
            {
                double r = std::min(r_const, dist[i]);
                r = std::max(r, 1.0); // minimum distance is 1 (not on same cell)
                // Dividing by r here and then multiplying by the relative_pos vector later on is equivalent to
                // multiplying by the unit vector in the direction of the neighbor (relative_pos[i])
                // Using r instead of dist prevents divide by zero errors, but if they're on the same cell, the
                // relative_pos vector will be zero anyway and the force goes to zero (which it really shouldn't, but
                // that's an issue with putting things on a grid where positions can be on the same cell).
                double f_lj = -(lj_gamma * lj_epsilon / r) * ((lj_a * std::pow(target_dist / r, lj_a)) - (2 * lj_b * std::pow(target_dist / r, lj_b)));
                center_x += f_lj * relative_pos[i].x;
                center_y += f_lj * relative_pos[i].y;
            }

            center_x /= neighbor_pos_vel.size();
            center_y /= neighbor_pos_vel.size();

            // if (id == 21804)
            // {
            // for (auto i = 0; i < neighbor_pos_vel.size(); i++)
            // {
            //     std::cout << relative_pos[i].x << "," << relative_pos[i].y << " -> " << dist[i] << std::endl;
            // }
            // }

            return {center_x, center_y};
        }

        std::vector<double> normalize_velocity(std::vector<double> vel)
        {
            // Normalize the velocity to the max_speed
            double speed = sqrt(pow(vel[0], 2) + pow(vel[1], 2));
            // vel / speed = vel / ||vel|| => unit vector
            // Then multiply by max_speed to set the vector's magnitude to max_speed
            // This reduces the magnitude of the vector to max_speed if it's too large,
            // but retains the direction of the vector.
            if (speed > max_speed)
            {
                if (vel[0] != 0)
                {
                    vel[0] *= max_speed / speed;
                }
                if (vel[1] != 0)
                {
                    vel[1] *= max_speed / speed;
                }
            }
            // std::cout << "Normalized velocity: " << vel[0] << ", " << vel[1] << std::endl;
            return vel;
        }

        //----------------------------------------------------------------------
        // POST-DECISION
        //----------------------------------------------------------------------

        //----------------------------------------------------------------------
        // PATHS
        //----------------------------------------------------------------------

        std::vector<double> set_pso_path(Pos target, std::vector<double> vel, int path_len = 0)
        {
            // use set_path (straight line path), but then adapt to do reflections
            // and terminate to number of steps in interval
            if (path_len == 0)
            {
                path_len = pso_step_interval;
            }

            // Set the initial path
            Pos curr_pos = get_pos();
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
                    vel[i] *= -1 * dim_flips;
                }
            }

            return vel;
        }

        std::vector<int> reflect_path()
        {
            // Flip the m_path_to_target wherever it hits the edge of the arena
            std::reverse(m_path_to_target.begin(), m_path_to_target.end());
            // flips from all previous positions
            std::vector<int> old_flips = {0, 0};
            // flips to get to current position
            std::vector<int> flip_count = {0, 0};
            for (auto i = 0; i < m_path_to_target.size(); i++)
            {
                Pos p = m_path_to_target[i];
                std::vector<int> pos_and_flips = flip_pos(p);
                m_path_to_target[i] = {pos_and_flips[0], pos_and_flips[1]};
                std::vector<int> new_flips = {pos_and_flips[2], pos_and_flips[3]};
                flip_count = {flip_count[0] + new_flips[0] - old_flips[0],
                              flip_count[1] + new_flips[1] - old_flips[1]};
                old_flips = new_flips;
            }
            std::reverse(m_path_to_target.begin(), m_path_to_target.end());
            return flip_count;
        }

        std::vector<int> flip_pos(Pos pos)
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
                        flip_count[i] += 1;
                    }
                    else if (vpos[i] >= grid_dim[i])
                    {
                        // (d-1)-(x-(d-1))
                        vpos[i] = 2 * grid_dim[i] - vpos[i] - 2;
                        flip_count[i] += 1;
                    }
                }
            }

            return {vpos[0], vpos[1], flip_count[0], flip_count[1]};
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

    }; // end class
} // namespace Kilosim
