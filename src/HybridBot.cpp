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
        // Pos curr_pos;
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
        // Time to wait before doing first PSO update, so they spread out at start
        int start_interval = 10;

        // PSO Parameters (set by main function)
        // Inertia (omega) from 0-1(ish)
        double pso_inertia; // 1.2
        // Constant personal cognition (weight)
        double pso_self_weight; // 0.0
        // Constant collective/group cognition (weight)
        double pso_group_weight; // 0.0
        // Current PSO velocity (for next step)
        // TODO: Starting PSO velocity might be set by parameters
        std::vector<double> pso_velocity = {uniform_rand_real(-1, 1),
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
        Pos min_loc;
        // Lowest value from OWN observations
        uint obs_min_val = 99999;
        Pos obs_min_loc;
        // Lowest value from RECEIVED messages
        uint rx_min_val = 99999;
        Pos rx_min_loc;

        // ---------------------------------------------------------------------

        // Set up things used in common between all decision-making robots
        // TODO: Run setup (self.setup())

    private:
        // States
        static const uint8_t INIT = 0;
        static const uint8_t DO_PSO = 1;
        static const uint8_t DECIDED = 2;
        static const uint8_t HOME = 3;

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

            initialize_neighbor_array();
        };

        void loop()
        {
            // Update positions
            prev_pos = curr_pos;
            curr_pos = get_pos();

            if (m_state == INIT)
            {
                std::map<Pos, double> pos_samples = sample_around();
                map_samples(pos_samples);
                update_mins(pos_samples);
                // Set the velocity on the first take (angling away from origin)
                if (get_tick() == 1)
                {
                    std::vector<double> tmp_vel = {
                        start_interval * cos(start_angle),
                        start_interval * sin(start_angle)};
                    target_pos = {2 * tmp_vel[0], 2 * tmp_vel[1]};
                    set_pso_path(target_pos, tmp_vel, start_interval);
                    target_pos = m_path_to_target[0]; // target is last position in path
                }
                else if (get_tick() >= start_interval || min_val < end_val)
                {
                    m_state = DO_PSO;
                    printf("init finished\n");
                }
            }
            else if (m_state == DO_PSO)
            {
                std::map<Pos, double> pos_samples = sample_around();
                map_samples(pos_samples);
                update_mins(pos_samples);
                int tick = get_tick();
                if (tick % step_interval == 1)
                {
                    std::vector<double> new_pos_vel = velocity_update(pos_samples);
                    target_pos = {new_pos_vel[0], new_pos_vel[1]};
                    pso_velocity = {new_pos_vel[2], new_pos_vel[3]};

                    set_pso_path(target_pos, pso_velocity);
                }
                if (is_finished())
                {
                    m_state = DECIDED;
                    printf("decision finished\n");
                }
            }
            else if (m_state == DECIDED)
            {
                set_led(0, 100, 0);
                // printf("decided...\n");
                // Found a target; go home
                // TODO: change to Boids
                target_pos = {0, 0};
                // velocity (={0,0}) doesn't matter here
                // path_len just has to be longer that whatever it generates
                set_pso_path(target_pos, {0, 0}, 1000);
                if (curr_pos == target_pos)
                {
                    m_state = HOME;
                    printf("Home\n");
                }
            }
            else if (m_state == HOME)
            {
                // nothing to do
            }

            // Call every loop
            // Process all messages in the queue since the last tick
            process_msgs();
            prune_neighbor_table();

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

        void process_msgs()
        {
            // Process all messages since the last tick
            // Add each one to the table
            std::vector<json> new_msgs = get_msg();
            for (auto i = 0; i < new_msgs.size(); i++)
            {
                json msg = new_msgs[i];
                add_msg_to_table(msg);
            }
        }

        void add_msg_to_table(json msg)
        {
            // TODO: Should probably invert this, because neighbor_table is longer to loop over
            // than new_msgs list is
            uint id = msg.at("id");
            for (auto i = 0; i < neighbor_table.size(); i++)
            {
                if (neighbor_table[i].id == id)
                {
                    // Neighbor is already in the table
                    // Update the value and stop looking
                    neighbor_table[i].min_loc_x = msg.at("min_loc_x");
                    neighbor_table[i].min_loc_y = msg.at("min_loc_y");
                    neighbor_table[i].min_val = msg.at("min_val");
                    neighbor_table[i].tick_added = msg.at("tick_added");
                    break;
                }
            }
        }

        void prune_neighbor_table()
        {
            // Remove old messages, based on some fixed time since they were added
            int curr_tick = get_tick();
            for (auto i = 0; i < neighbor_table.size(); i++)
            {

                if (neighbor_table[i].tick_added + rx_table_timeout >= curr_tick)
                {
                    neighbor_table[i].id = 0;
                }
            }
        }

        // Send message (continuously)
        // message_t *message_tx()
        // {
        //     Message should be changed/set by respective detection/observation function
        //     if (is_feature_disseminating)
        //     {
        //         update_tx_message_data();
        //         return &tx_message_data;
        //     }

        //     else
        //     {
        //         return NULL;
        //     }
        // }

        //----------------------------------------------------------------------
        // PSO
        //----------------------------------------------------------------------

        std::vector<double> velocity_update(std::map<Pos, double> samples)
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
            std::vector<int> new_pos = {0, 0};
            // Convert to vectors for
            std::vector<int> v_obs_min_loc = {obs_min_loc.x, obs_min_loc.y};
            std::vector<int> v_min_loc = {min_loc.x, min_loc.y};
            std::vector<int> v_curr_pos = {curr_pos.x, curr_pos.y};

            // Compute PSO
            for (auto i = 0; i < 2; i++)
            {
                // TODO: This is PSO + GD. Use a different function for Boids
                double inertia = pso_inertia + pso_velocity[i];
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

            if (vel_magnitude > pso_max_speed)
            {
                new_vel[0] *= pso_max_speed / vel_magnitude;
                new_vel[1] *= pso_max_speed / vel_magnitude;
            }

            return {new_pos[0], new_pos[1], new_vel[0], new_vel[1]};
        }

        //----------------------------------------------------------------------
        // MOVEMENT
        //----------------------------------------------------------------------

        // I think that this functionality is already taken care of by the position update in
        // kilosim-gridbots (Gridbot.robot_compute_next_step)

        // void move_toward_target()
        // {
        //     // Use the internal m_path_to_target to determine the robot's next step
        //     bool use_path = true; // In future, could use alternative
        //     if (curr_pos == target_pos)
        //     {
        //         // At target -- don't move
        //         move(0, 0);
        //     }
        //     else
        //     {
        //         Pos pos_diff;
        //         if (m_path_to_target.size() != 0)
        //         {
        //             // Use the path if one exists
        //             Pos next_pos = m_path_to_target.back();
        //             m_path_to_target.pop_back();
        //             pos_diff = {next_pos.x - curr_pos.x,
        //                         next_pos.y - curr_pos.y};
        //         }
        //         else
        //         {
        //             // If no path, move in direction of target
        //             pos_diff = {target_pos.x - curr_pos.x,
        //                         target_pos.y - curr_pos.y};
        //         }
        //         // Hacky(ish) way of getting the sign as -1/0/+1
        //         // See: https://stackoverflow.com/a/1903975/2552873
        //         std::cout << (pos_diff.x > 0) - (pos_diff.x < 0) << ", " << (pos_diff.y > 0) - (pos_diff.y < 0) << std::endl;
        //         move((pos_diff.x > 0) - (pos_diff.x < 0),
        //              (pos_diff.y > 0) - (pos_diff.y < 0));
        //     }
        // }

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
