/*
 * Gridbots doing target localization (min/max)
 *
 * Hybrid algorithm is PSO + gradient descent + Boids
 *
 * Created 2020-07 by Julia Ebert
 */

#include "Gridbot.h"

#include <cmath>

// Pseudo-booleans for convenience
#define FALSE 0
#define TRUE 1

namespace Kilosim
{
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

        // Threshold for ending (declaring target found)
        int end_val = 0;

        // PSO Parameters (set by main function)
        // Inertia (omega) from 0-1(ish)
        double pso_inertia; // 1.2
        // Constant personal cognition (weight)
        double pso_self_weight; // 0.0
        // Constant collective/group cognition (weight)
        double pso_group_weight; // 0.0
        // Maximum allowed magnitude of the PSO internal velocity (per tick?)
        // TODO: Not sure about the magnitude of this max_speed (self.max_pso_vel)
        double pso_max_speed; // 25
        // Current (starting) angle and velocity
        double start_angle = uniform_rand_real(5 * PI / 180, 85 * PI / 180);

        // Data values
        // Minimum value (global/collective) and its location
        int min_val = 99999; // start higher than anything observable
        Pos min_loc;
        // Lowest value from OWN observations
        int obs_min_val = 99999;
        Pos obs_min_loc;
        // Lowest value from RECEIVED messages
        int rx_min_val = 99999;
        Pos rx_min_loc;

        // ---------------------------------------------------------------------

        // Set up things used in common between all decision-making robots
        // TODO: Run setup (self.setup())

    private:
        // States
        static const uint8_t INIT = 0;
        static const uint8_t DO_PSO = 1;
        static const uint8_t DECIDED = 2;

        // Utility attributes/variables
        // Previous position (current position curr_pos is public for aggregators)
        Pos prev_pos;

        uint8_t m_state = INIT;

        void setup()
        {
            move(1, 1);
            set_led(100, 0, 0);
            curr_pos = get_pos();

            // Initialize map
            // TODO: Get/Set map size and fill with -1 (best equivalent to np.nan)
        };

        void loop()
        {
            // Update positions
            prev_pos = curr_pos;
            curr_pos = get_pos();
            // std::cout << prev_pos[0] << curr_pos[0] << std::endl;

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
                    Pos target_pos = {2 * tmp_vel[0], 2 * tmp_vel[1]};
                    set_pso_path(target_pos, tmp_vel, start_interval);
                }
                else if (get_tick() >= start_interval || min_val < end_val)
                {
                    m_state = DO_PSO;
                }
                // print_map();
            }
            else if (m_state == DO_PSO)
            {
            }
            else if (m_state == DECIDED)
            {
            }

            // Call every loop
            // move_toward_target();
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

        // TODO: Add comm_criteria

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
            std::cout << target.x << ", " << target.y << std::endl;
            // Set the initial path
            set_path(curr_pos.x, curr_pos.y, target.x, target.y);

            // Terminate unused length of the path
            m_path_to_target.erase(m_path_to_target.begin(), m_path_to_target.end() - path_len);

            std::cout << path_len << std::endl;

            printf("Original Path:\n");
            for (auto i = 0; i < m_path_to_target.size(); i++)
            {
                std::cout << m_path_to_target[i].x << ", " << m_path_to_target[i].y << std::endl;
            }
            printf("\n");

            // When path hits the edge of the arena, flip the line using the edge as a mirror
            std::vector<int> flips = reflect_path();

            printf("Final Path:\n");
            for (auto i = 0; i < m_path_to_target.size(); i++)
            {
                std::cout << m_path_to_target[i].x << ", " << m_path_to_target[i].y << std::endl;
            }
            printf("\n");

            double vel_magnitude = sqrt(pow(velocity[0], 2) + pow(velocity[1], 2));
            for (auto i = 0; i < 2; i++)
            {
                // iterate over x,y
                int dim_flips = flips[i];
                if (dim_flips)
                {
                    velocity[i] *= -1 * dim_flips;
                }
                if (abs(vel_magnitude) > pso_max_speed)
                {
                    velocity[i] *= pso_max_speed / vel_magnitude;
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

            std::cout << pos.x << "," << pos.y << " -> " << vpos[0] << "," << vpos[1] << std::endl;

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

    }; // end class
} // namespace Kilosim
