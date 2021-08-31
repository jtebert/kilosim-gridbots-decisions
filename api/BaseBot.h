#ifndef BASEBOT_H
#define BASEBOT_H

#include "Gridbot.h"

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
        int tick_added;
        bool from_neighbor;
    } neighbor_info_t;

    class BaseBot : public Gridbot
    {
        // This is an intermediate class between Gridbot and SweepBot/HybridBot.
        // Right now, its purpose is to provide a common interface for the aggregator functions to cast to, to avoid re-implementing the functions for each bot.

    public:
        int neighbor_count = 0; // number of neighbors heard from on the current tick

        // DECISION-MAKING parameters
        std::string end_condition; // "value" or "time"
        // Threshold for ending (declaring target found)
        // For "value": decision if min_val is <= end_val, decision is made
        // For "time": decision if get_tick() >= end_val
        int end_val;

        // Data values
        // Minimum value (global/collective) and its location
        uint min_val = 99999; // start higher than anything observable
        Pos min_loc = {-1, -1};

        // Home position is whether the robot goes to at end (and probably where it starts)
        Pos home_pos = {0, 0};

        uint8_t m_state;

        bool is_finished()
        {
            if (end_condition == "time")
            {
                return get_tick() >= end_val;
            }
            else if (end_condition == "value" || end_condition == "first_find")
            {
                return min_val <= end_val;
            }
            else
            {
                return false;
            }
        }

        std::vector<int> get_neighbors()
        {
            // Return a list of the IDs of all the neighbors heard from in this tick
            return neighbor_id_list;
        }

        int get_state()
        {
            // Get the robot's state, which tells where it is in the decision-making process
            // and where it is in the world (ie has it gone home yet or not)
            return m_state;
        }

        std::vector<std::vector<int>> get_map()
        {
            // Return the map (to be used my accumulators in main.cpp)
            return m_map;
        }

        int map_coverage_count()
        {
            // Count the number of cells that the robot has sensed (using the map)
            // Sum up the non-zero values in the map
            int count = 0;
            for (auto row = m_map.rbegin(); row != m_map.rend(); row++)
            {
                for (auto val = row->begin(); val != row->end(); val++)
                {
                    if (*val != -1)
                    {
                        count += 1;
                    }
                }
            }
            return count;
        }

        int map_visited_count()
        {
            // Get the count of cells visited (aka the total distance that the robot has traveled)
            int visit_count = 0;
            for (auto &row : m_map)
            {
                for (auto &val : row)
                {
                    if (val != -1)
                    {
                        visit_count += val;
                    }
                    // std::cout << val << " ";
                }
                // std::cout << std::endl;
            }
            // for (auto row = m_map.rbegin(); row != m_map.rend(); row++)
            // {
            //     for (auto val = row->begin(); val != row->end(); val++)
            //     {
            //         if (*val != -1)
            //         {
            //             visit_count += *val;
            //         }
            //     }
            // }
            return visit_count;
        }

        // Check if the robot has returned home (ie full simulation is complete for this run)
        bool is_home()
        {
            return m_state == HOME;
        }

    protected:
        // Common states between both sweep & hybrid robots
        static const uint8_t INIT = 0;
        static const uint8_t SPREAD = 1;
        static const uint8_t DO_PSO = 2;
        static const uint8_t DECIDED = 3;
        static const uint8_t SEND_HOME = 4;
        static const uint8_t GO_HOME = 5;
        static const uint8_t HOME = 6;
        static const uint8_t FOLLOW_PLAN = 7;
        static const uint8_t WAIT = 8;

        std::vector<neighbor_info_t> neighbor_table;
        std::vector<int> neighbor_id_list;

        //----------------------------------------------------------------------
        // MAPS
        //----------------------------------------------------------------------

        void map_samples(std::map<Pos, double> samples)
        {
            // Put the (position, sample) pairs into the map
            // Loop over the samples in the map
            // DO NOT combine with map_coverage and map_visited!!
            for (auto const &s : samples)
            {
                if (s.first.x >= 0 && s.first.x < m_arena_grid_width &&
                    s.first.y >= 0 && s.first.y < m_arena_grid_height)
                {
                    m_map[s.first.x][s.first.y] = s.second;
                }
            }
        }

        void map_coverage(std::map<Pos, double> samples)
        {
            // Label cells that have been *directly observed* (in samples) with 0.
            // Labels cells that have been *visited* with +1 every time they're visited.
            // (Unobserved cells are labeled with -1).
            // VISITED takes priority of OBSERVED in terms of labeling
            for (auto const &s : samples)
            {
                // Label each with 1
                if (s.first.x >= 0 && s.first.x < m_arena_grid_width &&
                    s.first.y >= 0 && s.first.y < m_arena_grid_height &&
                    m_map[s.first.x][s.first.y] == -1)
                {
                    // Only mark if it's withing arena bounds and not already visited
                    m_map[s.first.x][s.first.y] = 0;
                }
            }
        }

        void map_visited(Pos visited_cell)
        {
            // Mark the given cell as visited in the map (+1 to the current value)
            // This will always override a previous label of 0 (observed)
            if (m_map[visited_cell.x][visited_cell.y] < 1)
            {
                // Either unobserved (-1), or observed but not visited (0)
                m_map[visited_cell.x][visited_cell.y] = 1;
            }
            else
            {
                m_map[visited_cell.x][visited_cell.y] += 1;
            }
        }

        int process_msgs()
        {
            // Process all messages since the last tick
            // Add each one to the neighbor table
            // RETURNS: number of neighbors heard from in this tick
            // (this is used for Boids velocity updates)

            std::vector<json> new_msgs = get_msg();

            int curr_tick = get_tick();
            // Clear the list of neighbors heard from
            neighbor_id_list = {};

            for (auto m = 0ul; m < new_msgs.size(); m++)
            {
                json msg = new_msgs[m];
                if (!msg.is_null())
                {
                    // Add the neighbor to the neighbor table
                    add_neighbor(msg);
                    // Add the contents of the message's neighbors to the table
                    // (aka the neighbor's neighbors)
                    for (auto &n : msg.at("neighbors"))
                    {
                        if (n.at("id") != id) // Don't add self
                        {
                            add_neighbor(n, true);
                        }
                    }
                    // Keep a list of the IDs of the neighbors heard from
                    neighbor_id_list.push_back(msg.at("id"));
                }
            } // end msg loop
            return new_msgs.size();
        }

        void add_neighbor(json msg, bool from_table = false)
        {
            // Alternative way to add a neighbor to the table
            // that resizes the table if necessary (for new ids)
            bool was_added = false;
            for (auto i = 0ul; i < neighbor_table.size(); i++)
            {
                if (neighbor_table[i].id == msg.at("id"))
                {
                    // Found an existing neighbor
                    // IF from a received table, use the NEWEST value
                    // Update the value and stop looking
                    if (!from_table || (from_table && msg.at("tick_added") > neighbor_table[i].tick_added))
                    {
                        // Update the value only if...
                        // Not from an rx table
                        // OR the rx table value is newer than the table value
                        add_neighbor(msg, i, from_table);
                    }
                    was_added = true;
                    break;
                }
            }
            if (!was_added)
            {
                // Add to the end of the vector
                // std::cout << "adding new: " << neighbor_table.size() << std::endl;
                add_neighbor(msg, neighbor_table.size(), from_table);
            }
        }

        virtual void add_neighbor(json msg, unsigned long int index, bool from_table = false) = 0;

        // Compute whether all the robots in the neighbor_table have found a min_val below the threshold
        bool all_neighbors_decided()
        {
            if (end_condition == "value")
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
            // irrelevant if using time-based end condition
            return false;
        }

        bool is_time_to_go_home()
        {
            // Check if it's time to head home.
            // This is used only when the end_condition is "time".
            // Checks if there's enough time left to head home before time is up.
            if (end_condition == "time")
            {
                // Get the time left to go home
                int time_left = end_val - get_tick();

                // Otherwise, need to do more checks (ie full path)
                Pos curr_pos = get_pos();
                std::vector<Pos> path_home = create_line(curr_pos.x, curr_pos.y,
                                                         home_pos.x, home_pos.y);
                int path_home_len = path_home.size();

                if (path_home_len >= time_left)
                {
                    // If there's enough time left, go home!
                    return true;
                }
            }
            // We're not doing time-based end condition
            // OR keep going for remaining time
            return false;
        }
    };
}

#endif