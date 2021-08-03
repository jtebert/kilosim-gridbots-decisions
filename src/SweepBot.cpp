/*
 * Gridbots finding a target location with a sweep
 *
 * Created 2021-08 by Julia Ebert
 */

#include "Gridbot.h"

namespace Kilosim
{
    typedef struct lil_neighbor_info_t
    {
        // one entry (row) in a table of observations/messages from neighbor robots
        uint16_t id;
        uint min_val;
        uint min_loc_x;
        uint min_loc_y;
        uint curr_pos_x;
        uint curr_pos_y;
        int tick_added;
        bool from_neighbor;
    } lil_neighbor_info_t;

    class SweepBot : public Gridbot
    {
    public:
        uint8_t m_state;
        // Longest path by any of the robots
        int max_path_len;

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

        // Check if the robot has returned home (ie full simulation is complete for this run)
        bool is_home()
        {
            return m_state == HOME;
        }

        void set_path_external(std::vector<Pos> path)
        {
            // This is an alternative to creating a path with Gridbot.set_path()
            m_path_to_target.resize(path.size());
            m_path_to_target = path;
            m_follow_path = true;
            // When a path is set, turn off other movement
            move(0, 0);
        }

    private:
        // States
        static const uint8_t FOLLOW_PLAN = 0;
        static const uint8_t WAIT = 1;
        static const uint8_t SEND_HOME = 2;
        static const uint8_t GO_HOME = 3;
        static const uint8_t HOME = 4;

        std::vector<lil_neighbor_info_t> neighbor_table;

        void setup()
        {
            set_led(100, 0, 0);
            m_state = FOLLOW_PLAN;
        }

        void loop()
        {
            if (m_state == FOLLOW_PLAN)
            {
                // Move to the next location in the path
                // Observe here
                std::map<Pos, double> pos_samples = sample_around();
                // map_samples(pos_samples);
                update_mins(pos_samples);
                // This state is only used when end_val != time, so don't need to check is_time_to_go_home()
                if (is_time_to_go_home() ||
                    (get_tick() > 5 && all_neighbors_decided()))
                {
                    m_state = SEND_HOME;
                }
                else if (m_path_to_target.size() == 0)
                {
                    // Hit the end of the path; wait
                    m_state = WAIT;
                }
                // else it will automatically follow the path (I think)
            }
            else if (m_state == WAIT)
            {
                // Wait for all robots to finish their route
                if (get_tick() >= max_path_len || is_time_to_go_home() || all_neighbors_decided())
                {
                    m_state = SEND_HOME;
                }
            }
            else if (m_state == SEND_HOME)
            {
                m_state = GO_HOME;
                set_path(get_pos().x, get_pos().y, home_pos.y, home_pos.y);
            }
            else if (m_state == GO_HOME)
            {
                // Move back to the origin/collection point
                if (get_pos() == home_pos)
                {
                    m_state = HOME;
                }
            }
            else if (m_state == HOME)
            {
                // You're done
            }

            if (min_val <= end_val)
            {
                set_led(0, 100, 0);
            }

            // Send message
            update_send_msg();
            process_msgs();
        }

        void update_mins(std::map<Pos, double> pos_samples)
        {
            // Copied from HybridBot on 2021-08-03
            // Update the minimum location and value, if any of the new values are lower
            for (auto const &s : pos_samples)
            {
                if (s.second < min_val && s.second != -1) // (-1=not real obs.)
                {
                    // Set as the minimum if it's lower than anything seen before
                    min_val = s.second;
                    min_loc = s.first;
                }
            }
        }
        std::map<Pos, std::vector<double>> process_msgs()
        {
            // Copied from HybridBot on 2021-08-03
            // Process all messages since the last tick
            // Add each one to the neighbor table
            // RETURNS: map<Pos, Pos> of {position, velocity} of all neighbors heard from in this tick
            // (this is used for Boids velocity updates)

            std::vector<json> new_msgs = get_msg();

            int curr_tick = get_tick();
            std::map<Pos, std::vector<double>> neighbor_pos_vel;

            for (auto m = 0; m < new_msgs.size(); m++)
            {
                json msg = new_msgs[m];
                if (!msg.is_null())
                {
                    // Put into return map (position + velocity)
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
                }
            } // end msg loop
            return neighbor_pos_vel;
        }

        void add_neighbor(json msg, bool from_table = false)
        {
            // Copied from HybridBot on 2021-08-03
            // Alternative way to add a neighbor to the table
            // that resizes the table if necessary (for new ids)
            bool was_added = false;
            for (auto i = 0; i < neighbor_table.size(); i++)
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

        void add_neighbor(json msg, int ind, bool from_table = false)
        {
            // Copied from HybridBot on 2021-08-03
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
            // neighbor_table[ind].curr_vel_x = msg.at("curr_vel_x");
            // neighbor_table[ind].curr_vel_y = msg.at("curr_vel_y");

            if (from_table)
            {
                neighbor_table[ind].tick_added = msg.at("tick_added");
                // neighbor_table[ind].from_neighbor = true;
            }
            else
            {
                neighbor_table[ind].tick_added = get_tick();
                // neighbor_table[ind].from_neighbor = false;
            }
            // Check if the new message is a new min value
            if (msg.at("min_val") < min_val)
            {
                min_val = msg.at("min_val");
                min_loc = {msg.at("min_loc_x"), msg.at("min_loc_y")};
                // rx_min_val = msg.at("min_val");
                // rx_min_loc = {msg.at("min_loc_x"), msg.at("min_loc_y")};
            }
        }

        void update_send_msg()
        {
            // Copied from HybridBot on 2021-08-03
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
                // msg["curr_vel_x"] = velocity[0];
                // msg["curr_vel_y"] = velocity[1];
                msg["neighbors"] = neighbors_to_json();
                send_msg(msg);
            }
        }

        json neighbors_to_json()
        {
            // Copied from HybridBot on 2021-08-03
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
                    // neighbor["curr_vel_x"] = neighbor_table[i].curr_vel_x;
                    // neighbor["curr_vel_y"] = neighbor_table[i].curr_vel_y;
                    neighbors.push_back(neighbor);
                }
            }
            return neighbors;
        }

        bool all_neighbors_decided()
        {
            // Copied from HybridBot on 2021-08-03
            // Compute whether all the robots in the neighbor_table have found a min_val below the threshold
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
            // Copied from HybridBot on 2021-08-03
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