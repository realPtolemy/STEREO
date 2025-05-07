#include<event.hpp>
#include<shared_state.hpp>
/**
 * Reads line that has the structure of an event, then parses it into an Event object.
 *
 * @param line The line to parse.
 * @return The parsed Event object.
 */
// Event parse_line(const std::string& line);
void parse_lines(const std::string& lines, EventQueueGrouped<SyncedEvents> &event_queue, SyncedEvents &grouped_events);
Event parse_line(const std::string& line);
/**
 * A thread that reads events from a file, parses them, and pushes them into a queue.
 *
 * @param event_file_path The path to the event file.
 * @param camera_events A vector to store the parsed events.
 * @param event_queue A queue that stores batched events.
 */
void camera_thread_csv_new(const std::string &event_file_path, EventQueueGrouped<SyncedEvents> &event_queue);
void camera_thread_csv(const std::string &event_file_path, std::vector<Event> &camera_events, EventQueue<Event> &event_queue);
void camera_thread_udp(UDP& udp, std::vector<Event> &camera_events, EventQueue<Event> &event_queue);
void parse_bufferd_data(const uint16_t* buffered_data, ssize_t length, std::vector<Event> &camera_events);