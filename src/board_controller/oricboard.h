#pragma once

#include "board.h"
#include "easywsclient.hpp"
#include <atomic>
#include <mutex>
#include <nlohmann/json.hpp>
#include <queue>
#include <thread>
#include <vector>

using easywsclient::WebSocket;

class OricBoard : public Board
{
public:
    OricBoard (struct BrainFlowInputParams params);
    virtual ~OricBoard ();

    int prepare_session ();
    int start_stream (int buffer_size, const char *streamer_params);
    int stop_stream ();
    int release_session ();
    int config_board (std::string config, std::string &response);

private:
    // WebSocket connection
    WebSocket::pointer ws;
    std::string ws_url;
    // Thread management
    std::thread ws_thread;
    std::atomic<bool> is_streaming;
    std::atomic<bool> keep_alive;
    std::atomic<bool> connected;
    std::atomic<bool> session_prepared;
    // Data processing
    enum Constants
    {
        BLOCK_SIZE = 32,
        SAMPLES_PER_BUFFER = 10,
        ACCEL_DATA_SIZE = 6,
        EXPECTED_PACKET_SIZE = (BLOCK_SIZE * SAMPLES_PER_BUFFER) + ACCEL_DATA_SIZE
    };
    int previous_sample_number;
    int previous_timestamp;
    // WebSocket thread function
    void ws_thread_func ();
    // Board commands
    int send_command (const std::string &command, const std::vector<int> &parameters = {});
    int initialize_ads1299 ();
    void process_data (const std::vector<uint8_t> &data);
    std::mutex ws_mutex;
};