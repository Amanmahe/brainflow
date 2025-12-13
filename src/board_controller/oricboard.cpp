#include "oricboard.h"
#include "easywsclient.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")
#include <WinSock2.h>
#endif

OricBoard::OricBoard (struct BrainFlowInputParams params)
    : Board ((int)BoardIds::ORIC_BOARD, params)
{
    is_streaming = false;
    keep_alive = false;
    connected = false;
    session_prepared = false;
    previous_sample_number = -1;
    previous_timestamp = -1;

    // Construct WebSocket URL from params
    if (params.ip_address.empty ())
    {
        ws_url = "ws://oric.local:81";
    }
    else
    {
        ws_url = "ws://" + params.ip_address + ":81";
    }

    safe_logger (spdlog::level::info, "OricBoard created with URL: {}", ws_url);

#ifdef _WIN32
    // Initialize Winsock on Windows
    WSADATA wsaData;
    if (WSAStartup (MAKEWORD (2, 2), &wsaData) != 0)
    {
        safe_logger (spdlog::level::warn, "WSAStartup failed");
    }
#endif
}

OricBoard::~OricBoard ()
{
    skip_logs = true;
    try
    {
        if (is_streaming)
        {
            stop_stream ();
        }
        if (session_prepared)
        {
            release_session ();
        }
    }
    catch (...)
    {
        // SAFETY: do nothing in destructor
    }

#ifdef _WIN32
    WSACleanup ();
#endif
}

int OricBoard::prepare_session ()
{
    if (session_prepared)
    {
        safe_logger (spdlog::level::info, "Session already prepared");
        return (int)BrainFlowExitCodes::STATUS_OK;
    }

    safe_logger (spdlog::level::info, "Connecting to Oric board at {}", ws_url);

    // Create WebSocket connection
    ws = WebSocket::from_url (ws_url);

    if (!ws)
    {
        safe_logger (spdlog::level::err, "Failed to create WebSocket connection to {}", ws_url);
        return (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
    }

    connected = true;
    safe_logger (spdlog::level::info, "WebSocket connection created successfully");

    // Start WebSocket thread
    keep_alive = true;
    ws_thread = std::thread (&OricBoard::ws_thread_func, this);

    // Wait a bit for connection to establish
    std::this_thread::sleep_for (std::chrono::milliseconds (1000));

    if (!connected)
    {
        safe_logger (spdlog::level::err, "WebSocket connection failed after creation");
        keep_alive = false;
        if (ws_thread.joinable ())
        {
            ws_thread.join ();
        }
        return (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
    }

    // Initialize ADS1299
    int res = initialize_ads1299 ();
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        return res;
    }

    session_prepared = true;
    safe_logger (spdlog::level::info, "Oric board session prepared successfully");
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int OricBoard::start_stream (int buffer_size, const char *streamer_params)
{
    if (is_streaming)
    {
        safe_logger (spdlog::level::info, "Streaming already running");
        return (int)BrainFlowExitCodes::STATUS_OK;
    }

    if (!session_prepared)
    {
        safe_logger (spdlog::level::err, "Session not prepared");
        return (int)BrainFlowExitCodes::BOARD_NOT_CREATED_ERROR;
    }

    // Prepare data buffers and streamers
    int res = prepare_for_acquisition (buffer_size, streamer_params);
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        return res;
    }

    // Start data acquisition
    res = send_command ("rdatac");
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        safe_logger (spdlog::level::err, "Failed to start data acquisition");
        return res;
    }

    is_streaming = true;
    safe_logger (spdlog::level::info, "Oric board streaming started");
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int OricBoard::stop_stream ()
{
    if (!is_streaming)
    {
        safe_logger (spdlog::level::info, "Streaming not running");
        return (int)BrainFlowExitCodes::STATUS_OK;
    }

    // Stop data acquisition
    int res = send_command ("sdatac");
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        safe_logger (spdlog::level::warn, "Failed to send sdatac command");
    }

    is_streaming = false;
    safe_logger (spdlog::level::info, "Oric board streaming stopped");
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int OricBoard::release_session ()
{
    if (!session_prepared)
    {
        safe_logger (spdlog::level::info, "Session not prepared");
        return (int)BrainFlowExitCodes::STATUS_OK;
    }

    if (is_streaming)
    {
        stop_stream ();
    }

    // Close WebSocket connection
    keep_alive = false;
    connected = false;

    if (ws)
    {
        ws->close ();
        ws = nullptr; // Set to null pointer
    }

    if (ws_thread.joinable ())
    {
        ws_thread.join ();
    }

    // Free data buffers
    free_packages ();

    session_prepared = false;
    safe_logger (spdlog::level::info, "Oric board session released");
    return (int)BrainFlowExitCodes::STATUS_OK;
}

int OricBoard::config_board (std::string config, std::string &response)
{
    if (!session_prepared)
    {
        return (int)BrainFlowExitCodes::BOARD_NOT_CREATED_ERROR;
    }

    // Implement configuration commands
    if (config == "status")
    {
        int res = send_command ("status");
        response = (res == (int)BrainFlowExitCodes::STATUS_OK) ? "Status command sent" :
                                                                 "Failed to send status";
        return res;
    }
    else if (config.find ("wreg:") == 0)
    {
        // Parse register write command: "wreg:0x01,0x95"
        try
        {
            std::string reg_part = config.substr (5);
            size_t comma_pos = reg_part.find (',');
            if (comma_pos != std::string::npos)
            {
                int reg = std::stoi (reg_part.substr (0, comma_pos), nullptr, 16);
                int value = std::stoi (reg_part.substr (comma_pos + 1), nullptr, 16);
                int res = send_command ("wreg", {reg, value});
                response = (res == (int)BrainFlowExitCodes::STATUS_OK) ?
                    "Register write command sent" :
                    "Failed to write register";
                return res;
            }
        }
        catch (const std::exception &)
        {
            safe_logger (spdlog::level::err, "Invalid wreg command format: {}", config);
            response = "Invalid command format";
            return (int)BrainFlowExitCodes::INVALID_ARGUMENTS_ERROR;
        }
    }
    else if (config == "sdatac")
    {
        int res = send_command ("sdatac");
        response = (res == (int)BrainFlowExitCodes::STATUS_OK) ? "SDATAC command sent" :
                                                                 "Failed to send SDATAC";
        return res;
    }
    else if (config == "rdatac")
    {
        int res = send_command ("rdatac");
        response = (res == (int)BrainFlowExitCodes::STATUS_OK) ? "RDATAC command sent" :
                                                                 "Failed to send RDATAC";
        return res;
    }

    response = "Unsupported command";
    return (int)BrainFlowExitCodes::UNSUPPORTED_BOARD_ERROR;
}

int OricBoard::send_command (const std::string &command, const std::vector<int> &parameters)
{
    if (!connected || !ws)
    {
        safe_logger (spdlog::level::err, "Not connected to Oric board");
        return (int)BrainFlowExitCodes::BOARD_NOT_READY_ERROR;
    }

    try
    {
        nlohmann::json json_cmd;
        json_cmd["command"] = command;
        json_cmd["parameters"] = parameters;

        std::string message = json_cmd.dump ();
        ws->send (message);

        safe_logger (spdlog::level::debug, "Sent command: {}", command);
        return (int)BrainFlowExitCodes::STATUS_OK;
    }
    catch (const std::exception &e)
    {
        safe_logger (spdlog::level::err, "Exception sending command: {}", e.what ());
        return (int)BrainFlowExitCodes::GENERAL_ERROR;
    }
}

int OricBoard::initialize_ads1299 ()
{
    // Send initialization commands matching your Python code
    std::vector<std::pair<int, int>> init_commands = {
        {0x01, 0x95}, // CONFIG1
        {0x02, 0xC0}, // CONFIG2
        {0x03, 0xEC}, // CONFIG3
        {0x05, 0x68}, // CH1SET
        {0x06, 0x68}, // CH2SET
        {0x07, 0x68}, // CH3SET
        {0x08, 0x68}, // CH4SET
        {0x09, 0x68}, // CH5SET
        {0x0A, 0x68}, // CH6SET
        {0x0B, 0x68}, // CH7SET
        {0x0C, 0x68}  // CH8SET
    };

    // Stop data acquisition first
    int res = send_command ("sdatac");
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        return res;
    }

    // Send register configuration
    for (const auto &cmd : init_commands)
    {
        res = send_command ("wreg", {cmd.first, cmd.second});
        if (res != (int)BrainFlowExitCodes::STATUS_OK)
        {
            return res;
        }
        std::this_thread::sleep_for (std::chrono::milliseconds (10));
    }

    // Check status
    res = send_command ("status");
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        return res;
    }

    safe_logger (spdlog::level::info, "ADS1299 initialized successfully");
    return (int)BrainFlowExitCodes::STATUS_OK;
}

void OricBoard::process_data (const std::vector<uint8_t> &data)
{
    // Extract accelerometer data from the end of the packet
    std::vector<uint8_t> accel_data (data.end () - ACCEL_DATA_SIZE, data.end ());
    int16_t accel_x = static_cast<int16_t> ((accel_data[1] << 8) | accel_data[0]);
    int16_t accel_y = static_cast<int16_t> ((accel_data[3] << 8) | accel_data[2]);
    int16_t accel_z = static_cast<int16_t> ((accel_data[5] << 8) | accel_data[4]);

    // Process EEG data part
    std::vector<uint8_t> eeg_data (data.begin (), data.end () - ACCEL_DATA_SIZE);

    for (size_t block_location = 0; block_location < eeg_data.size (); block_location += BLOCK_SIZE)
    {
        if (block_location + BLOCK_SIZE > eeg_data.size ())
            break;

        std::vector<uint8_t> block (
            eeg_data.begin () + block_location, eeg_data.begin () + block_location + BLOCK_SIZE);

        // Extract timestamp and sample number
        uint32_t timestamp = (block[3] << 24) | (block[2] << 16) | (block[1] << 8) | block[0];
        uint32_t sample_number = (block[7] << 24) | (block[6] << 16) | (block[5] << 8) | block[4];

        // Extract channel data (8 channels)
        double package[13] = {0.0}; // 8 EEG + 3 accelerometer
        package[11] = static_cast<double> (sample_number);
        package[12] = static_cast<double> (timestamp);

        for (int channel = 0; channel < 8; channel++)
        {
            int channel_offset = 8 + (channel * 3);
            int32_t sample = (block[channel_offset] << 16) | (block[channel_offset + 1] << 8) |
                block[channel_offset + 2];

            // Convert to signed 24-bit
            if (sample & 0x800000)
            {
                sample |= 0xFF000000;
            }

            package[channel] = static_cast<double> (sample);
        }

        // Add accelerometer data to the package
        package[8] = static_cast<double> (accel_x);
        package[9] = static_cast<double> (accel_y);
        package[10] = static_cast<double> (accel_z);

        // Data validation (similar to your Python code)
        if (previous_sample_number != -1)
        {
            if (sample_number - previous_sample_number > 1)
            {
                safe_logger (spdlog::level::warn, "Sample lost: previous={}, current={}",
                    previous_sample_number, sample_number);
            }
            else if (sample_number == previous_sample_number)
            {
                safe_logger (spdlog::level::warn, "Duplicate sample: {}", sample_number);
            }
            else if (sample_number < static_cast<uint32_t>(previous_sample_number))
            {
                safe_logger (spdlog::level::warn, "Sample order missed: previous={}, current={}",
                    previous_sample_number, sample_number);
            }
        }

        previous_sample_number = sample_number;
        previous_timestamp = timestamp;

        // Check for blank data
        bool is_blank = true;
        for (size_t i = 0; i < 3; ++i)
        {
            if (package[i] != 0.0)
            {
                is_blank = false;
                break;
            }
        }

        if (is_blank)
        {
            for (size_t i = 4; i < 8; ++i)
            {
                if (package[i] <= 0.0)
                {
                    is_blank = false;
                    break;
                }
            }
        }

        if (is_blank)
        {
            safe_logger (spdlog::level::warn, "Blank data detected at sample {}", sample_number);
        }
        else
        {
            // Push package to BrainFlow data buffer
            push_package (package);
        }
    }
}

void OricBoard::ws_thread_func ()
{
    while (keep_alive && ws)
    {
        try
        {
            // Poll for messages (non-blocking)
            ws->poll ();

            // Receive messages
            ws->dispatch (
                [this] (const std::string &message)
                {
                    if (!is_streaming)
                        return;

                    // Convert string to byte array
                    std::vector<uint8_t> data (message.begin (), message.end ());

                    if (data.size () == OricBoard::EXPECTED_PACKET_SIZE)
                    {
                        process_data (data);
                    }
                    else if (data.size () > 0)
                    {
                        safe_logger (spdlog::level::debug,
                            "Received packet size: {} (expected: {})", data.size (),
                            OricBoard::EXPECTED_PACKET_SIZE);
                    }
                });

            std::this_thread::sleep_for (std::chrono::milliseconds (10));
        }
        catch (const std::exception &e)
        {
            safe_logger (spdlog::level::err, "Error in WebSocket thread: {}", e.what ());
            break;
        }
    }

    safe_logger (spdlog::level::info, "WebSocket thread exiting");
}