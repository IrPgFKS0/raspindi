/* Adapted from rpicam_vid.cpp - libcamera video record app for NDI output. */
#include <chrono>
#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>
#include <iostream>
#include <cstring> // for memset
#include <sys/socket.h>
#include <netinet/in.h> // for sockaddr_in
#include <unistd.h> // for close
#include <thread> // for std::thread
#include <queue>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "core/rpicam_encoder.hpp"
#include "output/output.hpp"
#include "ndi_output.hpp"
#include <libconfig.h++>

#define PORT 4511  // This is the number produced when converting "raspindi" to l33t (i.e. r45p1nd1).

using namespace std::placeholders;
libconfig::Config cfg;
        
template<typename T>
class ThreadSafeQueue {
private:
    std::queue<T> queue;
    std::mutex mutex;
    std::condition_variable cond;

public:
    void push(T value) {
        std::lock_guard<std::mutex> lock(mutex);
        queue.push(value);
        cond.notify_one();
    }

    bool try_pop(T& value) {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.empty()) {
            return false;
        }
        value = queue.front();
        queue.pop();
        return true;
    }

    void wait_and_pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex);
        cond.wait(lock, [this] { return !queue.empty(); });
        value = queue.front();
        queue.pop();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex);
        return queue.empty();
    }
};

std::vector<std::thread> client_threads;
std::mutex client_threads_mutex;
// Function to join all client threads
void join_all_client_threads() {
    std::lock_guard<std::mutex> lock(client_threads_mutex);
    for (auto& t : client_threads) {
        if (t.joinable()) {
            t.join(); // Wait for the thread to finish
        }
    }
    client_threads.clear(); // Clear the list after joining all threads
}

// Some keypress/signal handling.

std::atomic<int> signal_received(0);
static void default_signal_handler(int signal_number) {
    signal_received.store(signal_number);
    join_all_client_threads();
    LOG(1, "Received signal " << signal_number);
}

static int get_key_or_signal(VideoOptions const *options, pollfd p[1])
{
    int key = 0;
    if (signal_received.load() == SIGINT)
        return 'x';
    if (options->keypress)
    {
        poll(p, 1, 0);
        if (p[0].revents & POLLIN)
        {
            char *user_string = nullptr;
            size_t len;
            [[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
            key = user_string[0];
        }
    }
    if (options->signal)
    {
        if (signal_received.load() == SIGUSR1)
            key = '\n';
        else if ((signal_received.load() == SIGUSR2) || (signal_received.load() == SIGPIPE))
            key = 'x';
        signal_received = 0;
    }
    return key;
}

int loadConfig()
{
    try
    {
        cfg.readFile("/etc/raspindi.conf");
    }
    catch(const libconfig::FileIOException &fioex)
    {
        std::cerr << "Could not open config file /etc/raspindi.conf"
            << std::endl;
        return(EXIT_FAILURE);
    }
    catch(const libconfig::ParseException &pex)
    {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
              << " - " << pex.getError() << std::endl;
        return(EXIT_FAILURE);
    }
    return 0;
}

std::string _getValue(std::string parameter, std::string defaultValue)
{
//    std::cout << "Lookup Parameter: " << parameter << std::endl;
    try
    {
        std::string value = cfg.lookup(parameter).c_str();
//        std::cout << "Value: " << value << std::endl;
        return value;
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
//        std::cerr << "Setting Not Found Returning Default: " << defaultValue << std::endl;
        return defaultValue;
    }
}

static int get_colourspace_flags(std::string const &codec)
{
    if (codec == "mjpeg" || codec == "yuv420")
        return RPiCamEncoder::FLAG_VIDEO_JPEG_COLOURSPACE;
    else
        return RPiCamEncoder::FLAG_VIDEO_NONE;
}

// The main even loop for the application.
static void event_loop(RPiCamEncoder &app, ThreadSafeQueue<std::string>& commandQueue)
{
    VideoOptions const *options = app.GetOptions();
    std::unique_ptr<Output> output;
    if (options->output.empty()) {
        // Used to write output to NDI stream
        output = std::unique_ptr<Output>(
            new NdiOutput(options, _getValue("neopixel_path", "/tmp/neopixel.state"), _getValue("ndi_stream_name", "Video Feed"))
        );
    } else {
        // Used to write output to file
        output = std::unique_ptr<Output>(Output::Create(options));
    }

    app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
    app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

    app.OpenCamera();
    app.ConfigureVideo(get_colourspace_flags(options->codec));
    app.StartEncoder();
    app.StartCamera();
    auto start_time = std::chrono::high_resolution_clock::now();

    // Monitoring for keypresses and signals.
    signal(SIGUSR1, default_signal_handler);
    signal(SIGUSR2, default_signal_handler);
    signal(SIGINT, default_signal_handler);
    // SIGPIPE gets raised when trying to write to an already closed socket. This can happen, when
    // you're using TCP to stream to VLC and the user presses the stop button in VLC. Catching the
    // signal to be able to react on it, otherwise the app terminates.
    signal(SIGPIPE, default_signal_handler);
    pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

    for (unsigned int count = 0; ; count++)
    {
        std::string command;
        while (commandQueue.try_pop(command)) {
            std::cout << "Processing command: " << command << std::endl;
            auto delimiterPos = command.find(":");
            std::string commandType = command.substr(0, delimiterPos);
            std::string commandValue = command.substr(delimiterPos + 1);

            // Dynamically adjust parameter based on external input
            libcamera::ControlList controls;
            if (commandType == "SetExposureTime") {
                std::chrono::microseconds duration(std::stol(commandValue));
                if (!controls.get(controls::ExposureTime))
                    controls.set(controls::ExposureTime, duration.count()); // Use count() to pass the number of microseconds
            }
            if (commandType == "SetExposureValue") {
                if (!controls.get(controls::ExposureValue))
                    controls.set(controls::ExposureValue, std::stof(commandValue));
            }
            if (commandType == "SetAnalogueGain") {
                if (!controls.get(controls::AnalogueGain))
                    controls.set(controls::AnalogueGain, std::stod(commandValue));
            }
            if (commandType == "SetBrightness") {
                if (!controls.get(controls::Brightness))
                    controls.set(controls::Brightness, std::stof(commandValue));
            }
            if (commandType == "SetContrast") {
                if (!controls.get(controls::Contrast))
                    controls.set(controls::Contrast, std::stof(commandValue));
            }
            if (commandType == "SetSharpness") {
                if (!controls.get(controls::Sharpness))
                    controls.set(controls::Sharpness, std::stof(commandValue));
            }
            if (commandType == "SetSaturation") {
                if (!controls.get(controls::Saturation))
                    controls.set(controls::Saturation, std::stof(commandValue));
            }
            if (commandType == "SetAwbMode") {
                if (!controls.get(controls::AwbMode))   // Typically 0-7
                    controls.set(controls::AwbMode, std::stoi(commandValue));
            }

            // Apply the updated controls
            app.SetControls(controls);
        }
        RPiCamEncoder::Msg msg = app.Wait();
        if (msg.type == RPiCamApp::MsgType::Timeout)
        {
            LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
            app.StopCamera();
            app.StartCamera();
            continue;
        }
        if (msg.type == RPiCamEncoder::MsgType::Quit)
            return;
        else if (msg.type != RPiCamEncoder::MsgType::RequestComplete)
            throw std::runtime_error("unrecognised message!");
        int key = get_key_or_signal(options, p);
        if (key == '\n')
            output->Signal();

        LOG(2, "Viewfinder frame " << count);
        auto now = std::chrono::high_resolution_clock::now();
        bool timeout = !options->frames && options->timeout &&
                       ((now - start_time) > options->timeout.value);
        bool frameout = options->frames && count >= options->frames;
        if (timeout || frameout || key == 'x' || key == 'X')
        {
            if (timeout)
                LOG(1, "Halting: reached timeout of " << options->timeout.get<std::chrono::milliseconds>()
                                                      << " milliseconds.");
            app.StopCamera(); // stop complains if encoder very slow to close
            app.StopEncoder();
            return;
        }

        CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
        app.EncodeBuffer(completed_request, app.VideoStream());
        app.ShowPreview(completed_request, app.VideoStream());
    }
}

void handle_client(int client_socket, ThreadSafeQueue<std::string>& commandQueue) {
    const char* hello = "Ack\n";
    std::string commandPrefix; // Used to store the command part before ':'
    std::vector<std::string> validCommands = {
        "SetExposureTime",
        "SetExposureValue",
        "SetAnalogueGain",
        "SetBrightness",
        "SetContrast",
        "SetSharpness",
        "SetSaturation",
        "SetAwbMode"
        // Add more valid commands/settings as necessary (also update event_loop())
    };

    while (!signal_received.load()) {
        char buffer[1024] = {0};
        long valread = read(client_socket, buffer, sizeof(buffer));

        if (valread <= 0) {
            std::cout << "Client disconnected or error reading from socket\n";
            break;
        }

        // Convert the buffer to a std::string for easier manipulation
        std::string receivedCommand(buffer, valread);

        // Find the delimiter position in the valid command
        size_t delimiterPos = receivedCommand.find(':');
        if (delimiterPos != std::string::npos) {
            // Strip the newline character from the end of the string if present
            receivedCommand.erase(std::remove(receivedCommand.begin(), receivedCommand.end(), '\n'), receivedCommand.end());
            receivedCommand.erase(std::remove(receivedCommand.begin(), receivedCommand.end(), '\r'), receivedCommand.end());

            // Extract the command part (prefix) from the valid command
            commandPrefix = receivedCommand.substr(0, delimiterPos);

            // Check if the received command is in the list of valid commands
            if (std::find(validCommands.begin(), validCommands.end(), commandPrefix) != validCommands.end()) {
                // Push received command into the queue
                std::string command(buffer, valread);
                commandQueue.push(receivedCommand);

                // Process the valid command here
                std::cout << "Received valid command: " << receivedCommand << std::endl;

                // Send an acknowledgment back to the client
                write(client_socket, hello, strlen(hello));
            }
        } else {
            // Handle invalid command
            std::cout << "Received invalid command: " << receivedCommand << std::endl;
            const char* errorMsg = "Error: Invalid command\n";
            write(client_socket, errorMsg, strlen(errorMsg));
        }
    }

    close(client_socket);
    std::cout << "Connection closed\n";
}

void accept_new_clients(int server_fd, ThreadSafeQueue<std::string>& commandQueue)
{
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    while (!signal_received.load()) {
        std::cout << "\n+++++++ Waiting for new connection ++++++++\n\n";
        int new_socket;
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept");
            continue; // Continue to the next iteration instead of exiting
        }

        // Lock the mutex to safely add the new thread to the list
        std::lock_guard<std::mutex> lock(client_threads_mutex);
        client_threads.emplace_back(handle_client, new_socket, std::ref(commandQueue));
    }
}


int main(int argc, char *argv[])
{
    ThreadSafeQueue<std::string> commandQueue;

    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY; // Listen on any IP address
    address.sin_port = htons(PORT);

    memset(address.sin_zero, '\0', sizeof address.sin_zero);

    // Reuse ports & addr when restarting socket listener
    int yes = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1) {
        perror("setsockopt SO_REUSEADDR failed");
        exit(EXIT_FAILURE);
    }

    // Set SO_REUSEPORT to allow multiple sockets to bind to the same port
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes)) == -1) {
        perror("setsockopt SO_REUSEPORT failed");
        exit(EXIT_FAILURE);
    }
    // Bind the socket to the IP address and port
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Start listening for connections
    if (listen(server_fd, 10) < 0) { // Listen on the socket, with max 10 pending connections
        perror("listen");
        exit(EXIT_FAILURE);
    }

    // Start the thread for accepting new clients
    std::thread serverThread(accept_new_clients, server_fd, std::ref(commandQueue));

    try
    {
        RPiCamEncoder app;
        VideoOptions *options = app.GetOptions();
        loadConfig();
        if (options->Parse(argc, argv))
        {
            if (options->verbose >= 2)
                options->Print();

            std::thread videoThread(event_loop, std::ref(app), std::ref(commandQueue));
            videoThread.join();
        }
    }
    catch (std::exception const &e)
    {
        LOG_ERROR("ERROR: *** " << e.what() << " ***");
        return -1;
    }
    return 0;
}
