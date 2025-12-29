#include "HmiBridge.h"
#include "RdtJson.h" // CORRECTED: Include the renamed file from its new shared location
#include "Logger.h"    // RDT Logger
#include <algorithm>   // For std::remove

using json = nlohmann::json;

namespace Rdt {

HmiBridge::HmiBridge() : listen_socket_(INVALID_SOCKET) {}

HmiBridge::~HmiBridge() {
    stop();
}

bool HmiBridge::start(int port) {
    listen_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_socket_ == INVALID_SOCKET) {
        LOG_CRITICAL(MODULE_NAME, "Failed to create listen socket.");
        return false;
    }

    // Allow socket reuse
    int opt = 1;
    setsockopt(listen_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);

    if (bind(listen_socket_, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        LOG_CRITICAL_F(MODULE_NAME, "Failed to bind to port %d.", port);
        closeSocket(listen_socket_);
        return false;
    }

    if (listen(listen_socket_, SOMAXCONN) == SOCKET_ERROR) {
        LOG_CRITICAL(MODULE_NAME, "Failed to listen on socket.");
        closeSocket(listen_socket_);
        return false;
    }

    running_ = true;
    server_thread_ = std::jthread(&HmiBridge::serverThreadLoop, this);
    LOG_INFO_F(MODULE_NAME, "Server started, listening on port %d.", port);
    return true;
}

void HmiBridge::stop() {
    running_ = false;

    if (listen_socket_ != INVALID_SOCKET) {
        // Shutdown to unblock accept()
        shutdown(listen_socket_, SHUT_RDWR);
        closeSocket(listen_socket_);
        listen_socket_ = INVALID_SOCKET;
    }
    
    // Disconnect all clients
    std::lock_guard<std::mutex> lock(client_mutex_);
    for (SOCKET client : clients_) {
        shutdown(client, SHUT_RDWR);
        closeSocket(client);
    }
    clients_.clear();

    // The jthread will be automatically joined by its destructor
    LOG_INFO(MODULE_NAME, "Server stopped.");
}

void HmiBridge::serverThreadLoop() {
    while (running_) {
        sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);
        SOCKET clientSocket = accept(listen_socket_, (sockaddr*)&clientAddr, &clientLen);

        if (clientSocket == INVALID_SOCKET) {
            if (running_) {
                LOG_ERROR(MODULE_NAME, "accept() failed.");
            }
            continue;
        }

        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &clientAddr.sin_addr, client_ip, INET_ADDRSTRLEN);
        LOG_INFO_F(MODULE_NAME, "Client connected from %s.", client_ip);

        {
            std::lock_guard<std::mutex> lock(client_mutex_);
            clients_.push_back(clientSocket);
        }

        // Detach a thread to handle this client's reads
        std::thread(&HmiBridge::handleClient, this, clientSocket).detach();
    }
}

void HmiBridge::handleClient(SOCKET clientSocket) {
    char buffer[16384];
    std::string receive_buffer;

    while (running_) {
        int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
        if (bytesReceived <= 0) {
            // Connection closed or error
            break;
        }

        receive_buffer.append(buffer, bytesReceived);

        size_t pos;
        while ((pos = receive_buffer.find('\n')) != std::string::npos) {
            std::string line = receive_buffer.substr(0, pos);
            receive_buffer.erase(0, pos + 1);
            
            try {
                auto j = json::parse(line);
                processIncomingJson(j);
            } catch (const json::parse_error& e) {
                LOG_WARN_F(MODULE_NAME, "JSON parse error: %s", e.what());
            }
        }
    }
    
    LOG_INFO(MODULE_NAME, "Client disconnected.");
    removeClient(clientSocket);
}

void HmiBridge::removeClient(SOCKET sock) {
    std::lock_guard<std::mutex> lock(client_mutex_);
    auto it = std::find(clients_.begin(), clients_.end(), sock);
    if (it != clients_.end()) {
        closeSocket(*it);
        clients_.erase(it);
    }
}

void HmiBridge::closeSocket(SOCKET sock) {
    if (sock != INVALID_SOCKET) {
        close(sock);
    }
}

ControlState HmiBridge::poll() {
    ControlState latest_state; // Default-constructed
    bool has_new_state = false;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!incoming_queue_.empty()) {
            // Only process the very latest state, discard older ones to prevent lag
            latest_state = std::move(incoming_queue_.back());
            incoming_queue_.clear();
            has_new_state = true;
        }
    }
    
    if (has_new_state && control_handler_) {
        control_handler_(latest_state);
    }

    return latest_state;
}

void HmiBridge::processIncomingJson(const json& j) {
    if (!j.contains("type")) return;

    PacketType type = static_cast<PacketType>(j["type"].get<int>());

    if (type == PacketType::CONTROL_STATE && j.contains("payload")) {
        try {
            ControlState state = j["payload"].get<ControlState>();
            std::lock_guard<std::mutex> lock(queue_mutex_);
            incoming_queue_.push_back(std::move(state));
        } catch (const json::exception& e) {
            LOG_ERROR_F(MODULE_NAME, "JSON deserialization to ControlState failed: %s", e.what());
        }
    }
    // Handle other types like RESTART_CONTROLLER_REQ if needed
}

void HmiBridge::broadcastStatus(const RobotStatus& status) {
    if (!running_) return;

    std::vector<SOCKET> clients_copy;
    {
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (clients_.empty()) return;
        clients_copy = clients_;
    }

    json j;
    j["type"] = static_cast<int>(PacketType::STATUS_UPDATE);
    j["payload"] = status;
    std::string data = j.dump();
    data += "\n";

    for (SOCKET s : clients_copy) {
        if (send(s, data.c_str(), data.size(), MSG_NOSIGNAL) == -1) {
            // Error sending, likely client disconnected. The read thread will handle cleanup.
        }
    }
}

void HmiBridge::setControlStateHandler(std::function<void(const Rdt::ControlState&)> handler) {
    control_handler_ = std::move(handler);
}

} // namespace Rdt