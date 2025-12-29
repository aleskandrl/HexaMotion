/**
 * @file HmiBridge.h
 * @brief TCP Network Interface for the HexaMotion Controller.
 * @author HexaKinetica Team
 * @version 1.0
 *
 * This class handles raw TCP socket communication for the headless controller.
 * It uses a dedicated thread for listening and receiving data to avoid blocking
 * the main real-time control loop.
 */

#ifndef HMIBRIDGE_H
#define HMIBRIDGE_H

// *** FIX: Use direct include path, not relative ***
#include "RdtProtocol.h" 
#include "nlohmann/json.hpp"
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>

// Linux socket headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <poll.h>

// Type alias for cross-platform compatibility (though we target Linux)
using SOCKET = int;
constexpr SOCKET INVALID_SOCKET = -1;
constexpr int SOCKET_ERROR = -1;

namespace Rdt {

/**
 * @class HmiBridge
 * @brief Manages network communication with the HexaStudio HMI.
 */
class HmiBridge {
public:
    HmiBridge();
    ~HmiBridge();

    HmiBridge(const HmiBridge&) = delete;
    HmiBridge& operator=(const HmiBridge&) = delete;

    /**
     * @brief Starts the TCP server on a dedicated thread.
     * @param port Port to listen on.
     * @return True if successful.
     */
    bool start(int port);

    /**
     * @brief Stops the server, disconnects clients, and joins the server thread.
     */
    void stop();

    /**
     * @brief Polls for and processes all queued incoming messages.
     * @return The latest ControlState received from the HMI.
     */
    [[nodiscard]] ControlState poll();


    /**
     * @brief Sends the robot status to all connected clients.
     * @param status The status structure to serialize and broadcast.
     */
    void broadcastStatus(const RobotStatus& status);

    /**
     * @brief Register a handler for control state updates.
     */
    void setControlStateHandler(std::function<void(const ControlState&)> handler);


private:
    void serverThreadLoop();
    void handleClient(SOCKET clientSocket);
    void closeSocket(SOCKET sock);
    void removeClient(SOCKET sock);

    void processIncomingJson(const nlohmann::json& j);

    SOCKET listen_socket_;
    std::atomic<bool> running_{false};
    std::jthread server_thread_;

    std::vector<ControlState> incoming_queue_;
    std::mutex queue_mutex_;
    
    std::function<void(const ControlState&)> control_handler_;

    std::vector<SOCKET> clients_;
    std::mutex client_mutex_;

    static inline const std::string MODULE_NAME = "HmiBridge";
};

} // namespace Rdt

#endif // HMIBRIDGE_H