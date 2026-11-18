/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

/**
 * \file signalk_client.h
 * A small HTTP client for a Signal K server: reads the pilot's state and
 * sends commands as PUT requests, and asks for an access token.
 *
 * OpenCPN's own Signal K connection only reads, and plugins cannot write
 * through it, so the plugin talks to the server itself. Plain HTTP/1.0 over
 * a socket of its own, one request at a time on a worker thread; the results
 * come back on the main thread. (wxHTTP crashes on a worker thread on macOS,
 * in CFSocketInvalidate.)
 */
#ifndef NAVICO_AP_SIGNALK_CLIENT_H
#define NAVICO_AP_SIGNALK_CLIENT_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <wx/event.h>

class SignalKClient {
public:
  struct Response {
    int status = 0;        ///< HTTP status, 0 if the server could not be reached
    std::string body;      ///< the response body (JSON)
  };
  using Callback = std::function<void(const Response&)>;

  /** Results are delivered through \p owner->CallAfter (main thread). */
  explicit SignalKClient(wxEvtHandler* owner);
  ~SignalKClient();

  void SetServer(const std::string& host, int port, const std::string& token);
  std::string Host() const;
  int Port() const;
  bool HasToken() const;

  /** GET, PUT or POST \p path (from the server root, e.g. "/signalk/v1/api/...").
   *  Urgent requests (commands) go before the others, but stay in the order
   *  they were made: an NFU stop must never overtake the start before it. */
  void Request(const std::string& method, const std::string& path, const std::string& body,
               Callback done, bool urgent = false);
  /** Requests waiting or running. */
  size_t Pending() const;

private:
  struct Job {
    std::string method, path, body;
    Callback done;
    bool urgent = false;
  };
  void Run();
  Response Perform(const Job& job);

  wxEvtHandler* m_owner;
  mutable std::mutex m_mutex;
  std::condition_variable m_cv;
  std::deque<Job> m_jobs;
  size_t m_running = 0;
  bool m_stop = false;
  std::string m_host;
  int m_port = 3000;
  std::string m_token;
  std::shared_ptr<std::atomic<bool>> m_alive;
  std::thread m_thread;
};

#endif  // NAVICO_AP_SIGNALK_CLIENT_H
