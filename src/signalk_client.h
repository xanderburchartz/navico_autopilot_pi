/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

/**
 * \file signalk_client.h
 * A small HTTP client for a Signal K server: reads the pilot's state, sends
 * commands as PUT requests and asks for an access token.
 *
 * OpenCPN's own Signal K connection only reads, and plugins cannot write
 * through it, so the plugin talks to the server itself: plain HTTP/1.0 over
 * a socket, one request at a time, in order, on a worker thread. The results
 * come back on the main thread.
 * Not wxHTTP: it crashes on a worker thread on macOS (CFSocketInvalidate).
 * Not wxWebRequest: the wxWidgets build of OpenCPN for Android does not have it.
 */
#ifndef NAVICO_AP_SIGNALK_CLIENT_H
#define NAVICO_AP_SIGNALK_CLIENT_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
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
    int status = 0;    ///< HTTP status, 0 if the server could not be reached
    std::string body;  ///< the response body (JSON)
  };
  using Callback = std::function<void(const Response&)>;

  /** Results are delivered through \p owner->CallAfter (main thread). */
  explicit SignalKClient(wxEvtHandler* owner);
  /** Breaks off a running request and waits for the worker thread: about a
   *  second per address while connecting, longer while a host name is being
   *  looked up. The thread must be gone before OpenCPN unloads the plugin. */
  ~SignalKClient();

  void SetServer(const std::string& host, int port, const std::string& token);

  /** GET, PUT or POST \p path (from the server root, e.g. "/signalk/v1/api/..."). */
  void Request(const std::string& method, const std::string& path, const std::string& body,
               Callback done);

private:
  struct Job {
    std::string method, path, body;
    Callback done;
  };
  void Run();
  Response Perform(const Job& job, const std::string& host, int port, const std::string& token);

  wxEvtHandler* m_owner;
  std::mutex m_mutex;
  std::condition_variable m_cv;
  std::deque<Job> m_jobs;
  bool m_stop = false;
  std::string m_host;
  int m_port = 3000;
  std::string m_token;
  intptr_t m_socket = -1;  ///< socket of the running request, -1 if none
  /** False once the client is gone: results still queued are dropped. */
  std::shared_ptr<std::atomic<bool>> m_alive;
  std::thread m_thread;
};

#endif  // NAVICO_AP_SIGNALK_CLIENT_H
