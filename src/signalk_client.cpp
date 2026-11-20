/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

// On Windows winsock2.h must come before windows.h, which wx pulls in.
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include "signalk_client.h"

#include <vector>

#include <cerrno>
#include <cstdlib>
#include <cstring>

#ifdef _WIN32
using SocketHandle = SOCKET;
static void CloseSocket(SocketHandle s) { closesocket(s); }
static const SocketHandle kNoSocket = INVALID_SOCKET;
static const int kShutBoth = SD_BOTH;
#else
#include <fcntl.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
using SocketHandle = int;
static void CloseSocket(SocketHandle s) { close(s); }
static const SocketHandle kNoSocket = -1;
static const int kShutBoth = SHUT_RDWR;
#endif

namespace {

/** Connect within \p seconds; sending and each receive then wait at most
 *  \p io_seconds. kNoSocket if connecting failed. */
SocketHandle Connect(const std::string& host, int port, int seconds, int io_seconds) {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* res = nullptr;
  if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0 || !res)
    return kNoSocket;
  SocketHandle s = kNoSocket;
  for (addrinfo* ai = res; ai; ai = ai->ai_next) {
    s = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (s == kNoSocket) continue;
#ifdef _WIN32
    u_long nb = 1;
    ioctlsocket(s, FIONBIO, &nb);
#else
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif
    int r = connect(s, ai->ai_addr, static_cast<int>(ai->ai_addrlen));
    bool ok = r == 0;
    if (!ok) {
      // Windows reports a failed connect in the except set, others in the write set.
      fd_set w, e;
      FD_ZERO(&w);
      FD_SET(s, &w);
      FD_ZERO(&e);
      FD_SET(s, &e);
      timeval tv{seconds, 0};
      if (select(static_cast<int>(s) + 1, nullptr, &w, &e, &tv) > 0 && FD_ISSET(s, &w)) {
        int err = 0;
        socklen_t len = sizeof err;
        getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&err), &len);
        ok = err == 0;
      }
    }
    if (ok) {
#ifdef _WIN32
      nb = 0;
      ioctlsocket(s, FIONBIO, &nb);
      DWORD ms = io_seconds * 1000;
      setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&ms), sizeof ms);
      setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&ms), sizeof ms);
#else
      fcntl(s, F_SETFL, flags);
      timeval tv{io_seconds, 0};
      setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
      setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
#endif
      break;
    }
    CloseSocket(s);
    s = kNoSocket;
  }
  freeaddrinfo(res);
  return s;
}

bool SendAll(SocketHandle s, const std::string& data) {
  size_t sent = 0;
  while (sent < data.size()) {
#ifdef MSG_NOSIGNAL
    int flags = MSG_NOSIGNAL;
#else
    int flags = 0;
#endif
    auto n = send(s, data.data() + sent, static_cast<int>(data.size() - sent), flags);
    if (n <= 0) return false;
    sent += static_cast<size_t>(n);
  }
  return true;
}

}  // namespace

SignalKClient::SignalKClient(wxEvtHandler* owner)
    : m_owner(owner), m_alive(std::make_shared<std::atomic<bool>>(true)) {
  m_thread = std::thread([this] { Run(); });
}

SignalKClient::~SignalKClient() {
  m_alive->store(false);
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stop = true;
    m_jobs.clear();
    // Break off a request waiting for the server.
    if (m_socket != -1) shutdown(static_cast<SocketHandle>(m_socket), kShutBoth);
  }
  m_cv.notify_all();
  if (m_thread.joinable()) m_thread.join();
}

void SignalKClient::SetServer(const std::string& host, int port, const std::string& token) {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_host = host;
  m_port = port;
  m_token = token;
}

void SignalKClient::Request(const std::string& method, const std::string& path,
                            const std::string& body, Callback done) {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_jobs.push_back({method, path, body, std::move(done)});
  m_cv.notify_one();
}

void SignalKClient::RequestFirst(const std::string& method, const std::string& path,
                                 const std::string& body, Callback done) {
  std::vector<Callback> dropped;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto it = m_jobs.begin(); it != m_jobs.end();) {
      if (it->path == path) {
        if (it->done) dropped.push_back(std::move(it->done));
        it = m_jobs.erase(it);
      } else {
        ++it;
      }
    }
    m_jobs.push_front({method, path, body, std::move(done)});
  }
  m_cv.notify_one();
  // Callers count their pending requests: tell them about the dropped ones.
  for (auto& cb : dropped) {
    auto alive = m_alive;
    m_owner->CallAfter([alive, cb] {
      Response r;
      r.status = kDropped;
      if (alive->load()) cb(r);
    });
  }
}

void SignalKClient::Run() {
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
  for (;;) {
    Job job;
    std::string host, token;
    int port;
    {
      std::unique_lock<std::mutex> lock(m_mutex);
      m_cv.wait(lock, [this] { return m_stop || !m_jobs.empty(); });
      if (m_stop) break;
      job = std::move(m_jobs.front());
      m_jobs.pop_front();
      host = m_host;
      port = m_port;
      token = m_token;
    }
    Response r = Perform(job, host, port, token);
    if (job.done && m_alive->load()) {
      auto alive = m_alive;
      Callback done = job.done;
      m_owner->CallAfter([alive, done, r] {
        if (alive->load()) done(r);
      });
    }
  }
#ifdef _WIN32
  WSACleanup();
#endif
}

SignalKClient::Response SignalKClient::Perform(const Job& job, const std::string& host,
                                               int port, const std::string& token) {
  Response r;
  if (host.empty()) return r;
  // A PUT may take the server a while (it waits for the bus); a poll should not.
  SocketHandle s = Connect(host, port, 1, job.method == "GET" ? 1 : 3);
  if (s == kNoSocket) return r;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_stop) {
      CloseSocket(s);
      return r;
    }
    m_socket = static_cast<intptr_t>(s);
  }
#if defined(SO_NOSIGPIPE)
  int one = 1;
  setsockopt(s, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
  // HTTP/1.0: the server closes the connection after the answer and does not
  // send it in chunks.
  std::string req = job.method + " " + job.path + " HTTP/1.0\r\n";
  req += "Host: " + host + ":" + std::to_string(port) + "\r\n";
  req += "Accept: application/json\r\n";
  if (!token.empty()) req += "Authorization: Bearer " + token + "\r\n";
  if (job.method != "GET") {
    req += "Content-Type: application/json\r\n";
    req += "Content-Length: " + std::to_string(job.body.size()) + "\r\n";
  }
  req += "Connection: close\r\n\r\n";
  if (job.method != "GET") req += job.body;
  bool sent = SendAll(s, req);
  std::string answer;
  char buf[4096];
  while (sent) {
    auto n = recv(s, buf, sizeof buf, 0);
    if (n <= 0) break;
    answer.append(buf, static_cast<size_t>(n));
    if (answer.size() > 1000000) break;  // a pilot status is never this big
  }
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_socket = -1;
  }
  CloseSocket(s);
  // "HTTP/1.1 200 OK\r\n...\r\n\r\nbody"
  if (answer.compare(0, 5, "HTTP/") != 0) {
    // Connected and (perhaps partly) sent, but no answer in time: not the
    // same as "not reachable", the server may have carried it out.
    r.status = kNoAnswer;
    return r;
  }
  size_t sp = answer.find(' ');
  if (sp != std::string::npos) r.status = std::atoi(answer.c_str() + sp + 1);
  size_t head = answer.find("\r\n\r\n");
  if (head != std::string::npos) r.body = answer.substr(head + 4);
  return r;
}
