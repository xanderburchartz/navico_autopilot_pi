/***************************************************************************
 *   navico_autopilot_pi: check of the Signal K client against a server
 *   SPDX-License-Identifier: GPL-3.0-or-later
 *
 *   Runs SignalKClient (the plugin's own code) in a console wx event loop:
 *     sk_client_check host port GET  /path
 *     sk_client_check host port PUT  /path '{"value":"heading"}' [token]
 *     sk_client_check host port POST /path '{"...":"..."}'
 *   Prints the status and the body. Only for tests against a local server.
 ***************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <string>

#include <wx/app.h>
#include <wx/init.h>
#include <wx/timer.h>

#include "signalk_client.h"

class CheckApp : public wxAppConsole {
public:
  bool OnInit() override {
    if (argc < 5) {
      std::printf("usage: host port method path [body] [token]\n");
      return false;
    }
    std::string host = argv[1].ToStdString();
    int port = std::atoi(argv[2].mb_str());
    std::string method = argv[3].ToStdString();
    std::string path = argv[4].ToStdString();
    std::string body = argc > 5 ? argv[5].ToStdString() : "";
    std::string token = argc > 6 ? argv[6].ToStdString() : "";
    m_client = new SignalKClient(this);
    m_client->SetServer(host, port, token);
    m_client->Request(method, path, body, [this](const SignalKClient::Response& r) {
      std::printf("status %d\n%s\n", r.status, r.body.c_str());
      m_status = r.status;
      ExitMainLoop();
    });
    m_timeout.SetOwner(this);
    Bind(wxEVT_TIMER, [this](wxTimerEvent&) {
      std::printf("timeout\n");
      ExitMainLoop();
    });
    m_timeout.StartOnce(10000);
    return true;
  }
  int OnExit() override {
    delete m_client;
    return 0;
  }
  int m_status = 0;

private:
  SignalKClient* m_client = nullptr;
  wxTimer m_timeout;
};

wxIMPLEMENT_APP_CONSOLE(CheckApp);
