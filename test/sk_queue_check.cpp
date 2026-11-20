/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 *
 *   Checks the order of SignalKClient's requests against a slow server
 *   (test/sim/slow_sk.py): a poll is running, an NFU direction is queued,
 *   then a stop comes with RequestFirst. The stop must go out right after the
 *   poll and the queued direction must be dropped. Also: a PUT answered too
 *   late is reported as kNoAnswer, not as "not reachable" (0).
 ***************************************************************************/

#include <cstdio>
#include <string>
#include <vector>

#include <wx/app.h>
#include <wx/init.h>
#include <wx/timer.h>

#include "signalk_client.h"

class CheckApp : public wxAppConsole {
public:
  bool OnInit() override {
    if (argc < 3) {
      std::printf("usage: host port   (server: test/sim/slow_sk.py <get s> <put s> <port>)\n");
      return false;
    }
    m_client = new SignalKClient(this);
    m_client->SetServer(argv[1].ToStdString(), std::atoi(argv[2].mb_str()), "");
    const std::string nfu = "/signalk/v1/api/vessels/self/steering/autopilot/actions/nfu";
    auto log = [this](const char* what) {
      return [this, what](const SignalKClient::Response& r) {
        std::printf("%s: status %d\n", what, r.status);
        if (++m_done == 3) ExitMainLoop();
      };
    };
    m_client->Request("GET", "/signalk/v1/api/vessels/self/steering", "", log("poll"));
    m_client->Request("PUT", nfu, "{\"value\":\"port\"}", log("direction"));
    m_client->RequestFirst("PUT", nfu, "{\"value\":\"stop\"}", log("stop"));
    m_timeout.SetOwner(this);
    Bind(wxEVT_TIMER, [this](wxTimerEvent&) {
      std::printf("timeout\n");
      ExitMainLoop();
    });
    m_timeout.StartOnce(20000);
    return true;
  }
  int OnExit() override {
    delete m_client;
    return 0;
  }

private:
  SignalKClient* m_client = nullptr;
  wxTimer m_timeout;
  int m_done = 0;
};

wxIMPLEMENT_APP_CONSOLE(CheckApp);
