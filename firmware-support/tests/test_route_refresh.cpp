#include "../sketch/route_refresh.h"
#include <cstdio>
#include <stdexcept>
#include <set>
#include <string>
#include <vector>

static void check(bool value,const char *message) {if(!value) throw std::runtime_error(message);}
struct Port {
  bool allow=true, reply=true;
  unsigned stopAfter=0;
  std::set<std::string> routes;
  std::vector<std::string> calls;
  bool allowed() const {return allow;}
  bool registerOnly(const char *name,uint32_t ms) {
    check(ms==200,"bounded registration budget");
    calls.emplace_back(name);
    bool accepted=reply && routes.insert(name).second;
    if(stopAfter && calls.size()==stopAfter) allow=false;
    return accepted;
  }
};
int main() {
  unsigned checks=0;
  try {
    check(routeRefreshAllowed(1,1,10000,10001),"fresh unloaded normal state");++checks;
    for(uint8_t flags:{uint8_t(2),uint8_t(8),uint8_t(16),uint8_t(26)}) {
      check(!routeRefreshAllowed(1,flags,10000,10001),"enabled or maintenance skips");++checks;
    }
    check(!routeRefreshAllowed(0,0,0,1),"unpublished state skips");++checks;
    check(!routeRefreshAllowed(1,0,1,50002),"stale state skips");++checks;
    check(routeRefreshAllowed(1,0,0xfffffff0u,0x20u),"sample clock wrap");++checks;
    {
      RouteRefresh r;Port p;
      check(r.tick(0,p),"first quiet refresh");
      check(p.calls==std::vector<std::string>({"duck_command","duck_cal","duck_servo","duck_factory"}),"exact fixed route set");
      check(r.attempts()==4 && r.accepted()==4 && r.lastAcceptedMask()==15,"all returned booleans checked");
      check(!r.tick(999,p),"one second throttle");
      check(r.tick(1000,p) && r.notConfirmed()==4,"duplicate reply not misreported accepted");
      p.routes.clear();
      check(r.tick(2000,p) && p.routes.size()==4 && r.accepted()==8,"serial-route loss repaired without local rebind/reset");++checks;
    }
    {
      RouteRefresh r;Port p;p.allow=false;
      check(!r.tick(0,p) && p.calls.empty() && r.attempts()==0,"ineligible has no bridge call");
      p.allow=true;check(r.tick(1,p) && p.calls.size()==4,"becoming unloaded permits first refresh");++checks;
    }
    {
      RouteRefresh r;Port p;p.stopAfter=1;
      check(r.tick(0,p) && p.calls.size()==1 && r.interrupted()==1,"gate rechecked before every route");++checks;
    }
    {
      RouteRefresh r;Port p;p.reply=false;
      check(r.tick(0,p) && r.notConfirmed()==4 && r.lastAcceptedMask()==0,"lost ACK leaves unconfirmed status");
      check(!r.tick(500,p),"failure does not create busy retry");
      p.reply=true;check(r.tick(1000,p) && r.lastAcceptedMask()==15,"retry later after loss");++checks;
    }
    {
      RouteRefresh r;Port p;
      check(r.tick(0xfffffff0u,p),"period test starts near wrap");
      check(!r.tick(0x100u,p),"period wrap preserves throttle");
      check(r.tick(0x500u,p),"period wrap allows later attempt");++checks;
    }
    std::printf("{\"passed\":%u,\"hardware_access\":false,\"actual_candidate_header\":true}\n",checks);
    return 0;
  } catch(const std::exception &e) {std::fprintf(stderr,"FAIL: %s\n",e.what());return 1;}
}
