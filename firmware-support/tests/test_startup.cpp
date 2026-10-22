#include "../sketch/route_refresh.h"
#include <cstdio>
#include <stdexcept>
#include <vector>
static unsigned passed=0;
static void check(bool v,const char *m){if(!v)throw std::runtime_error(m);++passed;}
struct Port {
  uint32_t now=0,routerReadyAt=0;bool unavailable=false,prepared=false;
  unsigned allocations=0,begins=0,reports=0,reportFailures=0,bindFailures=0;
  unsigned failIndex=3;std::vector<unsigned> binds;
  bool beginBounded(uint32_t ms){check(ms==200,"startup uses bounded maintenance API");++begins;if(!prepared){prepared=true;++allocations;}return !unavailable&&now>=routerReadyAt;}
  bool bindLocal(unsigned i){binds.push_back(i);if(i==failIndex&&bindFailures){--bindFailures;return false;}return true;}
  bool startReports(){if(reportFailures){--reportFailures;return false;}++reports;return true;}
};
static void tick(RouteStartup &s,Port &p,uint32_t t){p.now=t;s.tick(t,p);}
int main(){try{
 {RouteStartup s;Port p;tick(s,p,0);check(s.localMask()==15&&s.reportsStarted(),"ready router starts local handlers and reports");for(unsigned t=100;t<40000;t+=100)tick(s,p,t);check(p.begins==1&&p.allocations==1&&p.reports==1,"already started never begin/reset or create another report thread");check(p.binds==std::vector<unsigned>({0,1,2,3}),"four local callbacks exactly once");}
 for(uint32_t ready:{10000u,30000u}){RouteStartup s;Port p;p.routerReadyAt=ready;
  for(uint32_t t=0;t<ready;t+=100){tick(s,p,t);check(!s.reportsStarted()&&p.binds.empty()&&p.reports==0,"late Linux has no false local readiness/report");}
  tick(s,p,ready);check(s.reportsStarted()&&p.reports==1&&p.allocations==1,"late10s/30s Linux recovers without MCU reboot/reallocation");check(p.begins==ready/1000+1,"one startup attempt per second");}
 {RouteStartup s;Port p;p.unavailable=true;for(unsigned t=0;t<=60000;t+=100)tick(s,p,t);check(!s.reportsStarted()&&s.localMask()==0&&p.reports==0,"permanent router outage never reports ready");check(p.begins==61&&p.allocations==1,"permanent failure is bounded, paced, allocated once");}
 {RouteStartup s;Port p;p.bindFailures=3;for(unsigned t=0;t<=3000;t+=100)tick(s,p,t);check(s.reportsStarted()&&p.begins==1&&p.reports==1,"local bind transient recovers without reset");unsigned seen[4]={};for(auto i:p.binds)++seen[i];check(seen[0]==1&&seen[1]==1&&seen[2]==1&&seen[3]==4,"successful callbacks are never rebound");}
 {RouteStartup s;Port p;p.bindFailures=1000;for(unsigned t=0;t<=30000;t+=100)tick(s,p,t);check(!s.reportsStarted()&&s.localMask()==7&&p.reports==0,"permanent single bind failure never starts report");check(p.begins==1,"local failure never clears router registrations");}
 {RouteStartup s;Port p;p.reportFailures=1;tick(s,p,0);check(!s.reportsStarted()&&s.localMask()==15,"report creation failure remains unready");tick(s,p,1000);check(s.reportsStarted()&&p.reports==1&&s.reportAttempts()==2,"failed creation can retry without duplicate live reports");}
 {RouteStartup s;Port p;p.unavailable=true;tick(s,p,0xfffffff0u);tick(s,p,0x100u);check(p.begins==1,"startup throttle survives millis wrap");p.unavailable=false;tick(s,p,0x500u);check(s.reportsStarted()&&p.begins==2,"post-wrap startup resumes");}
 std::printf("{\"passed_assertions\":%u,\"hardware_access\":false,\"actual_startup_header\":true}\n",passed);return 0;
}catch(const std::exception &e){std::fprintf(stderr,"FAIL %s\n",e.what());return 1;}}
