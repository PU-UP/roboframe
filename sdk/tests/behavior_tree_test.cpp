#include <roboframe/orchestration/sequence.hpp>
#include <cstring>
#include <stdexcept>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("BT sequence check failed"); }
int main() {
    int first=0,second=0,third=0,halts=0;
    bt::Status result=bt::Status::running;
    bt::Sequence tree({
        {"first",[&] { ++first; return bt::Status::success; },[&] { ++halts; }},
        {"second",[&] { ++second; return result; },[&] { ++halts; }},
        {"third",[&] { ++third; return bt::Status::success; },{}}
    });
    Check(tree.status().status==bt::Status::idle);
    Check(tree.Tick()==bt::Status::running);
    Check(std::strcmp(tree.status().node,"second")==0 && first==1 && second==1 && third==0);
    tree.Tick(); Check(first==1 && second==2); // Memory sequence does not repeat completed actions.
    tree.Halt(); tree.Halt(); Check(halts==1 && tree.status().status==bt::Status::idle);
    result=bt::Status::failure;
    Check(tree.Tick()==bt::Status::failure && first==2 && second==3 && third==0);
    tree.Tick(); Check(second==3); // Failure is terminal, not an implicit retry.
    tree.Halt(); Check(halts==1);
    result=bt::Status::success;
    Check(tree.Tick()==bt::Status::success && first==3 && second==4 && third==1);
    tree.Tick(); Check(third==1); // Completion is emitted once.
    tree.Halt(); Check(halts==1);
    result=bt::Status::idle;
    bool rejected=false;
    try { tree.Tick(); } catch (const std::logic_error&) { rejected=true; }
    Check(rejected);
    tree.Halt(); Check(halts==2);
}
