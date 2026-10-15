#include "interface/stdio_channel.hpp"
#include <roboframe/core/server.hpp>
#include <algorithm>
#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>

void Check(bool ok) { if (!ok) throw std::runtime_error("stdio worker check failed"); }
struct Pipe {
    int fd[2];
    Pipe() { Check(::pipe(fd)==0); }
    ~Pipe() { ::close(fd[0]);::close(fd[1]); }
};
std::string Read(int fd) {
    std::string result;
    char buffer[4096];
    for (;;) {
        const auto n=::read(fd,buffer,sizeof(buffer));
        if (n>0) result.append(buffer,n);
        else if (n<0 && errno==EINTR) continue;
        else { Check(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK));return result; }
    }
}
int main() {
    std::signal(SIGPIPE,SIG_IGN);
    Pipe input,output;
    Check(::fcntl(output.fd[0],F_SETFL,O_NONBLOCK)==0);
    roboframe::StdioChannel channel(input.fd[0],output.fd[1]);
    using S=roboframe::Server<int,int,int>;
    S server(std::chrono::milliseconds(10),{{},
        [&](const int& fault,const S::Context&) {
            channel.Queue("1","value");
            if (fault) throw std::runtime_error("test failure");
            return S::Step{roboframe::TaskState::running,1,0};
        },
        [&] { channel.Queue("0","value"); }});
    server.Activate();
    auto handle=server.Submit(0);
    // No Poll, Drain or other owner service runs while the owner is blocked.
    std::this_thread::sleep_for(std::chrono::milliseconds(180));
    const auto ongoing=Read(output.fd[0]);
    Check(std::count(ongoing.begin(),ongoing.end(),'\n')>=3);
    Check(server.RequestCancel(handle));
    server.WaitIdle();channel.Drain();
    const auto stopped=Read(output.fd[0]);
    Check(stopped.size()>=2 && stopped.substr(stopped.size()-2)=="0\n");
    S::Result result;
    Check(server.TakeResult(handle,result) && result.state==roboframe::TaskState::canceled);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    Check(Read(output.fd[0]).empty());

    handle=server.Submit(1);
    server.WaitIdle();channel.Drain();
    Check(server.TakeResult(handle,result) && result.state==roboframe::TaskState::failed && bool(result.error));
    const auto failed=Read(output.fd[0]);
    Check(failed.size()>=2 && failed.substr(failed.size()-2)=="0\n");
    server.Shutdown();
}
