#include "vicon_receiver/capture_queue.hpp"
#include <cassert>
#include <thread>

int main() {
    vicon_capture::Queue<3> small;
    vicon_capture::Frame in, out;
    assert(!small.pop(out));
    in.sequence=1; in.sdk_frame=99; in.sdk_pose[3]=.123; in.world_pose[1]=.456;
    assert(small.push(in));
    in.sequence=2; assert(small.push(in));
    in.sequence=3; assert(!small.push(in)); // no blocking, explicit overflow
    assert(small.pop(out) && out.sequence==1 && out.sdk_frame==99);
    assert(out.sdk_pose[3]==.123 && out.world_pose[1]==.456);
    assert(small.push(in));
    assert(small.pop(out) && out.sequence==2);
    assert(small.pop(out) && out.sequence==3 && out.queue_dropped==1);
    assert(!small.pop(out));
    vicon_capture::Queue<257> threaded;
    std::thread producer([&] {
        for (uint64_t i=1; i<=100000; ++i) {
            vicon_capture::Frame row;
            row.sequence=i; row.sdk_frame=i*2; row.world_pose[0]=double(i);
            while (!threaded.push(row)) std::this_thread::yield();
        }
    });
    for (uint64_t i=1; i<=100000; ++i) {
        while (!threaded.pop(out)) std::this_thread::yield();
        assert(out.sequence==i && out.sdk_frame==i*2 && out.world_pose[0]==double(i));
    }
    producer.join();
}
