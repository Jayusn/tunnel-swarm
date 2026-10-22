#!/usr/bin/env python3
"""Execute the production control callback with queued newer sensor inputs."""
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]


class InputCallbackPriorityTest(unittest.TestCase):
    def test_pending_inputs_are_consumed_before_control_without_concurrent_fsm_access(self):
        source = (PACKAGE / 'src/mpc_controller_node.cpp').read_text()
        callbacks = source[source.index('ros::CallbackQueue input_callback_queue;'):
                           source.index('int main(')]
        preamble = r'''
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <chrono>
#include <memory>
#include <thread>
#include <cassert>
namespace PayloadMPC {
struct MPCFSM {
    int last_input=0, solved_input=0, observer_input=0;
    std::thread::id owner=std::this_thread::get_id();
    void process() { assert(owner==std::this_thread::get_id()); solved_input=last_input; }
    void beginInputDiagnostics(double) { assert(owner==std::this_thread::get_id()); }
    void publishInputDiagnostics(double) { assert(owner==std::this_thread::get_id()); }
    void addNewForceObseverState() {
        assert(owner==std::this_thread::get_id()); observer_input=last_input;
    }
};
}
'''
        checks = r'''
struct Input : ros::CallbackInterface {
    int value;
    explicit Input(int v):value(v) {}
    CallResult call() override { fsm_ptr->last_input=value; return Success; }
};
struct ControlTimer : ros::CallbackInterface {
    CallResult call() override { MPC_controller_main(ros::TimerEvent{}); return Success; }
};
int main() {
    ros::Time::init();
    fsm_ptr.reset(new PayloadMPC::MPCFSM);
    ros::CallbackQueue timers;
    // The timer becomes ready before the latest sensor callbacks. A shared FIFO
    // would solve using input 0, then handle inputs 1 and 2 after the solve.
    timers.addCallback(ros::CallbackInterfacePtr(new ControlTimer));
    input_callback_queue.addCallback(ros::CallbackInterfacePtr(new Input(1)));
    input_callback_queue.addCallback(ros::CallbackInterfacePtr(new Input(2)));
    assert(timers.callOne()==ros::CallbackQueue::Called);
    assert(fsm_ptr->solved_input==2 && input_callback_queue.isEmpty());
    // New input waiting after an observer calculation must be seen by the next solve.
    system_state_update_main(ros::TimerEvent{});
    assert(fsm_ptr->observer_input==2);
    input_callback_queue.addCallback(ros::CallbackInterfacePtr(new Input(3)));
    MPC_controller_main(ros::TimerEvent{});
    assert(fsm_ptr->solved_input==3);
    MPC_controller_main(ros::TimerEvent{});
    assert(fsm_ptr->solved_input==3);
}
'''
        flags = shlex.split(subprocess.check_output(
            ['pkg-config', '--cflags', '--libs', 'roscpp'], text=True))
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'test.cpp'
            executable = Path(directory) / 'test'
            cpp.write_text(preamble + callbacks + checks)
            subprocess.run(['c++', '-std=c++14', str(cpp), '-o', str(executable)] + flags,
                           check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    unittest.main()
