"""Run the installed combined launch on localhost and exercise terminal Ctrl+C."""

import os
import signal
import socket
import subprocess
import time


def test_combined_launch_starts_disabled_and_exits_on_group_sigint(tmp_path):
    sockets = [socket.socket(), socket.socket()]
    for sock in sockets:
        sock.bind(("127.0.0.1", 0))
    ports = [sock.getsockname()[1] for sock in sockets]
    env = dict(os.environ, ROS_DOMAIN_ID="216", ROS_LOCALHOST_ONLY="1",
               ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    env.pop("ROS_DISCOVERY_SERVER", None)
    env.pop("ROS_STATIC_PEERS", None)
    output_path = tmp_path / "launch.log"
    command = ["ros2", "launch", "g1_pico_teleop", "pico_body_hands.launch.py",
               f"body_zmq_endpoint:=tcp://127.0.0.1:{ports[0]}",
               f"hand_zmq_endpoint:=tcp://127.0.0.1:{ports[1]}"]
    process = None
    try:
        with output_path.open("w") as output:
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                       env=env, start_new_session=True)
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                text = output_path.read_text()
                if "body output disabled" in text and "output is disabled" in text:
                    break
                assert process.poll() is None, text
                time.sleep(0.05)
            else:
                raise AssertionError(output_path.read_text())
            os.killpg(process.pid, signal.SIGINT)
            assert process.wait(timeout=12) == 0, output_path.read_text()
        text = output_path.read_text()
        assert "Traceback" not in text and "process has died" not in text, text
        assert text.count("process has finished cleanly") == 2, text
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        for sock in sockets:
            sock.close()
