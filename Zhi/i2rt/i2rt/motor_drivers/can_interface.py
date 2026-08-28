import logging
import time
from typing import List, Optional

import can

from i2rt.motor_drivers.utils import ReceiveMode


class CanInterface:
    def __init__(
        self,
        channel: str = "PCAN_USBBUS1",
        bustype: str = "socketcan",
        bitrate: int = 1000000,
        name: str = "default_can_interface",
        receive_mode: ReceiveMode = ReceiveMode.p16,
        use_buffered_reader: bool = False,
    ):
        self.channel = channel
        # Kept so reconnect() can rebuild an identical bus after the underlying
        # netdevice disappears (see reconnect).
        self.bustype = bustype
        self.bitrate = bitrate
        self.bus = can.interface.Bus(bustype=bustype, channel=channel, bitrate=bitrate)
        self._arm_rcv_timeout()
        self.busstate = self.bus.state
        self.name = name
        self.receive_mode = receive_mode
        self.use_buffered_reader = use_buffered_reader
        logging.info(f"Can interface {self.name} use_buffered_reader: {use_buffered_reader}")
        if use_buffered_reader:
            # Initialize BufferedReader for asynchronous message handling
            self.buffered_reader = can.BufferedReader()
            self.notifier = can.Notifier(self.bus, [self.buffered_reader])


    def _arm_rcv_timeout(self) -> None:
        """Set a kernel-level receive timeout on the CAN socket.

        THE silent-zombie mechanism, caught in a live py-spy dump on
        2026-08-25: when the USB-CAN adapter re-enumerates, select() wakes
        spuriously on the dying socket and the following recvfrom() blocks
        FOREVER in the kernel -- below every Python timeout, log call and
        recovery path in this codebase. The control thread freezes silently
        and the follower becomes a zombie with an empty log.

        SO_RCVTIMEO makes the kernel itself abort the recvfrom after 1s with
        EAGAIN, which surfaces as an exception the control loop's link-failure
        recovery already knows how to handle: rebuild the socket, re-enable
        the motors, carry on.
        """
        import socket as _socket
        import struct as _struct
        try:
            self.bus.socket.setsockopt(
                _socket.SOL_SOCKET, _socket.SO_RCVTIMEO, _struct.pack("ll", 1, 0))
        except (AttributeError, OSError):
            pass  # non-socketcan bus (e.g. PCAN); nothing to arm

    def reconnect(self) -> bool:
        """Tear down and rebuild the CAN socket for this channel.

        A CAN_RAW socket is bound to an *ifindex*, not to an interface name. When a
        USB-CAN adapter re-enumerates mid-session (the failure mode behind the
        long-teleop dropouts: the adapter briefly falls off the bus and comes back
        with the same name but a fresh ifindex) every send on the old socket fails
        with ENXIO - "No such device or address" - forever. `ip link show` looks
        perfectly healthy the whole time, which is why this used to look like a
        software hang rather than a cable problem.

        Rebuilding the socket rebinds to the current ifindex, so the chain can carry
        on driving the arm instead of dying and leaving a powered follower with no
        one at the controls. Returns True if a fresh bus is now in place.
        """
        try:
            if self.use_buffered_reader:
                try:
                    self.notifier.stop()
                except Exception:  # noqa: BLE001 - a dead notifier must not block the retry
                    pass
            try:
                self.bus.shutdown()
            except Exception:  # noqa: BLE001 - the old socket is already broken; nothing to salvage
                pass
            self.bus = can.interface.Bus(bustype=self.bustype, channel=self.channel, bitrate=self.bitrate)
            self._arm_rcv_timeout()
            self.busstate = self.bus.state
            if self.use_buffered_reader:
                self.buffered_reader = can.BufferedReader()
                self.notifier = can.Notifier(self.bus, [self.buffered_reader])
            logging.error(f"CAN interface {self.name} reconnected on channel {self.channel}")
            return True
        except Exception as e:  # noqa: BLE001 - caller retries; never raise out of a recovery path
            logging.error(f"CAN interface {self.name} reconnect on {self.channel} failed: {e}")
            return False

    def close(self) -> None:
        """Shut down the CAN bus."""
        if self.use_buffered_reader:
            self.notifier.stop()
        self.bus.shutdown()

    def _send_message_get_response(
        self, id: int, motor_id: int, data: List[int], max_retry: int = 5, expected_id: Optional[int] = None
    ) -> can.Message:
        """Send a message over the CAN bus.

        Args:
            id (int): The arbitration ID of the message.
            data (List[int]): The data payload of the message.

        Returns:
            can.Message: The message that was sent.
        """
        message = can.Message(arbitration_id=id, data=data, is_extended_id=False)
        for _ in range(max_retry):
            try:
                # logging.info("Sending message: %s at %f", message, time.time())
                self.bus.send(message)
                response = self._receive_message(motor_id, timeout=0.01)
                # logging.info("Received response: %s at %f", response, time.time())

                if expected_id is None:
                    expected_id = self.receive_mode.get_receive_id(motor_id)
                if response and (expected_id == response.arbitration_id):
                    return response
                self.try_receive_message(id)
            except (can.CanError, AssertionError) as e:
                logging.warning(e)
                logging.warning(
                    "\033[91m"
                    + f"CAN Error {self.name}: Failed to communicate with motor {id} over can bus. Retrying..."
                    + "\033[0m"
                )
            time.sleep(0.001)
        raise AssertionError(
            f"fail to communicate with the motor {id} on {self.name} at can channel {self.bus.channel_info}"
        )

    def try_receive_message(self, motor_id: Optional[int] = None, timeout: float = 0.009) -> Optional[can.Message]:
        """Try to receive a message from the CAN bus.

        Args:
            timeout (float): The time to wait for a message (in seconds).

        Returns:
            can.Message: The received message, or None if no message is received.
        """
        try:
            return self._receive_message(motor_id, timeout, supress_warning=True)
        except AssertionError:
            return None

    def _drain_bus(self, timeout_s: float = 0.05, idle_count: int = 10) -> int:
        """Drain pending CAN frames until the bus is idle.

        Loops `try_receive_message(timeout=0.001)` until either `idle_count`
        consecutive 1 ms reads return None or `timeout_s` wall-clock has
        elapsed. Used at init handovers (e.g. between encoder validation and
        motor bring-up) to flush stale frames that would otherwise be misread
        as the next motor's reply. Returns the number of frames consumed.
        """
        drained = 0
        idle = 0
        deadline = time.time() + timeout_s
        while time.time() < deadline and idle < idle_count:
            if self.try_receive_message(timeout=0.001) is None:
                idle += 1
            else:
                idle = 0
                drained += 1
        return drained

    def _receive_message(
        self, motor_id: Optional[int] = None, timeout: float = 0.009, supress_warning: bool = False
    ) -> Optional[can.Message]:
        """Receive a message from the CAN bus.

        Args:
            timeout (float): The time to wait for a message (in seconds).

        Returns:
            can.Message: The received message.

        Raises:
            AssertionError: If no message is received within the timeout.
        """
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.use_buffered_reader:
                message = self.buffered_reader.get_message(timeout=0.001)
            else:
                message = self.bus.recv(timeout=0.001)
            if message:
                return message
        if not supress_warning:
            logging.warning(
                "\033[91m"
                + f"Failed to receive message, {self.name} motor id {motor_id} motor timeout. Check if the motor is powered on or if the motor ID exists."
                + "\033[0m"
            )
